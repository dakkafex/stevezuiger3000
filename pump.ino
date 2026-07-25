/*
  ESP32-s3 BLE Switch Scheduler
  -----------------------------
  Replaces a WiFi-AP + webserver switch/timer with a low-power BLE setup.

  - Advertises over BLE periodically (no WiFi AP, no webserver).
  - You set a weekly schedule (day-of-week + time + duration) from the
    matching schedule_control.html page using Web Bluetooth.
  - Also supports manual ON/OFF (with an optional requested runtime) and
    a momentary test trigger, independent of the schedule. Manual ON/OFF
    state survives deep sleep via GPIO hold.
  - SAFETY: manual ON always auto-shuts-off after MAX_MANUAL_ON_SECONDS,
    enforced by the firmware itself (the device wakes specifically to do
    this) - so a pump/relay can never be left running indefinitely just
    because the app was closed or a connection dropped.
  - Schedule is stored in flash (NVS) - survives reboot/power loss.
  - Current time is kept on a DS3231 RTC module (battery-backed - keeps
    correct time through full power loss, not just deep sleep), synced
    from your phone/laptop the first time you connect.
  - Device deep-sleeps between events to save power, waking exactly when
    needed (next scheduled event, or the next periodic "connect window").

  REQUIRED LIBRARIES (Arduino Library Manager):
    - "NimBLE-Arduino" by h2zero
    - "ArduinoJson" by Benoit Blanchon
    - "RTClib" by Adafruit (also pulls in "Adafruit BusIO")

  HARDWARE:
    - DS3231 RTC module wired over I2C: SDA -> I2C_SDA_PIN, SCL -> I2C_SCL_PIN,
      plus VCC/GND. Keep its coin cell installed.

  BOARD: ESP32C3 Dev Module (or your specific C3 board)
*/

#include <NimBLEDevice.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <RTClib.h>
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_system.h"
#include <time.h>

// ---------------- USER CONFIG ----------------
#define RELAY_PIN            38       // GPIO driving your switch/pump MOSFET gate (safe general-purpose pin, no flash/PSRAM conflict)
#define RELAY_ACTIVE_HIGH     true     // set false if your relay is active-low
#define ADVERTISE_WINDOW_SEC  120       // how long BLE stays advertising when it wakes to let you connect
#define ADVERTISE_INTERVAL_SEC (4UL * 3600UL) // periodic "connect window" even with no scheduled event (4h)
#define MAX_SCHEDULE_JSON_LEN 4000      // max stored schedule string length
#define I2C_SDA_PIN           8        // DS3231 SDA - adjust to your wiring
#define I2C_SCL_PIN           9        // DS3231 SCL - adjust to your wiring
#define MAX_MANUAL_ON_SECONDS (20UL * 60UL) // HARD safety ceiling: manual ON auto-shuts-off after this long, no matter what
#define DEBUG_SERIAL          true      // set false once things are working, to save a little power/time
#define DEBUG_BAUD            115200

#if DEBUG_SERIAL
  #define DBG(...) Serial.printf(__VA_ARGS__)
#else
  #define DBG(...)
#endif

// ---------------- BLE UUIDs ----------------
#define SERVICE_UUID         "12345678-1234-1234-1234-123456789abc"
#define SCHEDULE_CHAR_UUID   "12345678-1234-1234-1234-123456789ab1" // read/write JSON schedule
#define TIME_CHAR_UUID       "12345678-1234-1234-1234-123456789ab2" // write epoch seconds (as string) to sync clock
#define STATUS_CHAR_UUID     "12345678-1234-1234-1234-123456789ab3" // read/notify status JSON
#define COMMAND_CHAR_UUID    "12345678-1234-1234-1234-123456789ab4" // write "TRIGGER" for manual activation

// ---------------- Persistent (RTC memory) state - survives deep sleep ----------------
// Only the wake-reason flag needs to survive deep sleep now; the clock
// itself lives on the DS3231, which survives full power loss too.
RTC_DATA_ATTR uint8_t  rtcWakeReason = 0;    // 0=boot/unknown, 1=event, 2=advertise-window, 3=auto-off safety
RTC_DATA_ATTR bool     relayManualOn = false; // persists manual ON/OFF across deep sleep
RTC_DATA_ATTR time_t   manualOffDeadline = 0; // epoch when manual ON must auto-shut-off; 0 = no deadline active

enum WakeReason { WAKE_UNKNOWN = 0, WAKE_EVENT = 1, WAKE_ADVERTISE = 2, WAKE_AUTO_OFF = 3 };

Preferences prefs;
NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pScheduleChar = nullptr;
NimBLECharacteristic* pTimeChar = nullptr;
NimBLECharacteristic* pStatusChar = nullptr;
NimBLECharacteristic* pCommandChar = nullptr;

RTC_DS3231 rtc;
bool rtcAvailable = false; // true if the DS3231 was found on the I2C bus at boot

bool bleActivityHappened = false; // set true on any write, so we know to recompute + maybe extend window

// ---------------- Time helpers ----------------
// Time now comes from the DS3231 rather than RTC memory, so it is valid
// immediately after ANY boot (power-on, reset, or deep sleep wake) as
// long as the module has been set at least once and its battery is good.
bool isTimeValid() {
  if (!rtcAvailable) return false;
  return !rtc.lostPower(); // lostPower() is true if battery was ever depleted/removed and it's never been set
}

time_t getCurrentEpoch() {
  if (!rtcAvailable) return 0;
  return rtc.now().unixtime();
}

void setCurrentEpoch(time_t newEpoch) {
  if (!rtcAvailable) return;
  rtc.adjust(DateTime((uint32_t)newEpoch));
}

// ---------------- Schedule model ----------------
// Stored as JSON array in NVS under key "schedule":
// [{"id":1,"days":[0,1,2,3,4,5,6],"hour":7,"minute":30,"duration":300,"enabled":true}, ...]
// days: 0=Sunday ... 6=Saturday  (matches JS Date.getDay())
// NOTE: the ESP32 has no timezone concept. The web page sends a
// timestamp constructed so that gmtime() on the device lines up with
// your intended LOCAL wall-clock day/hour/minute. See the HTML file.

String loadScheduleJson() {
  prefs.begin("sched", true); // read-only
  String s = prefs.getString("schedule", "[]");
  prefs.end();
  return s;
}

void saveScheduleJson(const String& json) {
  prefs.begin("sched", false);
  prefs.putString("schedule", json);
  prefs.end();
}

// Find the next occurrence (epoch seconds) of ANY enabled schedule entry
// at or after fromEpoch. Returns 0 if none found (empty/disabled schedule).
// NOTE: deliberately avoids timegm() (not available on all ESP32 toolchains)
// by doing the day/hour/minute math directly in epoch seconds instead of
// via struct tm - this also sidesteps any month/day rollover edge cases.
time_t computeNextEvent(const String& scheduleJson, time_t fromEpoch, JsonVariant* outEntry, DynamicJsonDocument* doc) {
  deserializeJson(*doc, scheduleJson);
  JsonArray arr = doc->as<JsonArray>();

  time_t best = 0;
  JsonVariant bestEntry;

  struct tm baseTm;
  gmtime_r(&fromEpoch, &baseTm);
  // Epoch of midnight (00:00:00 UTC) on the current day - everything else
  // is just adding whole days/hours/minutes in seconds from here.
  time_t todayMidnight = fromEpoch - (baseTm.tm_hour * 3600 + baseTm.tm_min * 60 + baseTm.tm_sec);

  for (JsonVariant entry : arr) {
    if (!entry["enabled"].as<bool>()) continue;
    int hour = entry["hour"].as<int>();
    int minute = entry["minute"].as<int>();
    JsonArray days = entry["days"].as<JsonArray>();

    for (int dayOffset = 0; dayOffset < 8; dayOffset++) {
      time_t candidate = todayMidnight + (time_t)dayOffset * 86400L + (time_t)hour * 3600L + (time_t)minute * 60L;

      if (candidate < fromEpoch) continue;

      struct tm normTm;
      gmtime_r(&candidate, &normTm);
      int wday = normTm.tm_wday; // 0=Sunday

      bool dayMatches = false;
      for (JsonVariant d : days) {
        if (d.as<int>() == wday) { dayMatches = true; break; }
      }
      if (!dayMatches) continue;

      if (best == 0 || candidate < best) {
        best = candidate;
        bestEntry = entry;
      }
      break; // found the earliest valid day-offset for this entry, no need to check further offsets
    }
  }

  if (outEntry) *outEntry = bestEntry;
  return best;
}

// ---------------- Relay control ----------------
void relaySet(bool on) {
  bool level = RELAY_ACTIVE_HIGH ? on : !on;
  digitalWrite(RELAY_PIN, level ? HIGH : LOW);
}

void firePulse(int durationSec) {
  relaySet(true);
  delay((unsigned long)durationSec * 1000UL);
  relaySet(relayManualOn); // return to whatever manual state was set, not just off
}

// ---------------- BLE callbacks ----------------
class ServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* server, NimBLEConnInfo& connInfo) override {
    DBG("[BLE] Client CONNECTED, handle=%d, addr=%s\n",
        connInfo.getConnHandle(), connInfo.getAddress().toString().c_str());
    bleActivityHappened = true;
  }
  void onDisconnect(NimBLEServer* server, NimBLEConnInfo& connInfo, int reason) override {
    DBG("[BLE] Client DISCONNECTED, handle=%d, reason=0x%02X\n",
        connInfo.getConnHandle(), reason);
    // Restart advertising so a fresh connection attempt can still succeed
    // during the remainder of the current advertise window.
    NimBLEDevice::getAdvertising()->start();
    DBG("[BLE] Advertising restarted after disconnect\n");
  }
};

class ScheduleCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
    std::string val = c->getValue();
    String json = String(val.c_str());
    DBG("[BLE] Schedule WRITE received, %d bytes\n", json.length());
    if (json.length() > 0 && json.length() < MAX_SCHEDULE_JSON_LEN) {
      saveScheduleJson(json);
      bleActivityHappened = true;
      DBG("[BLE] Schedule saved to NVS\n");
    } else {
      DBG("[BLE] Schedule WRITE rejected (empty or too long)\n");
    }
  }
  void onRead(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
    String s = loadScheduleJson();
    c->setValue(s.c_str());
    DBG("[BLE] Schedule READ served, %d bytes\n", s.length());
  }
};

class TimeCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
    std::string val = c->getValue();
    DBG("[BLE] Time WRITE received: '%s'\n", val.c_str());
    if (val.length() > 0) {
      time_t epoch = (time_t)strtoll(val.c_str(), nullptr, 10);
      if (epoch > 1700000000) { // sanity check: after ~Nov 2023
        setCurrentEpoch(epoch);
        bleActivityHappened = true;
        DBG("[BLE] Time synced to epoch %ld\n", (long)epoch);
      } else {
        DBG("[BLE] Time WRITE rejected (epoch %ld looks invalid)\n", (long)epoch);
      }
    }
  }
};

class CommandCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
    std::string val = c->getValue();
    String s = String(val.c_str());
    DBG("[BLE] Command WRITE received: '%s'\n", s.c_str());

    if (s == "TRIGGER") {
      DBG("[RELAY] Firing 5s test pulse\n");
      firePulse(5); // manual test pulse, 5 seconds - adjust as desired
    } else if (s == "OFF") {
      DBG("[RELAY] Manual OFF\n");
      relayManualOn = false;
      manualOffDeadline = 0;
      relaySet(false);
    } else if (s == "ON" || s.startsWith("ON:")) {
      // Optional "ON:<seconds>" lets the app request a specific runtime
      // (e.g. run the pump for 15 minutes). Always clamped to
      // MAX_MANUAL_ON_SECONDS - this is a hard ceiling enforced here in
      // firmware, not just something the app promises to respect.
      unsigned long requestedSec = MAX_MANUAL_ON_SECONDS;
      int colonIdx = s.indexOf(':');
      if (colonIdx >= 0) {
        long parsed = s.substring(colonIdx + 1).toInt();
        if (parsed > 0) requestedSec = (unsigned long)parsed;
      }
      if (requestedSec > MAX_MANUAL_ON_SECONDS) requestedSec = MAX_MANUAL_ON_SECONDS;

      relayManualOn = true;
      relaySet(true);
      if (isTimeValid()) {
        manualOffDeadline = getCurrentEpoch() + (time_t)requestedSec;
        DBG("[RELAY] Manual ON for %lu sec (auto-off at epoch %ld)\n",
            requestedSec, (long)manualOffDeadline);
      } else {
        // No valid clock (RTC missing/unset) - refuse to leave it running
        // unattended indefinitely. Fall back to the momentary pulse
        // behavior instead of an untimed ON.
        relayManualOn = false;
        manualOffDeadline = 0;
        DBG("[RELAY] No valid clock - refusing untimed ON, falling back to 5s pulse\n");
        firePulse(5);
      }
    }
    bleActivityHappened = true;
  }
};

void publishStatus() {
  DynamicJsonDocument doc(512);
  bool timeValid = isTimeValid();
  time_t now = getCurrentEpoch();
  doc["synced"] = timeValid;
  doc["rtcPresent"] = rtcAvailable;
  doc["currentEpoch"] = (long)now;
  doc["relayOn"] = relayManualOn;
  doc["manualOffDeadline"] = (long)manualOffDeadline;

  String schedJson = loadScheduleJson();
  DynamicJsonDocument schedDoc(MAX_SCHEDULE_JSON_LEN + 256);
  JsonVariant nextEntry;
  time_t nextEvent = 0;
  if (timeValid) {
    nextEvent = computeNextEvent(schedJson, now, &nextEntry, &schedDoc);
  }
  doc["nextEventEpoch"] = (long)nextEvent;

  String out;
  serializeJson(doc, out);
  pStatusChar->setValue(out.c_str());
  pStatusChar->notify();
  DBG("[BLE] Status notified: %s\n", out.c_str());
}

// ---------------- BLE setup ----------------
void startBLE() {
  DBG("[BLE] Initializing NimBLE...\n");
  NimBLEDevice::init("ESP32-Switch");
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  NimBLEService* pService = pServer->createService(SERVICE_UUID);

  pScheduleChar = pService->createCharacteristic(
    SCHEDULE_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE
  );
  pScheduleChar->setCallbacks(new ScheduleCallbacks());

  pTimeChar = pService->createCharacteristic(
    TIME_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE
  );
  pTimeChar->setCallbacks(new TimeCallbacks());

  pStatusChar = pService->createCharacteristic(
    STATUS_CHAR_UUID,
    NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
  );

  pCommandChar = pService->createCharacteristic(
    COMMAND_CHAR_UUID,
    NIMBLE_PROPERTY::WRITE
  );
  pCommandChar->setCallbacks(new CommandCallbacks());

  pService->start();

  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();
  DBG("[BLE] Advertising started as 'ESP32-Switch'\n");
}

void stopBLE() {
  DBG("[BLE] Shutting down BLE stack\n");
  NimBLEDevice::deinit(true);
}

// ---------------- Sleep planning ----------------
void goToSleepUntil(time_t targetEpoch, WakeReason reason) {
  time_t now = getCurrentEpoch();
  int64_t sleepSec = (int64_t)targetEpoch - (int64_t)now;
  if (sleepSec < 1) sleepSec = 1;

  // Hold the relay pin's current level through deep sleep if it's been
  // manually turned ON - otherwise the pin can reset/float on wake and
  // drop the switch unexpectedly.
  if (relayManualOn) {
    gpio_hold_en((gpio_num_t)RELAY_PIN);
    gpio_deep_sleep_hold_en();
  } else {
    gpio_hold_dis((gpio_num_t)RELAY_PIN);
  }

  rtcWakeReason = reason;
  DBG("[SLEEP] Going to sleep for %lld sec (reason=%d, wake at epoch %ld)\n",
      (long long)sleepSec, (int)reason, (long)targetEpoch);
  #if DEBUG_SERIAL
    Serial.flush();
  #endif
  esp_sleep_enable_timer_wakeup((uint64_t)sleepSec * 1000000ULL);
  esp_deep_sleep_start(); // does not return
}

void planNextSleepAndGo() {
  time_t now = getCurrentEpoch();
  String schedJson = loadScheduleJson();

  time_t nextAdvertise = now + ADVERTISE_INTERVAL_SEC;
  time_t nextEvent = 0;

  if (isTimeValid()) {
    DynamicJsonDocument tmpDoc(MAX_SCHEDULE_JSON_LEN + 256);
    JsonVariant entry;
    nextEvent = computeNextEvent(schedJson, now, &entry, &tmpDoc);
  }

  // The auto-off safety deadline takes priority over everything else -
  // if it's the soonest thing coming up, wake for it specifically so a
  // stuck-ON pump can never outlast MAX_MANUAL_ON_SECONDS.
  time_t candidates[3] = { nextEvent, nextAdvertise, 0 };
  WakeReason reasons[3] = { WAKE_EVENT, WAKE_ADVERTISE, WAKE_UNKNOWN };
  int count = 2;
  if (relayManualOn && manualOffDeadline != 0) {
    candidates[2] = manualOffDeadline;
    reasons[2] = WAKE_AUTO_OFF;
    count = 3;
  }

  time_t bestTime = 0;
  WakeReason bestReason = WAKE_ADVERTISE;
  for (int i = 0; i < count; i++) {
    if (candidates[i] == 0) continue; // 0 means "not applicable"
    if (bestTime == 0 || candidates[i] < bestTime) {
      bestTime = candidates[i];
      bestReason = reasons[i];
    }
  }
  if (bestTime == 0) bestTime = nextAdvertise; // fallback, should not normally happen

  DBG("[PLAN] now=%ld nextEvent=%ld nextAdvertise=%ld autoOffDeadline=%ld -> chosen=%ld reason=%d\n",
      (long)now, (long)nextEvent, (long)nextAdvertise,
      (relayManualOn ? (long)manualOffDeadline : -1L), (long)bestTime, (int)bestReason);

  goToSleepUntil(bestTime, bestReason);
}

void runAdvertiseWindow() {
  DBG("[MAIN] Opening advertise window (%d sec, extendable)\n", ADVERTISE_WINDOW_SEC);
  startBLE();
  unsigned long start = millis();
  bleActivityHappened = false;

  while (millis() - start < (ADVERTISE_WINDOW_SEC * 1000UL)) {
    int connCount = pServer ? pServer->getConnectedCount() : -1;
    DBG("[MAIN] Window tick: t=%lus connected=%d\n", (millis() - start) / 1000, connCount);
    publishStatus();
    delay(2000);
    // Extend the window a bit if something just got written, so the
    // person has time to see confirmation / do a follow-up write.
    if (bleActivityHappened && (millis() - start) > (ADVERTISE_WINDOW_SEC * 1000UL - 5000UL)) {
      start = millis() - (ADVERTISE_WINDOW_SEC * 1000UL) + 10000UL; // extend ~10s
      bleActivityHappened = false;
      DBG("[MAIN] Window extended due to recent activity\n");
    }
  }
  DBG("[MAIN] Advertise window closed\n");
  stopBLE();
}

void handleScheduledEvent() {
  time_t now = getCurrentEpoch();
  String schedJson = loadScheduleJson();
  DynamicJsonDocument doc(MAX_SCHEDULE_JSON_LEN + 256);
  JsonVariant entry;
  time_t eventTime = computeNextEvent(schedJson, now - 5, &entry, &doc); // small tolerance

  int duration = 5;
  if (!entry.isNull() && entry.containsKey("duration")) {
    duration = entry["duration"].as<int>();
  }
  DBG("[EVENT] Scheduled event firing, duration=%d sec\n", duration);
  firePulse(duration);
}

// ---------------- Arduino entry points ----------------
void setup() {
  #if DEBUG_SERIAL
    Serial.begin(DEBUG_BAUD);
    delay(300); // give the USB CDC host side a moment to attach
    Serial.println();
    Serial.println("========================================");
    Serial.println("[BOOT] ESP32 BLE Switch Scheduler starting");
  #endif

  esp_reset_reason_t resetReason = esp_reset_reason();
  DBG("[BOOT] esp_reset_reason = %d\n", (int)resetReason);
  // Common values: 1=power-on, 3=software reset, 5=deep-sleep wake,
  // 12=brownout - a repeated 12 here would point to a power problem.

  // A held pin must be released before it can be reconfigured/driven again.
  gpio_hold_dis((gpio_num_t)RELAY_PIN);
  pinMode(RELAY_PIN, OUTPUT);
  relaySet(relayManualOn); // restore whatever manual state persisted through sleep
  DBG("[BOOT] Relay pin %d set to %s (restored manual state)\n", RELAY_PIN, relayManualOn ? "ON" : "OFF");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  rtcAvailable = rtc.begin();
  DBG("[BOOT] DS3231 present=%s, lostPower=%s\n",
      rtcAvailable ? "yes" : "NO",
      rtcAvailable ? (rtc.lostPower() ? "yes (needs time sync)" : "no") : "n/a");
  // Note: rtc.lostPower() tells us if the DS3231 has never been set (or
  // its battery died) - isTimeValid() checks this. We don't need to
  // reset any local "synced" flag ourselves since the DS3231 is the
  // single source of truth for time and it persists across resets.

  if (isTimeValid()) {
    DBG("[BOOT] Current epoch = %ld\n", (long)getCurrentEpoch());
  }

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
  DBG("[BOOT] wakeup_cause=%d rtcWakeReason=%d relayManualOn=%d manualOffDeadline=%ld\n",
      (int)cause, (int)rtcWakeReason, (int)relayManualOn, (long)manualOffDeadline);

  if (cause == ESP_SLEEP_WAKEUP_TIMER && rtcWakeReason == WAKE_EVENT) {
    DBG("[BOOT] Waking to run a SCHEDULED EVENT\n");
    handleScheduledEvent();
  } else if (cause == ESP_SLEEP_WAKEUP_TIMER && rtcWakeReason == WAKE_AUTO_OFF) {
    // Safety cutoff: manual ON has run for the maximum allowed time.
    // Shut it off unconditionally, no BLE window needed for this - it's
    // a safety action, not something that should wait on a connection.
    DBG("[BOOT] Waking for AUTO-OFF SAFETY CUTOFF - shutting relay off\n");
    relayManualOn = false;
    manualOffDeadline = 0;
    relaySet(false);
    gpio_hold_dis((gpio_num_t)RELAY_PIN);
  } else {
    // Either a periodic connect window, or a fresh power-on/reset where
    // we always open a window so you can set the schedule / sync time.
    DBG("[BOOT] Opening advertise window (periodic window or fresh boot)\n");
    runAdvertiseWindow();
  }

  planNextSleepAndGo(); // never returns
}

void loop() {
  // Not used - everything happens in setup() before deep sleep.
}
