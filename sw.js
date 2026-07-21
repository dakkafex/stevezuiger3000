// Minimal service worker - just enough to satisfy Chrome's install
// criteria. We don't need offline caching since this page is only ever
// useful while actively talking to the ESP32 over Bluetooth.
self.addEventListener('fetch', () => {});
