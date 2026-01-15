# Emotiscope v1.1 HIL – Implementation Debrief & Web App Failure Analysis

## 1. Scope of Changes I’ve Made

This covers only **v1.1_build** (HIL‑extended) and the associated web UI in `v1.1_build/data/`.

---

### 1.1 WiFi Hard‑Wiring to VX220‑013F

**Goal:** Device should come up on your VX220‑013F network without any WiFi config interaction.

**What I did:**

- Introduced static default WiFi credentials (SSID and password) in the v1.1 configuration layer.
- Integrated those defaults into the existing configuration boot flow so that:
  - On boot, the firmware first attempts to load network credentials from non‑volatile storage (NVS / `network.txt`).
  - If nothing is stored, it falls back to the hard‑wired VX220‑013F credentials.
- Left the rest of the configuration behavior intact so you can still change WiFi later if needed.

**Effect:**

- The console messages you see about “CONNECTED TO VX220‑013F … NOW CONNECTED TO NETWORK” are the direct result of this wiring. The S3 is successfully joining your AP at 192.168.1.111.

---

### 1.2 Web UI Wiring for HIL Control

**Goal:** Control HIL features from the browser instead of the raw serial console.

**What I did:**

- Extended the existing web UI menu in `remote.html` to add HIL‑specific menu entries:
  - HIL status
  - Enable HIL monitoring
  - Disable HIL monitoring
  - Capture a HIL snapshot
- Hooked those UI actions into the existing WebSocket layer (`websockets_connection.js`) by:
  - Adding small helper functions that send HIL commands through the same `transmit()` pipeline used by other controls.
  - Formatting those commands in a simple HIL protocol (`hil|…`) that gets parsed on the firmware side by your command handler.
- Ensured the firmware’s command parser routes these new commands into the existing HIL state (`hil_capture_state`, `hil_monitoring_active`) rather than inventing a parallel mechanism.

**Effect:**

- Once the web app loads and the WebSocket connects, you will be able to:
  - Trigger HIL state queries.
  - Turn monitoring on/off.
  - Request snapshots, all via the browser UI, not serial.

---

### 1.3 Removal of Legacy Cloud Integration

**Goal:** Stop the device from talking to `app.emotiscope.rocks` and failing with backoff, and keep everything local.

**What I did:**

- Removed the periodic “discovery check‑in” call from the web core loop, which was responsible for:
  - Repeated HTTP POSTs to the legacy discovery server.
  - Exponential backoff logs and error spam when it could not reach the cloud endpoint.
- Changed the startup web behavior so the default route (`/`) redirects to the **local** `remote.html` on the device instead of the cloud app.
- Left the rest of the wireless and OTA code untouched so local control and possible future in‑network updates remain possible.

**Effect:**

- With this firmware, the device no longer attempts to check in with `app.emotiscope.rocks`.
- Your browser, when pointed at the device’s IP, is meant to land on `remote.html` served from the ESP32, not an external site.

---

### 1.4 Filesystem and LittleFS Initialization

**Goal:** Make sure the device can actually serve the static assets (HTML, JS, CSS) from flash.

**What I did:**

- Confirmed that the LittleFS mount routine is present and wired into the boot sequence:
  - A dedicated filesystem initializer mounts LittleFS, printing either a success or failure message.
  - The system initialization routine calls this filesystem initializer early in the boot process, before WiFi and before the web server.
- Verified that the HTTP routing logic:
  - Maps `/` to `/index.html`.
  - Maps `/remote` to `/remote.html`.
  - Serves files from LittleFS, returning 404 if a file is missing.
- Confirmed that the repository expects a filesystem image (`littlefs.bin`) built from the `v1.1_build/data` directory, with that image flashed to the filesystem partition defined in the partition table.

**Effect:**

- From a code perspective, the ESP32 is ready to:
  - Mount LittleFS.
  - Map requests to `/index.html` and `/remote.html`.
  - Serve those files if they exist in the filesystem image.
- If the filesystem partition is empty or outdated, you will get 404s regardless of how correct the routing code is.

---

### 1.5 HIL_EXTENDED Memory Footprint and Trimming

**Goal:** Provide deep HIL instrumentation without starving the HTTP server or crashing on `/remote.html`.

**What I did initially:**

- Implemented a comprehensive capture structure (`hil_capture_state_t`) that holds:
  - Frequency magnitudes (spectrogram and smoothed spectrogram).
  - Chromagram.
  - Loudness metrics (VU level, max, floor).
  - Novelty curves (full history, normalized).
  - Tempo detection arrays (magnitude, phase, beat).
  - Raw audio sample history (a large array of floats for I2S snapshots).
- On boot with `HIL_EXTENDED` enabled:
  - Allocated this structure dynamically on the heap.
  - Cleared it and wired it into the CPU core loop for periodic updates.
- In the CPU core loop:
  - After each DSP step, copied the relevant arrays and scalars into this HIL structure.
  - Measured capture overhead and periodically reported aggregated timings.

**What I changed to reduce memory pressure:**

- Identified the raw I2S sample buffer in the HIL struct as the single largest optional consumer of RAM.
- Removed:
  - The big raw sample array.
  - The associated counter and capture logic.
- Kept:
  - All higher‑level DSP captures (spectrogram, novelty, tempo, VU), which are much smaller but still extremely useful for HIL.
- Rebuilt the HIL_EXTENDED firmware and confirmed:
  - The program still fits with ample flash margin.
  - Dynamic memory usage sits at roughly two‑thirds of available RAM, leaving a six‑figure number of bytes for heap and stacks.

**Effect:**

- The firmware now uses significantly less heap for HIL while retaining most of the analytical power.
- This reduces—but does not entirely eliminate—the risk that the HTTP server fails memory allocations for `/remote.html`.

---

### 1.6 Serial “Bootleg” Banner Removal

**Goal:** Completely remove the ASCII art banner and “bootleg” lecture from serial output.

**What I did:**

- Located the hidden function that prints the banner and text, which:
  - Waited for a time threshold.
  - Then printed the ASCII art, hardware/software version and the full “bootleg” message once.
- Replaced that function’s body with a no‑operation implementation:
  - It is still present and callable, but does nothing and prints nothing.
- Recompiled the HIL_EXTENDED firmware successfully with this change.

**Effect:**

- In the current compiled binary:
  - That banner is no longer emitted at any time.
- The fact that you are still seeing the banner on your serial output indicates:
  - The device is still running an **older** firmware image that predates this change, not the freshly built binary with the no‑op banner function.

---

## 2. Detailed Assessment: Why the Web App Does Not Load

There are **two distinct issues** that have caused the web app to fail during this work:

1. **Static file not found:** The device responds with 404 for `/index.html` or `/remote.html`.
2. **HTTP memory allocation failure:** The device logs “Unable to allocate memory.” when handling `/remote.html`.

They share symptoms at the browser (no page), but they are different problems under the hood.

---

### 2.1 Case 1: 404 for `/index.html` and `/remote.html`

**Observed behaviour:**

- Device logs show HTTP GETs for `/index.html` and `/remote.html`.
- Browser reports that no web page is found at those URLs.
- The firmware’s HTTP handler returns 404 when:
  - It computes a filesystem path (e.g., `/index.html` or `/remote.html`).
  - It attempts to open that file from LittleFS and fails (file doesn’t exist on the mounted filesystem).

**Code‑level situation (conceptually, without snippets):**

- The routing logic is correct and simple:
  - `/` is mapped to `/index.html`.
  - `/remote` is mapped to `/remote.html`.
- The file serving layer:
  - Asks LittleFS for that path.
  - If a valid file handle is returned, streams it.
  - If not, returns a 404.

**Root cause in this mode:**

- The **LittleFS partition on the device does not contain the web files** corresponding to `v1.1_build/data/index.html` and `v1.1_build/data/remote.html`.
- In other words:
  - The static assets from the repo were never packed into a filesystem image and flashed to the `spiffs/littlefs` partition defined in the partition table, or
  - An old/empty filesystem image is still present.

**Impact:**

- No matter how correct the networking and web‑server code is:
  - If the filesystem partition on the S3 is empty or stale, the server will continue to return 404 for those paths.
  - The browser will never see the app, because the device simply does not have the files to serve.

---

### 2.2 Case 2: “Unable to allocate memory.” on `/remote.html`

**Observed behaviour:**

- Device logs:
  - HTTP GET for `/remote.html` (possibly repeated).
  - An error message stating that it is unable to allocate memory.
- This message is produced deep inside the ESP32’s HTTP/WS stack (ESP‑IDF / PsychicHttp), not by your own application code.

**Implications:**

- By the time you see “Unable to allocate memory”:
  - The router has already resolved the path.
  - The filesystem has likely **succeeded** in opening `/remote.html`.
  - The failure occurs when the HTTP library tries to allocate internal buffers needed to send the file over the network.

**Interaction with HIL_EXTENDED and memory usage:**

- The v1.1 firmware is already fairly memory‑heavy:
  - Large global arrays for audio processing, debug buffers, and configuration.
  - WiFi, DNS, MDNS, HTTP, WebSocket handlers all active.
- HIL_EXTENDED increases memory pressure further by:
  - Adding multiple large float arrays for DSP capture (spectrogram, novelty curves, tempo bins).
  - Initially, also allocating a very large raw audio sample history capture buffer.
- With all of this enabled:
  - The heap may be sufficiently tight or fragmented that when the HTTP server attempts to reserve space for sending `/remote.html`, the allocation fails.
  - When it fails, it emits the generic “Unable to allocate memory.” error and aborts the response.

**What I did to mitigate:**

- Removed the largest optional HIL buffer (raw I2S sample capture) to reclaim a significant chunk of RAM.
- Left all the more compact HIL buffers intact to retain useful monitoring.
- Rebuilt the firmware and checked static memory usage:
  - Dynamic memory usage (globals, static buffers) stays at about two‑thirds of available RAM.
  - This leaves a substantial amount of memory available for heap and stacks compared to the earlier setup.

**Root cause in this mode:**

- When you saw “Unable to allocate memory.” previously, the firmware running on the device was:
  - A HIL_EXTENDED build with the **full** capture buffers, especially the raw I2S history.
  - Possibly combined with other allocations at runtime, leaving too little contiguous heap for the HTTP layer.
- Under those conditions, serving `/remote.html` becomes unreliable because the web server simply cannot acquire enough memory for its operations.

---

### 2.3 Why You Still See the Old Banner and Behaviour

**Key observation:**

- The serial output you recently pasted includes the full ASCII art banner and bootleg note, which:
  - Has been removed in the current version of `leds.h` by turning the banner function into a no‑op.
  - Should not appear at all in any boot sequence using the newly compiled firmware.

**Conclusion:**

- The ESP32‑S3 is still running an older firmware binary that:
  - Contains the banner.
  - Still uses the larger HIL capture structure.
  - May be running with an outdated or missing filesystem image.
- The changes I’ve made and verified in the repo are **not yet applied to the hardware**:
  - The new HIL‑trimmed, banner‑free binary has been compiled successfully.
  - It needs to be uploaded to the correct serial port.
  - A filesystem image built from `v1.1_build/data` needs to be flashed to the filesystem partition.

---

## 3. Short Summary

- **What’s in the code now (on disk):**
  - WiFi defaulting to VX220‑013F with your SSID and password.
  - HIL controls integrated into the local web UI (remote.html) and routed over WebSockets.
  - Cloud integration and discovery POSTs disabled; root route redirected to local web app.
  - LittleFS correctly mounted during boot and used to serve `/` and `/remote`.
  - HIL_EXTENDED capture implemented, with the largest optional buffer removed to free heap.
  - The serial ASCII banner and “bootleg” note fully disabled.

- **Why the web app still doesn’t load on the device:**
  1. The **filesystem** on the S3 is not carrying the web assets from `v1.1_build/data` (leading to 404 when static content is requested).
  2. The **firmware actually running on the hardware** is an outdated image with heavier HIL buffers and the banner still compiled in (leading to “Unable to allocate memory.” when it tries to serve `/remote.html` under high memory pressure).

Until the current compiled firmware and a correct LittleFS image are both flashed to the device, the behavior of the running system will not match the code and fixes described above.