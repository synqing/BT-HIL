#ifndef HIL_EXPORT_H
#define HIL_EXPORT_H

#include <Arduino.h>
#include <LittleFS.h>
#include "global_defines.h"
#include "hil_capture.h"

// Export modes
enum hil_export_mode_t {
    HIL_EXPORT_NONE = 0,
    HIL_EXPORT_SERIAL,      // Stream CSV rows to Serial at 2Mbaud
    HIL_EXPORT_LITTLEFS,    // Write CSV to LittleFS (limited size)
    HIL_EXPORT_WEBSOCKET    // Stream to WebSocket client (future)
};

// Export state structure (Blueprint SL-2, SL-6, SL-9, SL-10)
struct hil_export_state_t {
    hil_export_mode_t mode;             // Current export mode
    File csv_file;                      // SL-2: Active CSV file handle (LittleFS)
    char csv_filename[64];              // Timestamped filename
    uint32_t frame_counter;             // SL-6: Frames logged since LOG_START
    uint32_t max_frames;                // Maximum frames to log (LittleFS limit)
    bool export_active;                 // Is export currently running
    bool fs_error;                      // SL-9: Filesystem error flag
    uint8_t origin_client_slot;         // Client that started the export

    // Column selection flags (to reduce data volume)
    bool export_spectrogram;
    bool export_chromagram;
    bool export_vu;
    bool export_novelty;
    bool export_tempi;
    bool export_samples;                // Raw I2S samples (large!)
};

// Global export state
extern hil_export_state_t* hil_export_state;

// Global flag to suppress system printf during CSV serial export
// When true, system logging (WebSocket events, etc.) should be silenced
extern volatile bool hil_csv_serial_export_active;

// Define the CSV export flag (suppresses system printf during serial export)
volatile bool hil_csv_serial_export_active = false;

// Initialize export state (called from setup)
inline void hil_export_init() {
    static hil_export_state_t state = {
        .mode = HIL_EXPORT_NONE,
        .csv_file = File(),
        .csv_filename = {0},
        .frame_counter = 0,
        .max_frames = 500,              // ~2.5 seconds at 200 FPS
        .export_active = false,
        .fs_error = false,
        .origin_client_slot = 255,
        .export_spectrogram = true,
        .export_chromagram = true,
        .export_vu = true,
        .export_novelty = false,        // Large - disabled by default
        .export_tempi = true,
        .export_samples = false         // Very large - disabled by default
    };
    hil_export_state = &state;
}

// Generate timestamped filename
inline void hil_export_create_filename(char* buffer, size_t len) {
    uint32_t ms = millis();
    snprintf(buffer, len, "/hil_%010lu.csv", ms);
}

// Write CSV header row (Print interface for LittleFS)
inline void hil_export_write_header(Print& out) {
    out.print("timestamp_ms,vu_level,vu_max,vu_floor");

    if (hil_export_state->export_spectrogram) {
        for (uint8_t i = 0; i < NUM_FREQS; i++) {
            out.printf(",spec_%d", i);
        }
    }

    if (hil_export_state->export_chromagram) {
        for (uint8_t i = 0; i < 12; i++) {
            out.printf(",chroma_%d", i);
        }
    }

    if (hil_export_state->export_tempi) {
        for (uint8_t i = 0; i < NUM_TEMPI; i++) {
            out.printf(",tempi_mag_%d", i);
        }
        for (uint8_t i = 0; i < NUM_TEMPI; i++) {
            out.printf(",tempi_phase_%d", i);
        }
        for (uint8_t i = 0; i < NUM_TEMPI; i++) {
            out.printf(",tempi_beat_%d", i);
        }
    }

    if (hil_export_state->export_novelty) {
        for (uint16_t i = 0; i < NOVELTY_HISTORY_LENGTH; i++) {
            out.printf(",novelty_%d", i);
        }
    }

    out.println();
}

// Write CSV header row (printf for serial - ESP32-S3 USB JTAG)
inline void hil_export_write_header_printf() {
    printf("timestamp_ms,vu_level,vu_max,vu_floor");

    if (hil_export_state->export_spectrogram) {
        for (uint8_t i = 0; i < NUM_FREQS; i++) {
            printf(",spec_%d", i);
        }
    }

    if (hil_export_state->export_chromagram) {
        for (uint8_t i = 0; i < 12; i++) {
            printf(",chroma_%d", i);
        }
    }

    if (hil_export_state->export_tempi) {
        for (uint8_t i = 0; i < NUM_TEMPI; i++) {
            printf(",tempi_mag_%d", i);
        }
        for (uint8_t i = 0; i < NUM_TEMPI; i++) {
            printf(",tempi_phase_%d", i);
        }
        for (uint8_t i = 0; i < NUM_TEMPI; i++) {
            printf(",tempi_beat_%d", i);
        }
    }

    if (hil_export_state->export_novelty) {
        for (uint16_t i = 0; i < NOVELTY_HISTORY_LENGTH; i++) {
            printf(",novelty_%d", i);
        }
    }

    printf("\n");
}

// Write CSV data row from capture state (Print interface for LittleFS)
inline void hil_export_write_row(Print& out, hil_capture_state_t* cap) {
    if (!cap) return;

    // Timestamp and VU (always included)
    out.printf("%lu,%.4f,%.4f,%.4f",
        millis(),
        cap->vu_level_capture,
        cap->vu_max_capture,
        cap->vu_floor_capture
    );

    if (hil_export_state->export_spectrogram) {
        for (uint8_t i = 0; i < NUM_FREQS; i++) {
            out.printf(",%.4f", cap->spectrogram_capture[i]);
        }
    }

    if (hil_export_state->export_chromagram) {
        for (uint8_t i = 0; i < 12; i++) {
            out.printf(",%.4f", cap->chromagram_capture[i]);
        }
    }

    if (hil_export_state->export_tempi) {
        for (uint8_t i = 0; i < NUM_TEMPI; i++) {
            out.printf(",%.4f", cap->tempi_magnitude_capture[i]);
        }
        for (uint8_t i = 0; i < NUM_TEMPI; i++) {
            out.printf(",%.4f", cap->tempi_phase_capture[i]);
        }
        for (uint8_t i = 0; i < NUM_TEMPI; i++) {
            out.printf(",%.4f", cap->tempi_beat_capture[i]);
        }
    }

    if (hil_export_state->export_novelty) {
        for (uint16_t i = 0; i < NOVELTY_HISTORY_LENGTH; i++) {
            out.printf(",%.4f", cap->novelty_curve_normalized_capture[i]);
        }
    }

    out.println();
}

// Write CSV data row (printf for serial - ESP32-S3 USB JTAG)
inline void hil_export_write_row_printf(hil_capture_state_t* cap) {
    if (!cap) return;

    // Timestamp and VU (always included)
    printf("%lu,%.4f,%.4f,%.4f",
        millis(),
        cap->vu_level_capture,
        cap->vu_max_capture,
        cap->vu_floor_capture
    );

    if (hil_export_state->export_spectrogram) {
        for (uint8_t i = 0; i < NUM_FREQS; i++) {
            printf(",%.4f", cap->spectrogram_capture[i]);
        }
    }

    if (hil_export_state->export_chromagram) {
        for (uint8_t i = 0; i < 12; i++) {
            printf(",%.4f", cap->chromagram_capture[i]);
        }
    }

    if (hil_export_state->export_tempi) {
        for (uint8_t i = 0; i < NUM_TEMPI; i++) {
            printf(",%.4f", cap->tempi_magnitude_capture[i]);
        }
        for (uint8_t i = 0; i < NUM_TEMPI; i++) {
            printf(",%.4f", cap->tempi_phase_capture[i]);
        }
        for (uint8_t i = 0; i < NUM_TEMPI; i++) {
            printf(",%.4f", cap->tempi_beat_capture[i]);
        }
    }

    if (hil_export_state->export_novelty) {
        for (uint16_t i = 0; i < NOVELTY_HISTORY_LENGTH; i++) {
            printf(",%.4f", cap->novelty_curve_normalized_capture[i]);
        }
    }

    printf("\n");
}

// Start CSV export to LittleFS
inline bool hil_export_start_littlefs(uint32_t max_frames = 500) {
    if (!hil_export_state) return false;
    if (hil_export_state->export_active) return false;

    hil_export_create_filename(hil_export_state->csv_filename, sizeof(hil_export_state->csv_filename));

    hil_export_state->csv_file = LittleFS.open(hil_export_state->csv_filename, FILE_WRITE);
    if (!hil_export_state->csv_file) {
        hil_export_state->fs_error = true;
        return false;
    }

    hil_export_state->mode = HIL_EXPORT_LITTLEFS;
    hil_export_state->frame_counter = 0;
    hil_export_state->max_frames = max_frames;
    hil_export_state->export_active = true;
    hil_export_state->fs_error = false;

    hil_export_write_header(hil_export_state->csv_file);
    return true;
}

// Start CSV export to Serial (uses printf for ESP32-S3 USB JTAG)
inline bool hil_export_start_serial() {
    if (!hil_export_state) return false;
    if (hil_export_state->export_active) return false;

    hil_export_state->mode = HIL_EXPORT_SERIAL;
    hil_export_state->frame_counter = 0;
    hil_export_state->max_frames = 0;  // No limit for serial
    hil_export_state->export_active = true;
    hil_export_state->fs_error = false;

    // Suppress system logging during CSV export
    hil_csv_serial_export_active = true;

    printf("--- HIL CSV START ---\n");
    hil_export_write_header_printf();
    return true;
}

// Stop export and finalize
inline void hil_export_stop() {
    if (!hil_export_state) return;
    if (!hil_export_state->export_active) return;

    if (hil_export_state->mode == HIL_EXPORT_LITTLEFS) {
        hil_export_state->csv_file.flush();
        hil_export_state->csv_file.close();
    }
    else if (hil_export_state->mode == HIL_EXPORT_SERIAL) {
        printf("--- HIL CSV END ---\n");
        fflush(stdout);
        // Re-enable system logging
        hil_csv_serial_export_active = false;
    }

    hil_export_state->mode = HIL_EXPORT_NONE;
    hil_export_state->export_active = false;
}

// Process one frame of export (called from cpu_core.h)
inline void hil_export_process_frame(hil_capture_state_t* cap) {
    if (!hil_export_state || !cap) return;
    if (!hil_export_state->export_active) return;

    switch (hil_export_state->mode) {
        case HIL_EXPORT_LITTLEFS:
            hil_export_write_row(hil_export_state->csv_file, cap);
            hil_export_state->frame_counter++;

            // Flush periodically
            if (hil_export_state->frame_counter % 50 == 0) {
                hil_export_state->csv_file.flush();
            }

            // Auto-stop at max frames
            if (hil_export_state->max_frames > 0 &&
                hil_export_state->frame_counter >= hil_export_state->max_frames) {
                hil_export_stop();
            }
            break;

        case HIL_EXPORT_SERIAL:
            hil_export_write_row_printf(cap);
            hil_export_state->frame_counter++;

            // Flush periodically
            if (hil_export_state->frame_counter % 100 == 0) {
                fflush(stdout);
            }
            break;

        default:
            break;
    }
}

// ============================================================================
// Phase 3: Binary Snapshot Export (flash-optimized)
// ============================================================================

// Binary file header (PRD §5.11.3)
struct hil_binary_header_t {
    uint32_t magic;           // 0x48494C01 = "HIL" version 1
    uint16_t array_length;
    uint8_t element_size;
    uint8_t element_type;     // 1 = float32
    uint32_t timestamp_ms;
    uint32_t reserved;
};

// Export binary snapshot (smaller than JSON, uses less flash)
inline bool hil_export_json_snapshot(hil_capture_state_t* cap) {
    if (!cap) return false;

    char fn[32];
    snprintf(fn, sizeof(fn), "/hil_%lu.bin", millis());

    File f = LittleFS.open(fn, FILE_WRITE);
    if (!f) return false;

    // Write binary header + VU + top BPM
    hil_binary_header_t hdr = {0x48494C01, 3, 4, 1, millis(), 0};
    f.write((uint8_t*)&hdr, sizeof(hdr));

    float vu[3] = {cap->vu_level_capture, cap->vu_max_capture, cap->vu_floor_capture};
    f.write((uint8_t*)vu, sizeof(vu));

    // Find top BPM
    float top_m = 0; uint16_t top_i = 0;
    for (uint16_t i = 0; i < NUM_TEMPI; i++) {
        if (cap->tempi_magnitude_capture[i] > top_m) {
            top_m = cap->tempi_magnitude_capture[i];
            top_i = i;
        }
    }
    uint16_t bpm = TEMPO_LOW + top_i;
    f.write((uint8_t*)&bpm, 2);
    f.write((uint8_t*)&top_m, 4);

    f.close();
    return true;
}

// ============================================================================
// Phase 4: WebSocket Binary Frames
// ============================================================================

// Binary message types for WebSocket streaming (PRD §5.12.2)
enum hil_ws_binary_type_t {
    HIL_BIN_SPECTROGRAM = 0x01,
    HIL_BIN_NOVELTY = 0x02,
    HIL_BIN_TEMPI_MAG = 0x03,
    HIL_BIN_TEMPI_PHASE = 0x04,
    HIL_BIN_TEMPI_BEAT = 0x05,
    HIL_BIN_CHROMAGRAM = 0x06,
    HIL_BIN_VU = 0x07,
    HIL_BIN_SAMPLE_HISTORY = 0x08
};

// WebSocket subscription state (per-client)
struct hil_ws_subscription_t {
    bool spectrogram;
    bool novelty;
    bool tempi;
    bool chromagram;
    bool vu;
    bool sample_history;
    uint8_t decimation;  // Only send every Nth frame (1 = every frame)
    uint8_t frame_counter;
};

// Global subscription state (simplified: single subscription for all clients)
static hil_ws_subscription_t hil_ws_subscription = {
    .spectrogram = false,
    .novelty = false,
    .tempi = false,
    .chromagram = false,
    .vu = false,
    .sample_history = false,
    .decimation = 40,  // Default: send every 40th frame (~5 Hz) - conservative to avoid httpd overload
    .frame_counter = 0
};

// Rate limiting for binary streaming
static uint32_t hil_ws_last_send_ms = 0;
static const uint32_t HIL_WS_MIN_INTERVAL_MS = 100;  // Max 10 Hz to avoid httpd errors

// Parse subscription command: "subscribe|spectrogram|tempi|vu"
inline void hil_ws_parse_subscription(const char* cmd) {
    // Reset all
    hil_ws_subscription.spectrogram = false;
    hil_ws_subscription.novelty = false;
    hil_ws_subscription.tempi = false;
    hil_ws_subscription.chromagram = false;
    hil_ws_subscription.vu = false;
    hil_ws_subscription.sample_history = false;

    if (strstr(cmd, "spectrogram")) hil_ws_subscription.spectrogram = true;
    if (strstr(cmd, "novelty")) hil_ws_subscription.novelty = true;
    if (strstr(cmd, "tempi")) hil_ws_subscription.tempi = true;
    if (strstr(cmd, "chromagram")) hil_ws_subscription.chromagram = true;
    if (strstr(cmd, "vu")) hil_ws_subscription.vu = true;
    if (strstr(cmd, "sample_history")) hil_ws_subscription.sample_history = true;
    if (strstr(cmd, "all")) {
        hil_ws_subscription.spectrogram = true;
        hil_ws_subscription.tempi = true;
        hil_ws_subscription.chromagram = true;
        hil_ws_subscription.vu = true;
    }
}

// Check if any WebSocket streaming is active
inline bool hil_ws_has_subscriptions() {
    return hil_ws_subscription.spectrogram || hil_ws_subscription.novelty ||
           hil_ws_subscription.tempi || hil_ws_subscription.chromagram ||
           hil_ws_subscription.vu || hil_ws_subscription.sample_history;
}

// Clear all subscriptions
inline void hil_ws_unsubscribe_all() {
    hil_ws_subscription.spectrogram = false;
    hil_ws_subscription.novelty = false;
    hil_ws_subscription.tempi = false;
    hil_ws_subscription.chromagram = false;
    hil_ws_subscription.vu = false;
    hil_ws_subscription.sample_history = false;
}

// Send a single binary frame: [1-byte type][4-byte timestamp][payload]
// Returns true if sent successfully
inline bool hil_ws_send_binary_frame(hil_ws_binary_type_t type, const void* data, uint16_t data_len) {
    extern PsychicWebSocketHandler websocket_handler;

    // Build frame: [type:1][timestamp_ms:4][data:N]
    static uint8_t frame_buffer[512];  // Max frame size
    if (data_len + 5 > sizeof(frame_buffer)) return false;

    frame_buffer[0] = (uint8_t)type;
    uint32_t ts = millis();
    memcpy(&frame_buffer[1], &ts, 4);
    memcpy(&frame_buffer[5], data, data_len);

    websocket_handler.sendAll(HTTPD_WS_TYPE_BINARY, frame_buffer, data_len + 5);
    return true;
}

// Process WebSocket binary streaming (called from gpu_core.h after render)
inline void hil_ws_process_subscriptions(hil_capture_state_t* cap) {
    if (!cap || !hil_ws_has_subscriptions()) return;

    // Time-based rate limiting (primary throttle)
    uint32_t now_ms = millis();
    if ((now_ms - hil_ws_last_send_ms) < HIL_WS_MIN_INTERVAL_MS) return;
    hil_ws_last_send_ms = now_ms;

    // Send ONE type per call to spread load - rotate through subscribed types
    static uint8_t send_slot = 0;
    send_slot = (send_slot + 1) % 5;

    switch (send_slot) {
        case 0:
            if (hil_ws_subscription.spectrogram) {
                hil_ws_send_binary_frame(HIL_BIN_SPECTROGRAM, cap->spectrogram_smooth_capture, sizeof(float) * NUM_FREQS);
            }
            break;
        case 1:
            if (hil_ws_subscription.chromagram) {
                hil_ws_send_binary_frame(HIL_BIN_CHROMAGRAM, cap->chromagram_capture, sizeof(float) * 12);
            }
            break;
        case 2:
            if (hil_ws_subscription.vu) {
                float vu_data[3] = {cap->vu_level_capture, cap->vu_max_capture, cap->vu_floor_capture};
                hil_ws_send_binary_frame(HIL_BIN_VU, vu_data, sizeof(vu_data));
            }
            break;
        case 3:
            if (hil_ws_subscription.tempi) {
                hil_ws_send_binary_frame(HIL_BIN_TEMPI_MAG, cap->tempi_magnitude_capture, sizeof(float) * NUM_TEMPI);
            }
            break;
        case 4:
            // Reserved for novelty - disabled by default due to size
            break;
    }
}

// ============================================================================
// Status Functions
// ============================================================================

// Get export status string
inline void hil_export_get_status(char* buffer, size_t len) {
    if (!hil_export_state) {
        snprintf(buffer, len, "hil_export_status|unavailable");
        return;
    }

    const char* mode_str = "none";
    switch (hil_export_state->mode) {
        case HIL_EXPORT_SERIAL: mode_str = "serial"; break;
        case HIL_EXPORT_LITTLEFS: mode_str = "littlefs"; break;
        case HIL_EXPORT_WEBSOCKET: mode_str = "websocket"; break;
        default: mode_str = "none"; break;
    }

    if (hil_export_state->export_active) {
        snprintf(buffer, len, "hil_export_status|active|%s|%lu|%lu|%s",
                 mode_str,
                 hil_export_state->frame_counter,
                 hil_export_state->max_frames,
                 hil_export_state->csv_filename);
    } else {
        snprintf(buffer, len, "hil_export_status|inactive|%s|%lu",
                 mode_str,
                 hil_export_state->frame_counter);
    }
}

#endif // HIL_EXPORT_H
