/*
--------------------------------------------------------
                     _                            _
                    | |                          | |
 ___   _   _   ___  | |_    ___   _ __ ___       | |__
/ __| | | | | / __| | __|  / _ \ | '_ ` _ \      | '_ \ 
\__ \ | |_| | \__ \ | |_  |  __/ | | | | | |  _  | | | |
|___/  \__, | |___/  \__|  \___| |_| |_| |_| (_) |_| |_|
        __/ |
       |___/

Foundational functions like UART initialization
*/

#include <Arduino.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef HIL_EXTENDED
#include "hil_capture.h"
#endif

#if __has_include(<driver/uart.h>)
#include <driver/uart.h>
#define EMOTISCOPE_HAS_UART_DRIVER 1
#else
#define EMOTISCOPE_HAS_UART_DRIVER 0
#endif

#if __has_include(<driver/usb_serial_jtag.h>)
#include <driver/usb_serial_jtag.h>
#define EMOTISCOPE_HAS_USB_SERIAL_JTAG 1
#else
#define EMOTISCOPE_HAS_USB_SERIAL_JTAG 0
#endif

volatile bool EMOTISCOPE_ACTIVE = true;

char serial_buffer[256];
uint16_t serial_buffer_index = 0;

uint32_t last_command_time = 0;

extern bool system_info_serial_enabled;

#ifdef HIL_EXTENDED
extern hil_capture_state_t* hil_capture_state;
static bool hil_serial_debug_enabled = false;

static void hil_print_snapshot() {
    hil_capture_cpu_snapshot_t snapshot;
    if (!hil_capture_read_cpu_snapshot(&snapshot)) {
        printf("HIL SNAPSHOT ----- UNAVAILABLE\n");
        return;
    }

    printf("HIL SNAPSHOT ----- BEGIN\n");

    printf("HIL VU LEVEL ----- %.3f (max=%.3f floor=%.3f)\n",
           snapshot.vu_level,
           snapshot.vu_max,
           snapshot.vu_floor);

    printf("HIL NOVELTY[0..15]:");
    for (uint16_t i = 0; i < 16; i++) {
        printf(" %.3f", snapshot.novelty_norm_0_15[i]);
    }
    printf("\n");

    printf("HIL SPECTROGRAM[0..15]:");
    for (uint16_t i = 0; i < 16; i++) {
        printf(" %.3f", snapshot.spectrogram_0_15[i]);
    }
    printf("\n");

    float top_mag[3] = {0.0f, 0.0f, 0.0f};
    uint16_t top_idx[3] = {0, 0, 0};
    for (uint16_t i = 0; i < NUM_TEMPI; i++) {
        float m = snapshot.tempi_magnitude[i];
        if (m > top_mag[0]) {
            top_mag[2] = top_mag[1];
            top_idx[2] = top_idx[1];
            top_mag[1] = top_mag[0];
            top_idx[1] = top_idx[0];
            top_mag[0] = m;
            top_idx[0] = i;
        } else if (m > top_mag[1]) {
            top_mag[2] = top_mag[1];
            top_idx[2] = top_idx[1];
            top_mag[1] = m;
            top_idx[1] = i;
        } else if (m > top_mag[2]) {
            top_mag[2] = m;
            top_idx[2] = i;
        }
    }

    printf("HIL TEMPI TOP 3 (BPM|mag):");
    for (uint8_t k = 0; k < 3; k++) {
        if (top_mag[k] > 0.0f) {
            uint16_t bpm = TEMPO_LOW + top_idx[k];
            printf(" %u|%.3f", bpm, top_mag[k]);
        }
    }
    printf("\n");

    printf("HIL SNAPSHOT ----- END\n");
}
#endif

static bool handle_keyboard_shortcut(char c) {
	extern void save_config_delayed();

	auto clamp01 = [](float v) -> float {
		if (v < 0.0f) return 0.0f;
		if (v > 1.0f) return 1.0f;
		return v;
	};

	if (c == 'I') {
		system_info_serial_enabled = !system_info_serial_enabled;
		printf("SYSTEM INFO PRINT - %s\n", system_info_serial_enabled ? "ON" : "OFF");
		Serial.printf("SYSTEM INFO PRINT - %s\n", system_info_serial_enabled ? "ON" : "OFF");
		return true;
	}

	if (c == 'i') {
		configuration.reverse_color_range = !configuration.reverse_color_range;
		save_config_delayed();
		printf("REVERSE RANGE ---- %s\n", configuration.reverse_color_range ? "ON" : "OFF");
		return true;
	}

	if (c == 'l') {
		configuration.auto_color_cycle = !configuration.auto_color_cycle;
		save_config_delayed();
		printf("AUTO COLOR ------- %s\n", configuration.auto_color_cycle ? "ON" : "OFF");
		return true;
	}

	if (c == '[' || c == ']') {
		const float hue_step = 0.01f;
		if (c == '[') {
			configuration.color -= hue_step;
			if (configuration.color < 0.0f) {
				configuration.color += 1.0f;
			}
		} else {
			configuration.color += hue_step;
			if (configuration.color > 1.0f) {
				configuration.color -= 1.0f;
			}
		}
		save_config_delayed();
		printf("HUE ------------- %.3f\n", configuration.color);
		return true;
	}

	if (c == ',' || c == '.') {
		extern int16_t increment_mode();
		extern int16_t decrement_mode();
		extern const char* get_light_mode_name(int16_t mode_index);
		extern void load_sliders_relevant_to_mode(int16_t mode_index);
		extern void load_toggles_relevant_to_mode(int16_t mode_index);
		extern void broadcast(const char* message);
		extern void toggle_standby();

		if (EMOTISCOPE_ACTIVE == true) {
			int16_t mode_index = (c == ',') ? decrement_mode() : increment_mode();
			load_sliders_relevant_to_mode(mode_index);
			load_toggles_relevant_to_mode(mode_index);
			broadcast("reload_config");
			printf("MODE %s -------- %s (%d)\n", (c == ',') ? "PREV" : "NEXT", get_light_mode_name(mode_index), (int)mode_index);
		} else {
			toggle_standby();
			printf("STANDBY ---------- TOGGLED\n");
		}
		return true;
	}

	if (c == '=' || c == '-') {
		const float step = 0.02f;
		configuration.brightness = clamp01(configuration.brightness + ((c == '=') ? step : -step));
		save_config_delayed();
		printf("BRIGHTNESS ------- %.3f\n", configuration.brightness);
		return true;
	}

	if (c == '+' || c == '_') {
		const float step = 0.02f;
		configuration.softness = clamp01(configuration.softness + ((c == '+') ? step : -step));
		save_config_delayed();
		printf("SOFTNESS --------- %.3f\n", configuration.softness);
		return true;
	}

	if (c == '{' || c == '}') {
		const float step = 0.02f;
		configuration.speed = clamp01(configuration.speed + ((c == '}') ? step : -step));
		save_config_delayed();
		printf("SPEED ------------ %.3f\n", configuration.speed);
		return true;
	}

	if (c == '<' || c == '>') {
		const float step = 0.02f;
		configuration.color_range = clamp01(configuration.color_range + ((c == '>') ? step : -step));
		save_config_delayed();
		printf("COLOR RANGE ------ %.3f\n", configuration.color_range);
		return true;
	}

	if (c == 'p') {
		const float step = 0.02f;
		configuration.warmth += step;
		if (configuration.warmth > 1.0f) {
			configuration.warmth -= 1.0f;
		}
		save_config_delayed();
		printf("WARMTH ----------- %.3f\n", configuration.warmth);
		return true;
	}

	if (c == 'o') {
		const float step = 0.02f;
		configuration.saturation += step;
		if (configuration.saturation > 1.0f) {
			configuration.saturation -= 1.0f;
		}
		save_config_delayed();
		printf("SATURATION ------- %.3f\n", configuration.saturation);
		return true;
	}

	if (c == 'r') {
		configuration.color = 0.33f;
		configuration.color_range = 0.50f;
		configuration.warmth = 0.00f;
		configuration.saturation = 0.75f;
		configuration.reverse_color_range = false;
		save_config_delayed();
		printf("COLOR RESET ------\n");
		printf("HUE ------------- %.3f\n", configuration.color);
		printf("COLOR RANGE ------ %.3f\n", configuration.color_range);
		printf("WARMTH ----------- %.3f\n", configuration.warmth);
		printf("SATURATION ------- %.3f\n", configuration.saturation);
		printf("REVERSE RANGE ---- %s\n", configuration.reverse_color_range ? "ON" : "OFF");
		return true;
	}

	if (c == 'm' || c == 'n') {
		const float step = 0.02f;
		configuration.background = clamp01(configuration.background + ((c == 'm') ? step : -step));
		save_config_delayed();
		printf("BACKGROUND ------- %.3f\n", configuration.background);
		return true;
	}

	#ifdef HIL_EXTENDED
	if (c == 'd' || c == 'D') {
		hil_serial_debug_enabled = !hil_serial_debug_enabled;
		printf("HIL SERIAL DEBUG -- %s\n", hil_serial_debug_enabled ? "ON" : "OFF");
		Serial.printf("HIL SERIAL DEBUG -- %s\n", hil_serial_debug_enabled ? "ON" : "OFF");
		return true;
	}
	if (c == 'H') {
		hil_print_snapshot();
		Serial.printf("HIL SNAPSHOT ----- PRINTED\n");
		return true;
	}
	#endif

	if (c == 'h' || c == '?') {
		printf("SHORTCUTS --------\n");
		printf("  ,. -------------- mode prev/next\n");
		printf("  [/] ------------- hue -/+\n");
		printf("  =/- ------------- brightness +/-\n");
		printf("  +/_ ------------- softness +/-(shift)\n");
		printf("  {} -------------- speed -/+ (shift)\n");
		printf("  </> ------------- color_range -/+ (shift)\n");
		printf("  p --------------- warmth wrap +\n");
		printf("  o --------------- saturation wrap +\n");
		printf("  r --------------- reset color tuning\n");
		printf("  n/m ------------- background -/+\n");
		printf("  i --------------- reverse_color_range toggle\n");
		printf("  l --------------- auto_color_cycle toggle\n");
		printf("  I --------------- system info print toggle\n");
		#ifdef HIL_EXTENDED
		printf("  H --------------- HIL snapshot\n");
		printf("  d --------------- HIL serial rx debug\n");
		#endif
		return true;
	}

	return false;
}

static int read_console_byte() {
	uint8_t b = 0;

#if EMOTISCOPE_HAS_USB_SERIAL_JTAG
	int usb_n = usb_serial_jtag_read_bytes(&b, 1, 0);
	if (usb_n == 1) {
		return (int)b;
	}
#endif

#if EMOTISCOPE_HAS_UART_DRIVER
	int uart_n = uart_read_bytes(UART_NUM_0, &b, 1, 0);
	if (uart_n == 1) {
		return (int)b;
	}
#endif

	if (Serial.available() > 0) {
		return (int)(uint8_t)Serial.read();
	}

	return -1;
}

void init_serial(uint32_t baud_rate) {
	// Artificial 10-second boot up wait time if needed
	//for(uint16_t i = 0; i < 10; i++){ printf("WAITING FOR %d SECONDS...\n", 10-i); delay(1000); }

	memset(serial_buffer, 0, 256);
	serial_buffer_index = 0;

#if EMOTISCOPE_HAS_USB_SERIAL_JTAG
	usb_serial_jtag_driver_config_t usb_jtag_cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
	usb_serial_jtag_driver_install(&usb_jtag_cfg);
#endif

	Serial.begin(baud_rate);
}

void check_serial() {
	extern bool queue_command(const char* command, uint16_t length, uint8_t client_slot);

	for (;;) {
		int rx = read_console_byte();
		if (rx < 0) {
			break;
		}
		char c = (char)rx;

        #ifdef HIL_EXTENDED
        if (hil_serial_debug_enabled) {
			if (c >= 32 && c <= 126) {
				printf("SER RX ----------- 0x%02X '%c'\n", (uint8_t)c, c);
			} else {
				printf("SER RX ----------- 0x%02X\n", (uint8_t)c);
			}
        }
        #endif
		if (handle_keyboard_shortcut(c)) {
			continue;
		}

		if(c == '\n'){
			if (serial_buffer_index > 0) {
				if (serial_buffer_index >= sizeof(serial_buffer)) {
					serial_buffer_index = sizeof(serial_buffer) - 1;
				}
				serial_buffer[serial_buffer_index] = '\0';

				bool queued = queue_command(serial_buffer, serial_buffer_index, 255);
				#ifdef HIL_EXTENDED
				if (hil_serial_debug_enabled) {
					printf("SER CMD QUEUE ---- %s | len=%u\n", queued ? "OK" : "FAIL", (uint16_t)serial_buffer_index);
					printf("SER CMD ---------- %s\n", serial_buffer);
				}
				#endif
			}

			memset(serial_buffer, 0, 256);
			serial_buffer_index = 0;
		}
		else if(c == '\r'){
			#ifdef HIL_EXTENDED
			if (hil_serial_debug_enabled) {
				printf("SER RX ----------- CR\n");
			}
			#endif
		}
		else{
			if (serial_buffer_index < (sizeof(serial_buffer) - 1)) {
				serial_buffer[serial_buffer_index] = c;
				serial_buffer_index++;
			} else {
				#ifdef HIL_EXTENDED
				if (hil_serial_debug_enabled) {
					printf("SER RX OVERFLOW --- reset\n");
				}
				#endif
				memset(serial_buffer, 0, 256);
				serial_buffer_index = 0;
			}
		}
	}
}

void init_boot_button() {
	pinMode(0, INPUT_PULLUP);
}

void check_boot_button(){
	extern void trigger_self_test();
	extern self_test_steps_t self_test_step;

	if(t_now_ms >= 1000){ // Wait 1 second before checking boot button
		if(digitalRead(0) == LOW){ // Boot button is pressed
			if(self_test_step == SELF_TEST_INACTIVE){ // Self test is not already running
				printf("BOOT BUTTON PRESSED, BEGINNING SELF TEST\n");
				EMOTISCOPE_ACTIVE = true; // Wake if needed
				trigger_self_test(); // Begin self test
			}
		}
	}
}

void init_system() {
	extern void init_hardware_version_pins(); // (hardware_version.h)
	extern void init_light_mode_list();       // (light_modes.h)
	extern void init_leds();
	extern void init_i2s_microphone();
	extern void init_window_lookup();
	extern void init_goertzel_constants_musical();
	extern void init_goertzel_constants_full_spectrum();
	extern void init_tempo_goertzel_constants();
	extern void init_wifi();
	extern void init_filesystem();
	extern void init_rmt_driver();
	extern void init_indicator_light();
	extern void init_touch();
	extern void init_noise_samples();
	extern void init_floating_point_lookups();
	extern void init_vu();

	init_hardware_version_pins();       // (hardware_version.h)
	init_serial(921600);				// (system.h)
	init_light_mode_list();             // (light_modes.h)
	init_filesystem();                  // (filesystem.h)
	init_configuration();               // (configuration.h)
	init_i2s_microphone();				// (microphone.h)
	init_window_lookup();				// (goertzel.h)
	init_goertzel_constants_musical();	// (goertzel.h)
	init_tempo_goertzel_constants();	// (tempo.h)	
	#ifndef DISABLE_INDICATOR
	init_indicator_light();             // (indicator.h)
	#endif
	init_rmt_driver();                  // (led_driver.h)
	#ifndef DISABLE_TOUCH
	init_touch();                       // (touch.h)
	#endif
	init_wifi();                        // (wireless.h)
	init_noise_samples();               // (utilities.h)
	init_floating_point_lookups();      // (utilities.h)
	init_boot_button();                 // (system.h)
	init_vu();                          // (vu.h)

	// Load sliders 
	load_sliders_relevant_to_mode(configuration.current_mode);

	// Load toggles
	load_toggles_relevant_to_mode(configuration.current_mode);
}
