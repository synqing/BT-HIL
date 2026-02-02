#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef HIL_EXTENDED
#include "hil_capture.h"
#include "hil_export.h"
extern hil_capture_state_t* hil_capture_state;
extern hil_export_state_t* hil_export_state;
extern volatile bool hil_monitoring_active;
#endif

#define COMMAND_QUEUE_SLOTS (32)

command command_queue[COMMAND_QUEUE_SLOTS];
uint16_t commands_queued = 0;

extern float clip_float(float input);
extern int16_t set_lightshow_mode_by_name(char* name);
extern void transmit_to_client_in_slot(const char* message, uint8_t client_slot);
// reboot_into_wifi_config_mode removed - HIL build doesn't need WiFi config mode

// Function to return the selected index as a null-terminated string with custom delimiter
// The result is stored in the provided buffer
bool load_substring_from_split_index(const char* input, int index, char* result, size_t result_size, char delimiter = '|') {
    // Ensure input and result buffer are not NULL
    if (input == NULL || result == NULL) {
        return false;
    }

    int len = strlen(input);
    int start = 0;
    int end = 0;
    int current_index = 0;

    // Iterate over the input to find the desired index
    for (int i = 0; i <= len; i++) {
        // Check for custom delimiter or end of string
        if (input[i] == delimiter || input[i] == '\0') {
            if (current_index == index) {
                end = i;
                break;
            }
            start = i + 1;
            current_index++;
        }
    }

    // Check if the index was not found or if the result buffer is too small
    int segment_length = end - start;
    if (current_index != index || segment_length + 1 > result_size) {
        return false;
    }

    // Copy the substring to the result buffer
	memset(result, 0, result_size);
    strncpy(result, input + start, segment_length);
    result[segment_length] = '\0';

    return true;
}

void shift_command_queue_left() {
	memmove(command_queue, command_queue + 1, (COMMAND_QUEUE_SLOTS - 1) * (sizeof(command)));
	memset(command_queue + COMMAND_QUEUE_SLOTS - 1, 0, 1 * sizeof(command));
}

void unrecognized_command_error(char* command){
	printf("UNRECOGNIZED COMMAND: %s\n", command);
}

static inline void transmit_or_print_to_origin(const char* message, command com) {
	if (com.origin_client_slot == 255) {
		printf("%s\n", message);
		return;
	}

	transmit_to_client_in_slot(message, com.origin_client_slot);
}

static void handle_hil_command(uint32_t t_now_ms, command com) {
#ifdef HIL_EXTENDED
	fetch_substring(com.command, '|', 1);

	if (fastcmp(substring, "help")) {
		transmit_or_print_to_origin("hil_help|hil|status,hil|monitoring|0|1,hil|snapshot", com);
		return;
	}

	if (fastcmp(substring, "status")) {
		char msg[128];
		snprintf(
			msg,
			sizeof(msg),
			"hil_status|%u|%u|%u",
			(uint16_t)1,
			(uint16_t)(hil_capture_state != NULL),
			(uint16_t)(hil_monitoring_active ? 1 : 0)
		);
		transmit_or_print_to_origin(msg, com);
		return;
	}

	if (fastcmp(substring, "monitoring")) {
		fetch_substring(com.command, '|', 2);
		int v = atoi(substring);
		hil_monitoring_active = (v != 0);
		transmit_or_print_to_origin(hil_monitoring_active ? "hil_monitoring|1" : "hil_monitoring|0", com);
		return;
	}

	if (fastcmp(substring, "snapshot")) {
		hil_capture_cpu_snapshot_t snapshot;
		if (!hil_capture_read_cpu_snapshot(&snapshot)) {
			transmit_or_print_to_origin("hil_snapshot|unavailable", com);
			return;
		}

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

		uint16_t bpm0 = TEMPO_LOW + top_idx[0];
		uint16_t bpm1 = TEMPO_LOW + top_idx[1];
		uint16_t bpm2 = TEMPO_LOW + top_idx[2];

		char msg[256];
		snprintf(
			msg,
			sizeof(msg),
			"hil_snapshot|%lu|%.3f|%.3f|%.3f|%u|%.3f|%u|%.3f|%u|%.3f",
			(unsigned long)t_now_ms,
			snapshot.vu_level,
			snapshot.vu_max,
			snapshot.vu_floor,
			bpm0, top_mag[0],
			bpm1, top_mag[1],
			bpm2, top_mag[2]
		);
		transmit_or_print_to_origin(msg, com);
		return;
	}

	transmit_or_print_to_origin("hil_error|unrecognized", com);
#else
	(void)t_now_ms;
	transmit_or_print_to_origin("hil_unavailable", com);
#endif
}

// Handle log|* commands for CSV export (Task 2.2, 2.4)
static void handle_log_command(uint32_t t_now_ms, command com) {
#ifdef HIL_EXTENDED
	fetch_substring(com.command, '|', 1);

	// log|help - Show available commands
	if (fastcmp(substring, "help")) {
		transmit_or_print_to_origin("log_help|log|start|serial,log|start|file|<frames>,log|stop,log|status", com);
		return;
	}

	// log|start|serial or log|start|file|<max_frames>
	if (fastcmp(substring, "start")) {
		fetch_substring(com.command, '|', 2);

		if (fastcmp(substring, "serial")) {
			if (hil_export_start_serial()) {
				hil_export_state->origin_client_slot = com.origin_client_slot;
				transmit_or_print_to_origin("log_started|serial", com);
			} else {
				transmit_or_print_to_origin("log_error|already_active", com);
			}
			return;
		}

		if (fastcmp(substring, "file") || fastcmp(substring, "littlefs")) {
			// Optional: max frames parameter
			fetch_substring(com.command, '|', 3);
			uint32_t max_frames = 500;  // Default
			if (strlen(substring) > 0) {
				max_frames = atoi(substring);
				if (max_frames < 10) max_frames = 10;
				if (max_frames > 5000) max_frames = 5000;  // LittleFS limit
			}

			if (hil_export_start_littlefs(max_frames)) {
				hil_export_state->origin_client_slot = com.origin_client_slot;
				char msg[128];
				snprintf(msg, sizeof(msg), "log_started|file|%s|%lu",
						 hil_export_state->csv_filename, max_frames);
				transmit_or_print_to_origin(msg, com);
			} else {
				transmit_or_print_to_origin("log_error|failed_to_create_file", com);
			}
			return;
		}

		transmit_or_print_to_origin("log_error|unknown_mode|use_serial_or_file", com);
		return;
	}

	// log|stop
	if (fastcmp(substring, "stop")) {
		if (hil_export_state && hil_export_state->export_active) {
			uint32_t frames = hil_export_state->frame_counter;
			hil_export_stop();
			char msg[128];
			snprintf(msg, sizeof(msg), "log_stopped|%lu", frames);
			transmit_or_print_to_origin(msg, com);
		} else {
			transmit_or_print_to_origin("log_error|not_active", com);
		}
		return;
	}

	// log|status
	if (fastcmp(substring, "status")) {
		char msg[256];
		hil_export_get_status(msg, sizeof(msg));
		transmit_or_print_to_origin(msg, com);
		return;
	}

	transmit_or_print_to_origin("log_error|unrecognized", com);
#else
	(void)t_now_ms;
	transmit_or_print_to_origin("log_unavailable", com);
#endif
}

// Handle export|* commands for JSON/binary export (Phase 3)
static void handle_export_command(uint32_t t_now_ms, command com) {
#ifdef HIL_EXTENDED
	fetch_substring(com.command, '|', 1);

	// export|help
	if (fastcmp(substring, "help")) {
		transmit_or_print_to_origin("export_help|export|json", com);
		return;
	}

	// export|json - Export JSON snapshot to LittleFS
	if (fastcmp(substring, "json")) {
		if (hil_capture_state && hil_export_json_snapshot(hil_capture_state)) {
			transmit_or_print_to_origin("export_json|success", com);
		} else {
			transmit_or_print_to_origin("export_json|failed", com);
		}
		return;
	}

	transmit_or_print_to_origin("export_error|unrecognized", com);
#else
	(void)t_now_ms;
	transmit_or_print_to_origin("export_unavailable", com);
#endif
}

// Handle subscribe|* commands for WebSocket binary streaming (Phase 4)
static void handle_subscribe_command(uint32_t t_now_ms, command com) {
#ifdef HIL_EXTENDED
	// Parse the full command to extract subscription types
	hil_ws_parse_subscription(com.command);

	char msg[256];
	snprintf(msg, sizeof(msg), "subscribed|spec=%d|tempi=%d|chroma=%d|vu=%d",
			 hil_ws_subscription.spectrogram,
			 hil_ws_subscription.tempi,
			 hil_ws_subscription.chromagram,
			 hil_ws_subscription.vu);
	transmit_or_print_to_origin(msg, com);
#else
	(void)t_now_ms;
	transmit_or_print_to_origin("subscribe_unavailable", com);
#endif
}

// Handle unsubscribe command
static void handle_unsubscribe_command(uint32_t t_now_ms, command com) {
#ifdef HIL_EXTENDED
	hil_ws_unsubscribe_all();
	transmit_or_print_to_origin("unsubscribed|all", com);
#else
	(void)t_now_ms;
	transmit_or_print_to_origin("unsubscribe_unavailable", com);
#endif
}

void parse_command(uint32_t t_now_ms, command com) {
	//printf("Parsing command: '%s'\n", com.command);
	
    fetch_substring(com.command, '|', 0);
	//printf("command type: %s\n", substring);

	if (fastcmp(substring, "set")) {
		// Get setting name
		fetch_substring(com.command, '|', 1);
		
		if (fastcmp(substring, "brightness")) {
			// Get brightness value
			fetch_substring(com.command, '|', 2);
			float setting_value = clip_float(atof(substring));
			configuration.brightness = setting_value;

			update_ui(UI_NEEDLE_EVENT, configuration.brightness);
		}
		else if (fastcmp(substring, "softness")) {
			// Get softness value
			fetch_substring(com.command, '|', 2);
			float setting_value = atof(substring);
			configuration.softness = setting_value;

			update_ui(UI_NEEDLE_EVENT, configuration.softness);
		}
		else if (fastcmp(substring, "speed")) {
			// Get speed value
			fetch_substring(com.command, '|', 2);
			float setting_value = atof(substring);
			configuration.speed = setting_value;

			update_ui(UI_NEEDLE_EVENT, configuration.speed);
		}
		else if (fastcmp(substring, "color")) {
			// Get color value
			fetch_substring(com.command, '|', 2);
			float setting_value = clip_float(atof(substring));
			configuration.color = setting_value;

			//update_ui(UI_HUE_EVENT, configuration.hue); 
		}
		else if (fastcmp(substring, "mirror_mode")) {
			// Get mirror_mode value
			fetch_substring(com.command, '|', 2);
			bool setting_value = (bool)atoi(substring);
			configuration.mirror_mode = setting_value;
		}
		else if (fastcmp(substring, "warmth")) {
			// Get warmth value
			fetch_substring(com.command, '|', 2);
			float setting_value = atof(substring);
			configuration.warmth = setting_value;

			update_ui(UI_NEEDLE_EVENT, configuration.warmth);
		}
		else if (fastcmp(substring, "color_range")) {
			// Get color_range value
			fetch_substring(com.command, '|', 2);
			float setting_value = atof(substring);
			configuration.color_range = setting_value;

			//update_ui(UI_HUE_EVENT, configuration.hue);
		}
		else if (fastcmp(substring, "saturation")) {
			// Get saturation value
			fetch_substring(com.command, '|', 2);
			float setting_value = atof(substring);
			configuration.saturation = sqrt(clip_float(setting_value));

			//update_ui(UI_NEEDLE_EVENT, configuration.saturation);
		}
		else if (fastcmp(substring, "background")) {
			// Get background value
			fetch_substring(com.command, '|', 2);
			float setting_value = atof(substring);
			configuration.background = setting_value;

			update_ui(UI_NEEDLE_EVENT, configuration.background);
		}
		else if (fastcmp(substring, "screensaver")) {
			// Get screensaver value
			fetch_substring(com.command, '|', 2);
			bool setting_value = (bool)atoi(substring);
			configuration.screensaver = setting_value;
		}
		else if (fastcmp(substring, "temporal_dithering")){
			// Get temporal_dithering value
			fetch_substring(com.command, '|', 2);
			bool setting_value = (bool)atoi(substring);
			configuration.temporal_dithering = setting_value;
		}
		else if (fastcmp(substring, "reverse_color_range")){
			// Get reverse_color_range value
			fetch_substring(com.command, '|', 2);
			bool setting_value = (bool)atoi(substring);
			configuration.reverse_color_range = setting_value;
		}
		else if (fastcmp(substring, "auto_color_cycle")){
			// Get auto_color_cycle value
			fetch_substring(com.command, '|', 2);
			bool setting_value = (bool)atoi(substring);
			configuration.auto_color_cycle = setting_value;
		}

		else if (fastcmp(substring, "mode")) {
			// Get mode name
			fetch_substring(com.command, '|', 2);

			int16_t mode_index = set_light_mode_by_name(substring);
			if(mode_index == -1){
				unrecognized_command_error(substring);
			}
			else{
				load_sliders_relevant_to_mode(mode_index);
				load_toggles_relevant_to_mode(mode_index);
				broadcast("reload_config");
			}
		}
		else{
			unrecognized_command_error(substring);
		}

		// Open a save request for later
		save_config_delayed();
	}
	else if (fastcmp(substring, "get")) {
		// Name of thing to get
		fetch_substring(com.command, '|', 1);
		
		// If getting configuration struct contents
		if (fastcmp(substring, "config")) {
			// Wake on command
			EMOTISCOPE_ACTIVE = true;
			sync_configuration_to_client();
			load_menu_toggles();
		}

		// If getting mode list
		else if (fastcmp(substring, "modes")) {
			transmit_to_client_in_slot("clear_modes", com.origin_client_slot);

			uint16_t num_modes = sizeof(light_modes) / sizeof(light_mode);
			for(uint16_t mode_index = 0; mode_index < num_modes; mode_index++){
				char command_string[128];
				uint8_t mode_type = (uint8_t)light_modes[mode_index].type;

				snprintf(command_string, 128, "new_mode|%d|%d|%.64s", mode_index, mode_type, light_modes[mode_index].name);
				transmit_to_client_in_slot(command_string, com.origin_client_slot);
			}

			transmit_to_client_in_slot("modes_ready", com.origin_client_slot);
		}
		// If getting slider list
		else if (fastcmp(substring, "sliders")) {
			transmit_to_client_in_slot("clear_sliders", com.origin_client_slot);

			for(uint16_t i = 0; i < sliders_active; i++){
				char command_string[128];
				snprintf(command_string, 128, "new_slider|%s|%.3f|%.3f|%.3f", sliders[i].name, sliders[i].slider_min, sliders[i].slider_max, sliders[i].slider_step);
				transmit_to_client_in_slot(command_string, com.origin_client_slot);
			}

			transmit_to_client_in_slot("sliders_ready", com.origin_client_slot);
		}

		// If getting toggle list
		else if (fastcmp(substring, "toggles")) {
			transmit_to_client_in_slot("clear_toggles", com.origin_client_slot);

			for(uint16_t i = 0; i < toggles_active; i++){
				char command_string[128];
				snprintf(command_string, 128, "new_toggle|%s", toggles[i].name);
				transmit_to_client_in_slot(command_string, com.origin_client_slot);
			}

			transmit_to_client_in_slot("toggles_ready", com.origin_client_slot);
		}

		// If getting menu toggle list
		else if (fastcmp(substring, "menu_toggles")) {
			transmit_to_client_in_slot("clear_menu_toggles", com.origin_client_slot);

			for(uint16_t i = 0; i < menu_toggles_active; i++){
				char command_string[128];
				snprintf(command_string, 128, "new_menu_toggle|%s", menu_toggles[i].name);
				transmit_to_client_in_slot(command_string, com.origin_client_slot);
			}

			transmit_to_client_in_slot("menu_toggles_ready", com.origin_client_slot);
		}

		// If getting touch values
		else if (fastcmp(substring, "touch_vals")) {
			char command_string[128];
			snprintf(command_string, 128, "touch_vals|%lu|%lu|%lu", uint32_t(touch_pins[0].touch_value), uint32_t(touch_pins[1].touch_value), uint32_t(touch_pins[2].touch_value));
			transmit_to_client_in_slot(command_string, com.origin_client_slot);
		}

		// If getting version number
		else if (fastcmp(substring, "version")) {
			char command_string[128];
			snprintf(command_string, 128, "version|%d.%d.%d", SOFTWARE_VERSION_MAJOR, SOFTWARE_VERSION_MINOR, SOFTWARE_VERSION_PATCH);
			transmit_to_client_in_slot(command_string, com.origin_client_slot);
		}

		// Couldn't figure out what to "get"
		else{
			unrecognized_command_error(substring);
		}
	}
	else if (fastcmp(substring, "hil")) {
		handle_hil_command(t_now_ms, com);
	}
	else if (fastcmp(substring, "log")) {
		handle_log_command(t_now_ms, com);
	}
	#ifdef HIL_EXTENDED
	else if (fastcmp(substring, "export")) {
		handle_export_command(t_now_ms, com);
	}
	else if (fastcmp(substring, "subscribe")) {
		handle_subscribe_command(t_now_ms, com);
	}
	else if (fastcmp(substring, "unsubscribe")) {
		handle_unsubscribe_command(t_now_ms, com);
	}
	#endif
	else if (fastcmp(substring, "reboot")) {
		transmit_to_client_in_slot("disconnect_immediately", com.origin_client_slot);
		printf("Device was instructed to reboot! Please wait...\n");
		delay(100);
		ESP.restart();
	}
	// reboot_wifi_config command removed - HIL build has hardcoded WiFi credentials
	else if (fastcmp(substring, "button_tap")) {
		printf("REMOTE TAP TRIGGER\n");
		if(EMOTISCOPE_ACTIVE == true){
			int16_t next_mode_index = increment_mode();
			load_sliders_relevant_to_mode(next_mode_index);
			load_toggles_relevant_to_mode(next_mode_index);
			broadcast("reload_config");
		}
		else{
			toggle_standby();
		}
	}
	else if (fastcmp(substring, "button_hold")) {
		printf("REMOTE HOLD TRIGGER\n");
		toggle_standby();
	}
	else if (fastcmp(substring, "ping")) {
		transmit_to_client_in_slot("pong", com.origin_client_slot);
	}
	else if (fastcmp(substring, "touch_start")) {
		printf("APP TOUCH START\n");
		app_touch_active = true;
	}
	else if (fastcmp(substring, "touch_end")) {
		printf("APP TOUCH END\n");
		app_touch_active = false;
	}
	else if (fastcmp(substring, "slider_touch_start")) {
		slider_touch_active = true;
		//update_ui(UI_SHOW_EVENT);
	}
	else if (fastcmp(substring, "slider_touch_end")) {
		slider_touch_active = false;
	}
	else if (fastcmp(substring, "check_update")) {
		extern bool check_update();
		if(check_update() == true){ // Update available
			transmit_to_client_in_slot("update_available", com.origin_client_slot);
		}
		else{
			transmit_to_client_in_slot("no_updates", com.origin_client_slot);
		}
	}
	else if (fastcmp(substring, "perform_update")) {
		extern void perform_update(int16_t client_slot);
		perform_update(com.origin_client_slot);
	}
	else if (fastcmp(substring, "self_test")) {
		if(t_now_ms >= 1000){ // Wait 1 second before checking boot button
			if(self_test_step == SELF_TEST_INACTIVE){ // Self test is not already running
				printf("SELF TEST TRIGGERED FROM APP, BEGINNING SELF TEST\n");
				EMOTISCOPE_ACTIVE = true; // Wake if needed
				trigger_self_test(); // Begin self test
			}
		}
	}
	else if(fastcmp(substring, "increment_mode")){
		int16_t next_mode_index = increment_mode();
		load_sliders_relevant_to_mode(next_mode_index);
		load_toggles_relevant_to_mode(next_mode_index);
		broadcast("reload_config");
	}
	else if (fastcmp(substring, "start_debug_recording")) {
		audio_recording_index = 0;
		memset(audio_debug_recording, 0, sizeof(int16_t)*MAX_AUDIO_RECORDING_SAMPLES);
		audio_recording_live = true;
	}
	else{
		unrecognized_command_error(substring);
	}

	// printf("current brightness value: %.3f\n", configuration.brightness);
}

void process_command_queue() {
	if (commands_queued > 0) {
		parse_command(t_now_ms, command_queue[0]);
		shift_command_queue_left();
		commands_queued -= 1;
	}
}

bool queue_command(const char* command, uint16_t length, uint8_t client_slot) {
	if (length < MAX_COMMAND_LENGTH) {
		if (commands_queued < COMMAND_QUEUE_SLOTS - 1) {
			memset(command_queue[commands_queued].command, 0, MAX_COMMAND_LENGTH);
			memcpy(command_queue[commands_queued].command, command, length);
			command_queue[commands_queued].origin_client_slot = client_slot;
			commands_queued += 1;
		}
		else {
			return false;
		}
	}
	else {
		return false;
	}

	return true;
}
