/*
------------------------------------------------------------------------
                                                                  _
                                                                 | |
  ___   _ __    _   _             ___    ___    _ __    ___      | |__
 / __| | '_ \  | | | |           / __|  / _ \  | '__|  / _ \     | '_ \ 
| (__  | |_) | | |_| |          | (__  | (_) | | |    |  __/  _  | | | |
 \___| | .__/   \__,_|           \___|  \___/  |_|     \___| (_) |_| |_|
       | |              ______
       |_|             |______|

Main loop of the CPU core (Core 1)
*/

void run_cpu() {
	profile_function([&]() {
		//------------------------------------------------------------------------------------------
		// TIMING
		// ----------------------------------------------------------------------------------
		static uint32_t iter = 0;
		iter++;

		//------------------------------------------------------------------------------------------
		// AUDIO CALCULATIONS
		// ----------------------------------------------------------------------

		// Get new audio chunk from the I2S microphone
		acquire_sample_chunk();	 // (microphone.h)

		#ifdef HIL_EXTENDED
		// Task 1.6: Capture raw I2S samples every 10th frame (~20 Hz) to reduce data volume
		if (hil_capture_state && hil_monitoring_active) {
			if (hil_capture_state->sample_history_capture_counter++ % 10 == 0) {
				hil_capture_cpu_write_begin();
				memcpy(hil_capture_state->sample_history_capture, sample_history, SAMPLE_HISTORY_LENGTH * sizeof(float));
				hil_capture_cpu_write_end();
			}
		}
		#endif

		uint32_t processing_start_us = micros();

		// Calculate the magnitudes of the currently studied frequency set
		calculate_magnitudes();  // (goertzel.h)

		get_chromagram();        // (goertzel.h)

		run_vu(); // (vu.h)

		//printf("update_tempo() = %.4fus\n", measure_execution([&]() {
		// Log novelty and calculate the most probable tempi
		update_tempo();	 // (tempo.h)
		#ifdef HIL_EXTENDED
		static uint32_t hil_capture_overhead_sum_us = 0;
		static uint32_t hil_capture_overhead_samples = 0;
		static uint32_t hil_capture_overhead_next_broadcast_ms = 0;

		if (hil_capture_state && hil_monitoring_active) {
			uint32_t cap_start_us = micros();

			hil_capture_cpu_write_begin();

			memcpy(hil_capture_state->spectrogram_capture, spectrogram, sizeof(float) * NUM_FREQS);
			memcpy(hil_capture_state->spectrogram_smooth_capture, spectrogram_smooth, sizeof(float) * NUM_FREQS);
			memcpy(hil_capture_state->chromagram_capture, chromagram, sizeof(float) * 12);

			hil_capture_state->vu_level_capture = vu_level;
			hil_capture_state->vu_max_capture = vu_max;
			hil_capture_state->vu_floor_capture = vu_floor;

			for (uint8_t i = 0; i < NUM_TEMPI; i++) {
				hil_capture_state->tempi_magnitude_capture[i] = tempi[i].magnitude;
			}
			memcpy(hil_capture_state->novelty_curve_normalized_capture, novelty_curve_normalized, sizeof(float) * NOVELTY_HISTORY_LENGTH);

			hil_capture_cpu_write_end();

			// Task 2.3: Export captured data to CSV (if export is active)
			hil_export_process_frame(hil_capture_state);

			uint32_t cap_end_us = micros();
			hil_capture_overhead_sum_us += (cap_end_us - cap_start_us);
			hil_capture_overhead_samples += 1;

			uint32_t now_ms = millis();
			if (now_ms >= hil_capture_overhead_next_broadcast_ms) {
				hil_capture_overhead_next_broadcast_ms = now_ms + 250;
				if (hil_capture_overhead_samples > 0) {
					uint32_t avg_us = hil_capture_overhead_sum_us / hil_capture_overhead_samples;
					char stat_buffer[64] = {0};
					snprintf(stat_buffer, sizeof(stat_buffer), "hil_capture_overhead_us|%lu", (unsigned long)avg_us);
					broadcast(stat_buffer);
				}
				hil_capture_overhead_sum_us = 0;
				hil_capture_overhead_samples = 0;
			}
		} else if (!hil_monitoring_active) {
			hil_capture_overhead_sum_us = 0;
			hil_capture_overhead_samples = 0;
		}
		#endif
		//}));

		// Update the FPS_CPU variable
		watch_cpu_fps();  // (system.h)

		// Occasionally print the average frame rate
		print_system_info();

		// print_audio_data();

		#ifndef DISABLE_TOUCH
		read_touch();
		#endif

		check_serial();

		check_boot_button();

		//neural_network_feed_forward();

		//------------------------------------------------------------------------------------------
		// WIFI
		// ------------------------------------------------------------------------------------
		//run_wireless();	 // (wireless.h)

		//------------------------------------------------------------------------------------------
		// TESTING AREA, SHOULD BE BLANK IN PRODUCTION

		/*
		printf("MATCH STRCMP    | microseconds taken = %.4f\n", measure_execution([&]() {
			(strcmp("test_string_value", "test_string_value") == 0);
		}));
		*/

		//------------------------------------------------------------------------------------------	

		//------------------------------------------------------------------------------------------
		// CPU USAGE CALCULATION
		// -------------------------------------------------------------------
		uint32_t processing_end_us = micros();
		uint32_t processing_us_spent = processing_end_us - processing_start_us;
		uint32_t audio_core_us_per_loop = 1000000.0 / FPS_CPU;
		float audio_frame_to_processing_ratio = processing_us_spent / float(audio_core_us_per_loop);
		CPU_CORE_USAGE = audio_frame_to_processing_ratio;

		//------------------------------------------------------------------------------------------
		yield();  // Keep CPU watchdog happy

	}, __func__);
}
