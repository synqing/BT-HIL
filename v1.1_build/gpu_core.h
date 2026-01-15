/*
-------------------------------------------------------------------------
                                                                   _
                                                                  | |
  __ _   _ __    _   _             ___    ___    _ __    ___      | |__
 / _` | | '_ \  | | | |           / __|  / _ \  | '__|  / _ \     | '_ \ 
| (_| | | |_) | | |_| |          | (__  | (_) | | |    |  __/  _  | | | |
 \__, | | .__/   \__,_|           \___|  \___/  |_|     \___| (_) |_| |_|
  __/ | | |              ______
 |___/  |_|             |______|

Main loop of the GPU core (Core 0)
*/

float lpf_drag = 0.0;

void run_gpu() {
	profile_function([&]() {
		// Get the current time in microseconds and milliseconds
		static uint32_t t_last_us = micros();

		t_now_us = micros();
		t_now_ms = millis();

		// Calculate the "delta" value, to scale movements based on FPS, like a game engine
		const uint32_t ideal_us_interval = (1000000 / REFERENCE_FPS);
		uint32_t t_elapsed_us = t_now_us - t_last_us;
		float delta = float(t_elapsed_us) / ideal_us_interval;

		// Save the current timestamp for next loop
		t_last_us = t_now_us;

		// Update the novelty curve
		update_novelty();  // (tempo.h)
		#ifdef HIL_EXTENDED
		if (hil_capture_state && hil_monitoring_active) {
			hil_capture_gpu_write_begin();
			memcpy(hil_capture_state->novelty_curve_capture, novelty_curve, sizeof(float) * NOVELTY_HISTORY_LENGTH);
			hil_capture_gpu_write_end();
		}
		#endif

		// Update the tempi phases
		update_tempi_phase(delta);	// (tempo.h)
		#ifdef HIL_EXTENDED
		if (hil_capture_state && hil_monitoring_active) {
			hil_capture_gpu_write_begin();
			for (uint8_t i = 0; i < NUM_TEMPI; i++) {
				hil_capture_state->tempi_phase_capture[i] = tempi[i].phase;
				hil_capture_state->tempi_beat_capture[i] = tempi[i].beat;
			}
			hil_capture_gpu_write_end();

			// Phase 4: Process WebSocket binary streaming subscriptions
			hil_ws_process_subscriptions(hil_capture_state);
		}
		#endif

		// Update auto color cycling
		update_auto_color();  // (leds.h)

		run_indicator_light();

		// RUN THE CURRENT MODE
		// ------------------------------------------------------------

		clear_display();
		#ifdef HIL_EXTENDED
		uint32_t t_draw_start = micros();
		#endif
		light_modes[configuration.current_mode].draw();
		#ifdef HIL_EXTENDED
		uint32_t t_draw_end = micros();
		#endif

		// If silence is detected, show a blue debug LED
		// leds[NUM_LEDS - 1] = add(leds[NUM_LEDS - 1], {0.0, 0.0, silence_level});

		apply_background(configuration.background);

		draw_ui_overlay();  // (ui.h)

		if( EMOTISCOPE_ACTIVE == true && configuration.screensaver == true){
			run_screensaver();
		}

		apply_brightness();

		if( EMOTISCOPE_ACTIVE == false ){
			run_standby();
		}

		render_touches();  // (touch.h)
		
		// This value decays itself non linearly toward zero all the time, 
		// *really* slowing down the LPF when it's set to 1.0.
		// This is a super hacky way to fake a true fade transition between modes
		lpf_drag *= 0.9975;

		if(lpf_drag < screensaver_mix*0.8){
			lpf_drag = screensaver_mix*0.8;
		}

		// Apply a low pass filter to every color channel of every pixel on every frame
		// at hundreds of frames per second
		//
		// To anyone who reads this: microcontrollers are fucking insane now.
		// When I got into all this in 2012, I had a 16MHz single core AVR
		//
		// The DMA and SIMD-style stuff inside the ESP32-S3 is some pretty crazy shit.
		float lpf_cutoff_frequency = 0.5 + (1.0-(sqrt(configuration.softness)))*14.5;
		lpf_cutoff_frequency = lpf_cutoff_frequency * (1.0 - lpf_drag) + 0.5 * lpf_drag;
		#ifdef HIL_EXTENDED
		uint32_t t_lpf_start = micros();
		#endif
		apply_image_lpf(lpf_cutoff_frequency);
		#ifdef HIL_EXTENDED
		uint32_t t_lpf_end = micros();
		#endif

		//clip_leds();
		#ifdef HIL_EXTENDED
		uint32_t t_tonemap_start = micros();
		#endif
		apply_tonemapping();
		#ifdef HIL_EXTENDED
		uint32_t t_tonemap_end = micros();
		#endif

		//apply_frame_blending( configuration.softness );
		//apply_phosphor_decay( configuration.softness );

		// Apply an incandescent LUT to reduce harsh blue tones
		#ifdef HIL_EXTENDED
		uint32_t t_warmth_start = micros();
		#endif
		apply_warmth( configuration.warmth );  // (leds.h)
		#ifdef HIL_EXTENDED
		uint32_t t_warmth_end = micros();
		#endif

		// Apply white balance
		multiply_CRGBF_array_by_LUT( leds, WHITE_BALANCE, NUM_LEDS );

		#ifdef HIL_EXTENDED
		uint32_t t_brightness_start = micros();
		#endif
		apply_master_brightness();
		#ifdef HIL_EXTENDED
		uint32_t t_brightness_end = micros();
		#endif

		#ifdef HIL_EXTENDED
		uint32_t t_gamma_start = micros();
		#endif
		apply_gamma_correction();
		#ifdef HIL_EXTENDED
		uint32_t t_gamma_end = micros();
		#endif

		// Quantize the image buffer with dithering,
		// output to the 8-bit LED strand
		#ifdef HIL_EXTENDED
		uint32_t t_transmit_start = micros();
		#endif
		transmit_leds();
		#ifdef HIL_EXTENDED
		uint32_t t_transmit_end = micros();

		// Phase 6: GPU Pipeline Timing Instrumentation
		// Accumulate timing and broadcast averages every 250ms (4Hz)
		{
			static uint32_t timing_acc_draw = 0;
			static uint32_t timing_acc_lpf = 0;
			static uint32_t timing_acc_tonemap = 0;
			static uint32_t timing_acc_warmth = 0;
			static uint32_t timing_acc_brightness = 0;
			static uint32_t timing_acc_gamma = 0;
			static uint32_t timing_acc_transmit = 0;
			static uint32_t timing_frame_count = 0;
			static uint32_t timing_last_broadcast_ms = 0;

			// Accumulate this frame's timings
			timing_acc_draw += (t_draw_end - t_draw_start);
			timing_acc_lpf += (t_lpf_end - t_lpf_start);
			timing_acc_tonemap += (t_tonemap_end - t_tonemap_start);
			timing_acc_warmth += (t_warmth_end - t_warmth_start);
			timing_acc_brightness += (t_brightness_end - t_brightness_start);
			timing_acc_gamma += (t_gamma_end - t_gamma_start);
			timing_acc_transmit += (t_transmit_end - t_transmit_start);
			timing_frame_count++;

			// Broadcast every 250ms
			if (hil_monitoring_active && (t_now_ms - timing_last_broadcast_ms >= 250)) {
				if (timing_frame_count > 0) {
					// Calculate averages
					uint32_t avg_draw = timing_acc_draw / timing_frame_count;
					uint32_t avg_lpf = timing_acc_lpf / timing_frame_count;
					uint32_t avg_tonemap = timing_acc_tonemap / timing_frame_count;
					uint32_t avg_warmth = timing_acc_warmth / timing_frame_count;
					uint32_t avg_brightness = timing_acc_brightness / timing_frame_count;
					uint32_t avg_gamma = timing_acc_gamma / timing_frame_count;
					uint32_t avg_transmit = timing_acc_transmit / timing_frame_count;

					// Broadcast combined timing message
					static char timing_msg[128];
					snprintf(timing_msg, sizeof(timing_msg),
						"hil_timing|draw|%lu|lpf|%lu|tonemap|%lu|warmth|%lu|brightness|%lu|gamma|%lu|transmit|%lu",
						avg_draw, avg_lpf, avg_tonemap, avg_warmth, avg_brightness, avg_gamma, avg_transmit);
					broadcast(timing_msg);

					// Reset accumulators
					timing_acc_draw = 0;
					timing_acc_lpf = 0;
					timing_acc_tonemap = 0;
					timing_acc_warmth = 0;
					timing_acc_brightness = 0;
					timing_acc_gamma = 0;
					timing_acc_transmit = 0;
					timing_frame_count = 0;
				}
				timing_last_broadcast_ms = t_now_ms;
			}
		}
		#endif

		// Update the FPS_GPU variable
		watch_gpu_fps();  // (system.h)
	}, __func__ );
}
