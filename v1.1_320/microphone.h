/*
-----------------------------------------------------------------------------------------
             _                                 _                                  _
            (_)                               | |                                | |
 _ __ ___    _    ___   _ __    ___    _ __   | |__     ___    _ __     ___      | |__
| '_ ` _ \  | |  / __| | '__|  / _ \  | '_ \  | '_ \   / _ \  | '_ \   / _ \     | '_ \ 
| | | | | | | | | (__  | |    | (_) | | |_) | | | | | | (_) | | | | | |  __/  _  | | | |
|_| |_| |_| |_|  \___| |_|     \___/  | .__/  |_| |_|  \___/  |_| |_|  \___| (_) |_| |_|
                                      | |
                                      |_|

Functions for reading and storing data acquired by the I2S microphone
*/

#include "driver/i2s_std.h"
#include "driver/gpio.h"

// Define I2S pins (ESP32-S3-DevKitC-1 target hardware)
#define I2S_LRCLK_PIN 12
#define I2S_BCLK_PIN  14
#define I2S_DIN_PIN   13

#define CHUNK_SIZE 64
#define SAMPLE_RATE 12800

// SAMPLE_HISTORY_LENGTH is now defined in global_defines.h

float sample_history[SAMPLE_HISTORY_LENGTH];
const float recip_scale = 1.0 / 131072.0; // max 18 bit signed value

// DC Blocker filter state and coefficients
// y[n] = g * (x[n] - x[n-1] + R * y[n-1])
// R = exp(-2π * fc / fs), g = (1+R)/2 for unity gain at high frequencies
// fc = 5 Hz cutoff for fast settling while preserving bass content
// fs = 12800 Hz sample rate
#define DC_BLOCKER_FC 5.0f
#define DC_BLOCKER_R  0.997545f  // exp(-2*PI*5/12800) ≈ 0.997545
#define DC_BLOCKER_G  0.998772f  // (1 + R) / 2

static float dc_blocker_x_prev = 0.0f;  // Previous input sample
static float dc_blocker_y_prev = 0.0f;  // Previous output sample

// Clip counter for monitoring headroom (reset periodically)
static uint32_t clip_count = 0;
static uint32_t sample_count_since_clip_report = 0;

volatile bool waveform_locked = false;
volatile bool waveform_sync_flag = false;

i2s_chan_handle_t rx_handle;

void init_i2s_microphone(){
	// Get the default channel configuration
	i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);

	// Create a new RX channel and get the handle of this channel
	i2s_new_channel(&chan_cfg, NULL, &rx_handle);

	// Configuration for the I2S standard mode, suitable for the SPH0645 microphone
	// SPH0645 with SEL=3.3V outputs on RIGHT channel (WS HIGH in standard I2S)
	i2s_std_config_t std_cfg = {
		.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(12800), // Sample rate 12.8 kHz
		.slot_cfg = {
			.data_bit_width = I2S_DATA_BIT_WIDTH_32BIT, // 24-bit data in 32-bit container
			.slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT, // 32-bit slot width
			.slot_mode = I2S_SLOT_MODE_STEREO, // Stereo frame, but only reading one channel
			.slot_mask = I2S_STD_SLOT_RIGHT, // SPH0645 with SEL=3.3V → RIGHT channel
			.ws_width = 32, // WS signal width as 32 BCLK periods
			.ws_pol = false, // Standard I2S polarity: WS HIGH = RIGHT channel
			.bit_shift = true, // Standard I2S: MSB delayed by 1 BCLK after WS transition
			.left_align = true, // Data is left-aligned (MSB first)
			.big_endian = false, // Little endian
			.bit_order_lsb = false, // MSB received first
		},
		.gpio_cfg = {
			.mclk = I2S_GPIO_UNUSED,
			.bclk = (gpio_num_t)I2S_BCLK_PIN,
			.ws = (gpio_num_t)I2S_LRCLK_PIN,
			.dout = I2S_GPIO_UNUSED,
			.din = (gpio_num_t)I2S_DIN_PIN,
			.invert_flags = {
				.mclk_inv = false,
				.bclk_inv = false,
				.ws_inv = false,
			},
		},
	};



	// Initialize the channel
	i2s_channel_init_std_mode(rx_handle, &std_cfg);

	// Start the RX channel
	i2s_channel_enable(rx_handle);
}

void acquire_sample_chunk() {
	profile_function([&]() {
		// Buffer to hold audio samples
		uint32_t new_samples_raw[CHUNK_SIZE];
		float new_samples[CHUNK_SIZE];

		// Read audio samples into int32_t buffer, but **only when emotiscope is active**
		if( EMOTISCOPE_ACTIVE == true ){
			size_t bytes_read = 0;
			i2s_channel_read(rx_handle, new_samples_raw, CHUNK_SIZE*sizeof(uint32_t), &bytes_read, portMAX_DELAY);
		}
		else{
			memset(new_samples_raw, 0, sizeof(uint32_t) * CHUNK_SIZE);
		}

		// Convert raw I2S samples to float and apply DC blocker
		// Step 1: Shift to get signed 18-bit range, convert to float
		// Step 2: Apply DC blocking high-pass filter (removes hardware DC offset)
		// Step 3: Scale to -1.0 to 1.0 range with clipping protection

		for (uint16_t i = 0; i < CHUNK_SIZE; i++) {
			// Extract signed sample from I2S data (24-bit in 32-bit container, we use 18-bit range)
			float x = (float)(((int32_t)new_samples_raw[i]) >> 14);

			// DC Blocker: y[n] = g * (x[n] - x[n-1] + R * y[n-1])
			float y = DC_BLOCKER_G * (x - dc_blocker_x_prev + DC_BLOCKER_R * dc_blocker_y_prev);
			dc_blocker_x_prev = x;
			dc_blocker_y_prev = y;

			// Soft clip to 18-bit range and track clipping
			if (y > 131072.0f) {
				y = 131072.0f;
				clip_count++;
			} else if (y < -131072.0f) {
				y = -131072.0f;
				clip_count++;
			}

			new_samples[i] = y;
		}

		sample_count_since_clip_report += CHUNK_SIZE;

		// Convert audio from "18-bit" float range to -1.0 to 1.0 range
		dsps_mulc_f32(new_samples, new_samples, CHUNK_SIZE, recip_scale, 1, 1);

		// Diagnostic: Report DC blocker status and clip rate periodically
		static uint32_t last_diag_print = 0;
		if (t_now_ms - last_diag_print > 2000) {
			float clip_rate = (sample_count_since_clip_report > 0)
				? (100.0f * clip_count / sample_count_since_clip_report)
				: 0.0f;
			printf("I2S DC BLOCKER --- dc_estimate=%.1f clip_rate=%.3f%% (%lu/%lu)\n",
				dc_blocker_x_prev - dc_blocker_y_prev,  // Estimated DC offset being removed
				clip_rate,
				(unsigned long)clip_count,
				(unsigned long)sample_count_since_clip_report);
			clip_count = 0;
			sample_count_since_clip_report = 0;
			last_diag_print = t_now_ms;
		}

		// Add new chunk to audio history
		waveform_locked = true;
		shift_and_copy_arrays(sample_history, SAMPLE_HISTORY_LENGTH, new_samples, CHUNK_SIZE);

		// If debug recording was triggered
		if(audio_recording_live == true){
			int16_t out_samples[CHUNK_SIZE];
			for(uint16_t i = 0; i < CHUNK_SIZE; i += 4){
				out_samples[i+0] = new_samples[i+0] * 32767;
				out_samples[i+1] = new_samples[i+1] * 32767;
				out_samples[i+2] = new_samples[i+2] * 32767;
				out_samples[i+3] = new_samples[i+3] * 32767;
			}
			memcpy(&audio_debug_recording[audio_recording_index], out_samples, sizeof(int16_t)*CHUNK_SIZE);
			audio_recording_index += CHUNK_SIZE;
			if(audio_recording_index >= MAX_AUDIO_RECORDING_SAMPLES){
				audio_recording_index = 0;
				audio_recording_live = false;
				broadcast("debug_recording_ready");
				save_audio_debug_recording();
			}
		}
		
		// Used to sync GPU to this when needed
		waveform_locked = false;
		waveform_sync_flag = true;
	}, __func__);
}
