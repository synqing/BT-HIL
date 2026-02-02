void draw_debug_novelty(){
	for(uint16_t i = 0; i < NUM_LEDS; i++){
		int32_t index = ((NOVELTY_HISTORY_LENGTH-1)-NUM_LEDS)+i;
		float mag_vu = vu_curve[index];
		float mag_spec = novelty_curve_normalized[index];

		CRGBF dot_color = {
			mag_vu,
			mag_spec,
			0.0,
		};

		leds[i] = dot_color;
	}
}

void draw_debug(){
	for(uint16_t i = 0; i < (NUM_LEDS>>1); i++){
		float progress = num_leds_float_lookup[i<<1];
		leds[i] = hsv(
			get_color_range_hue(progress),
			1.0,
			1.0
		);
	}
	for(uint16_t i = 0; i < (NUM_LEDS>>1); i++){
		float progress = float(i) / (NUM_LEDS>>1);
		leds[(NUM_LEDS>>1)+i] = hsv(
			get_color_range_hue(progress),
			1.0,
			1.0 - progress
		);
	}
}