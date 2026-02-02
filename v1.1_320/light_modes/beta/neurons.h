void draw_neurons() {
	for (uint16_t i = 0; i < (NUM_LEDS>>1); i++) {
		float input_neuron_value = clip_float(input_neuron_values[i]);

		float hidden_neuron_2_value = clip_float(hidden_neuron_1_values[i>>1]*0.2);
		float hidden_neuron_3_value = clip_float(hidden_neuron_3_values[i>>1]*0.2);
		float output_neuron_value   = clip_float(output_neuron_values[i]);

		CRGBF color_network = {
			(hidden_neuron_2_value*hidden_neuron_2_value),
			(hidden_neuron_3_value*hidden_neuron_3_value),
			sqrt(output_neuron_value),
		};

		CRGBF color_spectral = {
			0,
			spectrogram_smooth[i],
			0,
		};

		leds[(NUM_LEDS>>1)+i] = color_network;
		leds[i] = color_spectral;
	}
}