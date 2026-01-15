void run_web() {
	profile_function([&]() {
		handle_wifi();
		// dns_server removed - HIL build doesn't need captive portal

		#ifdef HIL_EXTENDED
		if (hil_serial_debug_enabled) {
			printf("CMDQ BEFORE ------ %u\n", (uint16_t)commands_queued);
		}
		#endif
		process_command_queue();
		#ifdef HIL_EXTENDED
		if (hil_serial_debug_enabled) {
			printf("CMDQ AFTER ------- %u\n", (uint16_t)commands_queued);
		}
		#endif

		if (web_server_ready == true) {
			sync_configuration_to_file_system();
		}
	}, __func__ );
}
