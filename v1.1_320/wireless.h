// HIL Mode - Simplified wireless configuration
#define MAX_NETWORK_CONNECT_ATTEMPTS (3)

String WEB_VERSION = "";

// Define a char array to hold the formatted MAC address string
char mac_str[18]; // MAC address string format "XX:XX:XX:XX:XX:XX" + '\0'

// HIL CSV export flag - suppress system logging during serial CSV export
extern volatile bool hil_csv_serial_export_active;

PsychicHttpServer server;
PsychicWebSocketHandler websocket_handler;
websocket_client websocket_clients[MAX_WEBSOCKET_CLIENTS];

volatile bool web_server_ready = false;
int16_t connection_status = -1;

uint8_t network_connection_attempts = 0;

int16_t get_slot_of_client(PsychicWebSocketClient client) {
	for (uint16_t i = 0; i < MAX_WEBSOCKET_CLIENTS; i++) {
		if (websocket_clients[i].socket == client.socket()) {
			return i;
		}
	}

	return -1;
}

PsychicWebSocketClient *get_client_in_slot(uint8_t slot) {
	PsychicWebSocketClient *client = websocket_handler.getClient(websocket_clients[slot].socket);
	if (client != NULL) {
		return client;
	}

	return NULL;
}

void init_websocket_clients() {
	for (uint16_t i = 0; i < MAX_WEBSOCKET_CLIENTS; i++) {
		websocket_clients[i] = {
			-1,	 // int socket;
			0,	 // uint32_t last_ping;
		};
	}
}

bool welcome_websocket_client(PsychicWebSocketClient client) {
	bool client_welcome_status = true;
	//uint32_t t_now_ms = millis();

	uint16_t current_client_count = 0;
	int16_t first_open_slot = -1;
	for (uint16_t i = 0; i < MAX_WEBSOCKET_CLIENTS; i++) {
		if (websocket_clients[i].socket != -1) {
			current_client_count += 1;
		}
		else {
			if (first_open_slot == -1) {
				first_open_slot = i;
			}
		}
	}

	// If no room left for new clients
	if (current_client_count >= MAX_WEBSOCKET_CLIENTS || first_open_slot == -1) {
		client_welcome_status = false;
	}

	// If there is room in the party, client is welcome and should be initialized
	if (client_welcome_status == true) {
		websocket_clients[first_open_slot] = {
			client.socket(),  // int socket;
			t_now_ms,		  // uint32_t last_ping;
		};
		if (!hil_csv_serial_export_active) printf("PLAYER WELCOMED INTO OPEN SLOT #%i\n", first_open_slot);
	}

	return client_welcome_status;
}

void websocket_client_left(uint16_t client_index) {
	if (!hil_csv_serial_export_active) printf("PLAYER #%i LEFT\n", client_index);
	PsychicWebSocketClient *client = get_client_in_slot(client_index);
	if (client != NULL) {
		client->close();
	}

	websocket_clients[client_index].socket = -1;
}

void websocket_client_left(PsychicWebSocketClient client) {
	int socket = client.socket();
	for (uint16_t i = 0; i < MAX_WEBSOCKET_CLIENTS; i++) {
		if (websocket_clients[i].socket == socket) {
			websocket_client_left((uint16_t)i);
			break;
		}
	}
}

void check_if_websocket_client_still_present(uint16_t client_slot) {
	if (websocket_clients[client_slot].socket != -1) {
		// make sure our client is still connected.
		PsychicWebSocketClient *client = get_client_in_slot(client_slot);
		if (client == NULL) {
			websocket_client_left(client_slot);
		}
	}
}

void transmit_to_client_in_slot(const char *message, uint8_t client_slot) {
	PsychicWebSocketClient *client = get_client_in_slot(client_slot);
	if (client != NULL) {
		client->sendMessage(message);
	}
}

void init_web_server() {
	server.config.max_uri_handlers = 20;  // maximum number of .on() calls

	WEB_VERSION = "?v=" + String(SOFTWARE_VERSION_MAJOR) + "." + String(SOFTWARE_VERSION_MINOR) + "." + String(SOFTWARE_VERSION_PATCH);

	// Initialize websocket clients first
	init_websocket_clients();

	// Set up WebSocket callbacks BEFORE registering handler (per PsychicHttp example)
	websocket_handler.onOpen([](PsychicWebSocketClient *client) {
		if (!hil_csv_serial_export_active) printf("[socket] connection #%i connected from %s\n", client->socket(), client->remoteIP().toString().c_str());
		if (welcome_websocket_client(client) == true) {
			client->sendMessage("welcome");
		}
		else {
			// Room is full, client not welcome
			if (!hil_csv_serial_export_active) printf("PLAYER WAS DENIED ENTRY (ROOM FULL)\n");
			client->close();
		}
	});

	websocket_handler.onFrame([](PsychicWebSocketRequest *request, httpd_ws_frame *frame) {
		httpd_ws_type_t frame_type = frame->type;

		// If it's text, it might be a command
		if (frame_type == HTTPD_WS_TYPE_TEXT) {
			queue_command((char *)frame->payload, frame->len, get_slot_of_client(request->client()));
		}
		else {
			if (!hil_csv_serial_export_active) printf("UNSUPPORTED WS FRAME TYPE: %d\n", (uint8_t)frame->type);
		}

		return ESP_OK;
	});

	websocket_handler.onClose([](PsychicWebSocketClient *client) {
		if (!hil_csv_serial_export_active) printf("[socket] connection #%i closed from %s\n", client->socket(), client->remoteIP().toString().c_str());
		websocket_client_left(client);
	});

	// Now register WebSocket handler with server (after callbacks are set)
	server.on("/ws", &websocket_handler);

	server.on("/audio", [](PsychicRequest *request, PsychicResponse *response) {
		String filename = "/audio.bin";
		PsychicFileResponse fileResponse(response, LittleFS, filename);

		return fileResponse.send();
	});

	server.on("/mac", HTTP_GET, [](PsychicRequest *request, PsychicResponse *response) {
		response->addHeader("Access-Control-Allow-Origin", "*");
   		return response->send(mac_str);
	});

	server.on("/*", HTTP_GET, [](PsychicRequest *request, PsychicResponse *response) {
		esp_err_t result = ESP_OK;
		String path = "";

		char url[128] = { 0 };
		request->url().toCharArray(url, 128);

		// Remove queries
		fetch_substring(url, '?', 0);

		if(fastcmp(substring, "/") || fastcmp(substring, "/remote") || fastcmp(substring, "/wifi-setup")){
			path += "/index.html";
		}
		else{
			path += substring;
		}

		printf("HTTP GET %s\n", path.c_str());

		// Check if client accepts gzip and .gz file exists
		String acceptEncoding = request->header("Accept-Encoding");
		bool clientAcceptsGzip = acceptEncoding.indexOf("gzip") >= 0;
		String gzPath = path + ".gz";
		File file;
		bool useGzip = false;

		if (clientAcceptsGzip) {
			file = LittleFS.open(gzPath);
			if (file) {
				useGzip = true;
			}
		}

		// Fall back to uncompressed if gzip not available
		if (!file) {
			file = LittleFS.open(path);
		}

		if (file) {
			String etagStr(file.getLastWrite(), 10);

			PsychicFileResponse fileResponse(response, file, useGzip ? gzPath : path);
			fileResponse.addHeader("Cache-Control", "public, max-age=900");
			fileResponse.addHeader("ETag", etagStr.c_str());
			if (useGzip) {
				fileResponse.addHeader("Content-Encoding", "gzip");
			}
			result = fileResponse.send();
			file.close();
		}
		else {
			result = response->send(404);
		}
		return result;
	});

	// Start server AFTER all handlers are registered
	server.start();

	const char *local_hostname = "emotiscope";
	if (!MDNS.begin(local_hostname)) {
		Serial.println("Error starting mDNS");
		return;
	}
	MDNS.addService("http", "tcp", 80);

	web_server_ready = true;
}

void get_mac(){
	// Define a variable to hold the MAC address
	uint8_t mac_address[6]; // MAC address is 6 bytes

	// Retrieve the MAC address of the device
	WiFi.macAddress(mac_address);

	// Format the MAC address into the char array
	snprintf(mac_str, sizeof(mac_str), "%02X:%02X:%02X:%02X:%02X:%02X",
		mac_address[0], mac_address[1], mac_address[2],
		mac_address[3], mac_address[4], mac_address[5]);

	// Print the MAC address string
	printf("MAC Address: %s\n", mac_str);
}

void init_wifi() {
	network_connection_attempts = 0;
	WiFi.begin(wifi_ssid, wifi_pass);
	printf("Started connection attempt to %s...\n", wifi_ssid);
	get_mac();
	esp_wifi_set_ps(WIFI_PS_NONE);
}

void handle_wifi() {
	static int16_t connection_status_last = -1;
	static uint32_t last_reconnect_attempt = 0;
	const uint32_t reconnect_interval_ms = 5000;

	connection_status = WiFi.status();

	// WiFi status has changed
	if (connection_status != connection_status_last) {
		// Emotiscope connected sucessfully to your network
		if (connection_status == WL_CONNECTED) {
			printf("CONNECTED TO %s SUCCESSFULLY @ %s\n", wifi_ssid, WiFi.localIP().toString().c_str());
		}

		// Emotiscope disconnected from a network
		else if (connection_status == WL_DISCONNECTED) {
			printf("DISCONNECTED FROM WIFI!\n");
		}

		// Emotiscope wireless functions are IDLE
		else if (connection_status == WL_IDLE_STATUS) {
			printf("WIFI IN IDLE STATE.\n");
		}

		// Emotiscope failed to connect to your network
		else if (connection_status == WL_CONNECT_FAILED) {
			printf("FAILED TO CONNECT TO %s\n", wifi_ssid);
		}

		// Emotiscope lost connection to your network
		else if (connection_status == WL_CONNECTION_LOST) {
			printf("LOST CONNECTION TO %s\n", wifi_ssid);
		}

		// Emotiscope can't see your network
		else if (connection_status == WL_NO_SSID_AVAIL) {
			printf("UNABLE TO REACH SSID %s\n", wifi_ssid);
		}

		// Anything else
		else {
			printf("WIFI STATUS CHANGED TO UNHANDLED STATE: %i\n", connection_status);
		}

		if (connection_status == WL_CONNECTED && connection_status_last != WL_CONNECTED) {
			printf("NOW CONNECTED TO NETWORK\n");
			print_filesystem();
			if (web_server_ready == false) {
				init_web_server();
			}
		}

		else if (connection_status != WL_CONNECTED && connection_status_last == WL_CONNECTED) {
			printf("LOST CONNECTION TO NETWORK, RETRYING\n");
			WiFi.disconnect();
		}
	}
	else if (connection_status != WL_CONNECTED && millis() - last_reconnect_attempt >= reconnect_interval_ms) {
		printf("ATTEMPTING TO RECONNECT TO THE NETWORK\n");
		last_reconnect_attempt = millis();
		WiFi.reconnect();
		network_connection_attempts++;
	}

	connection_status_last = connection_status;
}
