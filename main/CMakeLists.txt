idf_component_register(SRCS "main.c"
		"dns_server.c"
		"config.c"
		"core_dump.c"
		"log.c"
		"interface/ntrip_util.c"
		"retry.c"
		"status_led.c"
		"stream_stats.c"
		"uart.c"
		"util.c"
		"web_server.c"
		"wifi.c"
		"gnss.c"
        "supervisor.c"
        "provisioning.c"
		"interface/ntrip_server.c"
		"interface/socket_client.c"
		"interface/socket_server.c"
		"protocol/nmea.c"
		"update.c"
		"state.c"
        INCLUDE_DIRS "include")

spiffs_create_partition_image(www ../www FLASH_IN_PROJECT)

# NOTE: there is intentionally NO enroll-secret compile definition. Enrollment
# auth uses a PER-DEVICE key written into NVS at flash time (KEY_RTK_ENROLL_KEY),
# derived from a host/server-only master. No fleet-wide secret is ever baked into
# the binary (the .bin is public via OTA, so a baked secret would leak the fleet).
# See docs/enroll-per-device-key.md.
