idf_component_register(SRCS "main.c"
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

# Fleet provisioning secret, injected from the build environment (CI passes it
# from a GitHub secret; locally via `-e RTK_ENROLL_SECRET=...`). When unset the
# build falls back to the placeholder in provisioning.c so it still compiles.
# Never hardcode the secret in tracked source.
if(DEFINED ENV{RTK_ENROLL_SECRET})
    target_compile_definitions(${COMPONENT_LIB} PRIVATE RTK_ENROLL_SECRET="$ENV{RTK_ENROLL_SECRET}")
endif()
