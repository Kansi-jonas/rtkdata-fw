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
		"eth.c"
		"board_test.c"
		"gnss.c"
		"gnss_site.c"
		"gnss_site_runtime.c"
        "supervisor.c"
        "provisioning.c"
		"interface/ntrip_server.c"
		"interface/ntrip_tx_core.c"
		"interface/ntrip_conn.c"
		"telemetry_json.c"
		"gnss_ack.c"
		"gnss_capture.c"
		"interface/socket_client.c"
		"interface/socket_server.c"
		"protocol/nmea.c"
		"update.c"
		"state.c"
        INCLUDE_DIRS "include")

# The board test build (-DRTK_TEST_PAGE=1) gets ../test-page/test.html on top
# of the normal pages; a customer image never carries the file.
if(CONFIG_RTK_TEST_PAGE)
    set(RTK_WWW_DIR "${CMAKE_BINARY_DIR}/www_test")
    file(REMOVE_RECURSE "${RTK_WWW_DIR}")
    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../www/" DESTINATION "${RTK_WWW_DIR}")
    file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/../test-page/test.html" DESTINATION "${RTK_WWW_DIR}")
    # The copy happens at configure time: any edited page must re-run it.
    file(GLOB_RECURSE RTK_WWW_SOURCES CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/../www/*")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
                 ${RTK_WWW_SOURCES} "${CMAKE_CURRENT_SOURCE_DIR}/../test-page/test.html")
    spiffs_create_partition_image(www "${RTK_WWW_DIR}" FLASH_IN_PROJECT)
else()
    spiffs_create_partition_image(www ../www FLASH_IN_PROJECT)
endif()

# NOTE: there is intentionally NO enroll-secret compile definition. Enrollment
# auth uses a PER-DEVICE key written into NVS at flash time (KEY_RTK_ENROLL_KEY),
# derived from a host/server-only master. No fleet-wide secret is ever baked into
# the binary (the .bin is public via OTA, so a baked secret would leak the fleet).
# See docs/enroll-per-device-key.md.
