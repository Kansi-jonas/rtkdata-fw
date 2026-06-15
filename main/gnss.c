#include <stdbool.h>
#include <string.h>
#include <math.h>
#include<ctype.h>
#include <esp_ota_ops.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "gnss.h"
#include "config.h"
#include "util.h"
#include "uart.h"

#define LOG_TAG "GNSS"

/*default command for um980*/
//unlog
#define GNSS_UM980_DISABLE_COM1 "unlog com1\r\n"
#define GNSS_UM980_DISABLE_COM2 "unlog com2\r\n"
#define GNSS_UM980_DISABLE_COM3 "unlog com3\r\n"
#define GNSS_UM980_UNMAKS_ALL   "UNMASK ALL\r\n"
#define GNSS_UM980_CUTANGLE_10d "MASK 10.0\r\n"
#define GNSS_UM980_SAVE         "saveconfig\r\n"
#define GNSS_UM980_LOGLIST      "LOG LOGLISTA ONCE\r\n"
#define GNSS_UM980_VERSION      "VERSIONA\r\n"

/*for ntrip server config*/
#define GNSS_UM980_MODE_BASE "mode,base\r\n"

#define GNSS_UM980_ENABLE_1074 "rtcm1074,com1,1\r\n"
#define GNSS_UM980_ENABLE_1077 "rtcm1077,com1,1\r\n"
#define GNSS_UM980_ENABLE_1084 "rtcm1084,com1,1\r\n"
#define GNSS_UM980_ENABLE_1087 "rtcm1087,com1,1\r\n"
#define GNSS_UM980_ENABLE_1094 "rtcm1094,com1,1\r\n"
#define GNSS_UM980_ENABLE_1097 "rtcm1097,com1,1\r\n"
#define GNSS_UM980_ENABLE_1114 "rtcm1114,com1,1\r\n"
#define GNSS_UM980_ENABLE_1117 "rtcm1117,com1,1\r\n"
#define GNSS_UM980_ENABLE_1124 "rtcm1124,com1,1\r\n"
#define GNSS_UM980_ENABLE_1127 "rtcm1127,com1,1\r\n"
#define GNSS_UM980_ENABLE_1137 "rtcm1137,com1,1\r\n"
#define GNSS_UM980_ENABLE_1005 "rtcm1005,com1,10\r\n"
#define GNSS_UM980_ENABLE_1019 "rtcm1019,com1,30\r\n"
#define GNSS_UM980_ENABLE_1020 "rtcm1020,com1,30\r\n"
#define GNSS_UM980_ENABLE_1042 "rtcm1042,com1,30\r\n"
#define GNSS_UM980_ENABLE_1046 "rtcm1046,com1,30\r\n"

// #define GNSS_UM980_ENABLE_4076 "rtcm4076,com1,onchanged\r\n"
#define GNSS_UM980_RTCM1033      "log com1 rtcm1033 ontime 30\r\n"
#define GNSS_UM980_ENABLE_B2a    "CONFIG RTCMB1CB2a ENABLE\r\n"
#define GNSS_UM980_CMD_BASE_AUTO "mode base 1008 time 60  10\r\n"
#define GNSS_UM980_MMP           "CONFIG MMP ENABLE\r\n"
#define GNSS_UM980_ENABLE_G2     "CONFIG SIGNALGROUP 2\r\n"
#define GNSS_UM980_ANTIJAM       "CONFIG ANTIJAM FORCE\r\n"
#define GNSS_UM980_PPP_E6        "CONFIG PPP ENABLE E6-HAS\r\n"
#define GNSS_UM980_PPP_DATUM     "CONFIG PPP DATUM WGS84\r\n"

/*for ntrip client config*/
#define GNSS_UM980_ROVER        "mode rover\r\n"
#define GNSS_UM980_RTKTIMEOUT   "RTKTIMEOUT 20\r\n"
#define GNSS_UM980_DGPSTIMEOUT  "DGPSTIMEOUT 60\r\n"

#define GNSS_UM980_GPGGA "log com1 gpgga ontime 1\r\n" // 0.2->1
// #define GNSS_UM980_GNGGA "log com2 gngga ontime 0.2\r\n"
// #define GNSS_UM980_GPRMC "log com3 gprmc ontime 0.2\r\n"
#define GNSS_UM980_GPGST "log com1 gpgst ontime 1\r\n" // 
#define GNSS_UM980_GPGSA "log com1 gpgsa ontime 5\r\n"
#define GNSS_UM980_GPGSV "log com1 gpgsv ontime 10\r\n"
#define GNSS_UM980_GPVTG "log com1 gpvtg ontime 1\r\n" //1
#define GNSS_UM980_GPRMC "log com1 gprmc ontime 1\r\n" //1
#define GNSS_UM980_GPGLL "log com1 gpgll ontime 1\r\n" //1
// #define GNSS_UM980_GPSGGA "log com2 gpsgga ontime 1\r\n"
// #define GNSS_UM980_BDSGGA "log com2 bdsgga ontime 1\r\n"
// #define GNSS_UM980_GLOGGA "log com2 glogga ontime 1\r\n"
// #define GNSS_UM980_GALGGA "log com2 galgga ontime 1\r\n"

#define GNSS_UM980_GPGGA_DISABLE "unlog com1 gpgga\r\n"
#define GNSS_UM980_GNGGA_DISABLE "unlog com1 gngga\r\n"
#define GNSS_UM980_GPGSA_DISABLE "unlog com1 gpgsa\r\n"
#define GNSS_UM980_GPGSV_DISABLE "unlog com1 gpgsv\r\n"
#define GNSS_UM980_GPRMC_DISABLE "unlog com1 gprmc\r\n"
#define GNSS_UM980_GPVTG_DISABLE "unlog com1 gpvtg\r\n"
#define GNSS_UM980_GPGLL_DISABLE "unlog com1 gpgll\r\n"
#define GNSS_UM980_GPZDA_DISABLE "unlog com1 gpzda\r\n"
#define GNSS_UM980_GPGST_DISABLE "unlog com1 gpgst\r\n"

#define GNSS_UM980_GPGGA_CONFIG "log com1 gpgga ontime %lf\r\n"
#define GNSS_UM980_GNGGA_CONFIG "log com1 gngga ontime %lf\r\n"
#define GNSS_UM980_GPRMC_CONFIG "log com1 gprmc ontime %lf\r\n"
#define GNSS_UM980_GPGST_CONFIG "log com1 gpgst ontime %lf\r\n"
#define GNSS_UM980_GPGSV_CONFIG "log com1 gpgsv ontime %lf\r\n"
#define GNSS_UM980_GPVTG_CONFIG "log com1 gpvtg ontime %lf\r\n"
#define GNSS_UM980_GPGLL_CONFIG "log com1 gpgll ontime %lf\r\n"
#define GNSS_UM980_GPZDA_CONFIG "log com1 gpzda ontime %lf\r\n"
#define GNSS_UM980_GPGSA_CONFIG "log com1 gpgsa ontime %lf\r\n"

#define GPIO_GNSS_RESET  GPIO_NUM_22

#define GNSS_CMD_DELAY_MS 500

void config_gnss_base()
{
    ESP_LOGE(LOG_TAG, "config for base");
    set_gnss_cmd_mode(true);
    
    drv_uart_gnss_send((uint8_t *)GNSS_UM980_DISABLE_COM1, strlen(GNSS_UM980_DISABLE_COM1));
    vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));

    drv_uart_gnss_send((uint8_t *)GNSS_UM980_DISABLE_COM2, strlen(GNSS_UM980_DISABLE_COM2));
    vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));

    drv_uart_gnss_send((uint8_t *)GNSS_UM980_VERSION, strlen(GNSS_UM980_VERSION));
    vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));

    //drv_uart_gnss_send((uint8_t *)GNSS_UM980_ENABLE_1074, strlen(GNSS_UM980_ENABLE_1074));
    //vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS)); 

    drv_uart_gnss_send((uint8_t *)GNSS_UM980_ENABLE_1077, strlen(GNSS_UM980_ENABLE_1077));
    vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));   

    //drv_uart_gnss_send((uint8_t *)GNSS_UM980_ENABLE_1084, strlen(GNSS_UM980_ENABLE_1084));
    //vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));

    drv_uart_gnss_send((uint8_t *)GNSS_UM980_ENABLE_1087, strlen(GNSS_UM980_ENABLE_1087));
    vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));  

    //drv_uart_gnss_send((uint8_t *)GNSS_UM980_ENABLE_1094, strlen(GNSS_UM980_ENABLE_1094));
    //vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));

    drv_uart_gnss_send((uint8_t *)GNSS_UM980_ENABLE_1097, strlen(GNSS_UM980_ENABLE_1097));
    vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS)); 

    //drv_uart_gnss_send((uint8_t *)GNSS_UM980_ENABLE_1114, strlen(GNSS_UM980_ENABLE_1114));
    //vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));

     drv_uart_gnss_send((uint8_t *)GNSS_UM980_ENABLE_1117, strlen(GNSS_UM980_ENABLE_1117));
    vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));

    //drv_uart_gnss_send((uint8_t *)GNSS_UM980_ENABLE_1124, strlen(GNSS_UM980_ENABLE_1124));
    //vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));

    drv_uart_gnss_send((uint8_t *)GNSS_UM980_ENABLE_1127, strlen(GNSS_UM980_ENABLE_1127));
    vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));    

    drv_uart_gnss_send((uint8_t *)GNSS_UM980_ENABLE_1137, strlen(GNSS_UM980_ENABLE_1137));
    vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));  

    //drv_uart_gnss_send((uint8_t *)GNSS_UM980_ENABLE_1005, strlen(GNSS_UM980_ENABLE_1005));
    //vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));

    //drv_uart_gnss_send((uint8_t *)GNSS_UM980_ENABLE_1019, strlen(GNSS_UM980_ENABLE_1019));
    //vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));

    //drv_uart_gnss_send((uint8_t *)GNSS_UM980_ENABLE_1020, strlen(GNSS_UM980_ENABLE_1020));
    //vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));

    //drv_uart_gnss_send((uint8_t *)GNSS_UM980_ENABLE_1042, strlen(GNSS_UM980_ENABLE_1042));
    //vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));

    //drv_uart_gnss_send((uint8_t *)GNSS_UM980_ENABLE_1046, strlen(GNSS_UM980_ENABLE_1046));
    //vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));

    drv_uart_gnss_send((uint8_t *)GNSS_UM980_RTCM1033, strlen(GNSS_UM980_RTCM1033));
    vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));

    drv_uart_gnss_send((uint8_t *)GNSS_UM980_CUTANGLE_10d,strlen(GNSS_UM980_CUTANGLE_10d));
    vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));

    drv_uart_gnss_send((uint8_t *)GNSS_UM980_ENABLE_B2a, strlen(GNSS_UM980_ENABLE_B2a));
    vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));

    drv_uart_gnss_send((uint8_t *)GNSS_UM980_ENABLE_G2, strlen(GNSS_UM980_ENABLE_G2));
    vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));

    drv_uart_gnss_send((uint8_t *)GNSS_UM980_PPP_E6, strlen(GNSS_UM980_PPP_E6));
    vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));

    drv_uart_gnss_send((uint8_t *)GNSS_UM980_PPP_DATUM, strlen(GNSS_UM980_PPP_DATUM));
    vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));

    drv_uart_gnss_send((uint8_t *)GNSS_UM980_CMD_BASE_AUTO, strlen(GNSS_UM980_CMD_BASE_AUTO));      
    vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));

    drv_uart_gnss_send((uint8_t *)GNSS_UM980_SAVE, strlen(GNSS_UM980_SAVE));
    vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));

    drv_uart_gnss_send((uint8_t *)GNSS_UM980_LOGLIST, strlen(GNSS_UM980_LOGLIST));
    vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));

    set_gnss_cmd_mode(false);    
}

void gnss_reset_low()
{
    gpio_set_direction(GPIO_GNSS_RESET, GPIO_MODE_OUTPUT);
    gpio_pullup_en(GPIO_GNSS_RESET);
    gpio_set_level(GPIO_GNSS_RESET, 0); 
}

void gnss_reset_high()
{
    gpio_set_direction(GPIO_GNSS_RESET, GPIO_MODE_OUTPUT);
    gpio_pullup_en(GPIO_GNSS_RESET);
    gpio_set_level(GPIO_GNSS_RESET, 1); 
}

void gnss_clear_config()
{
    ESP_LOGI(LOG_TAG, "clearing GNSS UM980 config");
    set_gnss_cmd_mode(true);

    // COM-Ports leeren
    drv_uart_gnss_send((uint8_t *)GNSS_UM980_DISABLE_COM1, strlen(GNSS_UM980_DISABLE_COM1));
    vTaskDelay(pdMS_TO_TICKS(150));
    drv_uart_gnss_send((uint8_t *)GNSS_UM980_DISABLE_COM2, strlen(GNSS_UM980_DISABLE_COM2));
    vTaskDelay(pdMS_TO_TICKS(150));
    drv_uart_gnss_send((uint8_t *)GNSS_UM980_DISABLE_COM3, strlen(GNSS_UM980_DISABLE_COM3));
    vTaskDelay(pdMS_TO_TICKS(150));

    // Einzelne NMEA Logs deaktivieren
    drv_uart_gnss_send((uint8_t *)GNSS_UM980_GPGGA_DISABLE, strlen(GNSS_UM980_GPGGA_DISABLE));
    vTaskDelay(pdMS_TO_TICKS(50));
    drv_uart_gnss_send((uint8_t *)GNSS_UM980_GNGGA_DISABLE, strlen(GNSS_UM980_GNGGA_DISABLE));
    vTaskDelay(pdMS_TO_TICKS(50));
    drv_uart_gnss_send((uint8_t *)GNSS_UM980_GPGSA_DISABLE, strlen(GNSS_UM980_GPGSA_DISABLE));
    vTaskDelay(pdMS_TO_TICKS(50));
    drv_uart_gnss_send((uint8_t *)GNSS_UM980_GPGSV_DISABLE, strlen(GNSS_UM980_GPGSV_DISABLE));
    vTaskDelay(pdMS_TO_TICKS(50));
    drv_uart_gnss_send((uint8_t *)GNSS_UM980_GPRMC_DISABLE, strlen(GNSS_UM980_GPRMC_DISABLE));
    vTaskDelay(pdMS_TO_TICKS(50));
    drv_uart_gnss_send((uint8_t *)GNSS_UM980_GPVTG_DISABLE, strlen(GNSS_UM980_GPVTG_DISABLE));
    vTaskDelay(pdMS_TO_TICKS(50));
    drv_uart_gnss_send((uint8_t *)GNSS_UM980_GPGLL_DISABLE, strlen(GNSS_UM980_GPGLL_DISABLE));
    vTaskDelay(pdMS_TO_TICKS(50));
    drv_uart_gnss_send((uint8_t *)GNSS_UM980_GPZDA_DISABLE, strlen(GNSS_UM980_GPZDA_DISABLE));
    vTaskDelay(pdMS_TO_TICKS(50));
    drv_uart_gnss_send((uint8_t *)GNSS_UM980_GPGST_DISABLE, strlen(GNSS_UM980_GPGST_DISABLE));
    vTaskDelay(pdMS_TO_TICKS(50));

    // Alle Masken aufheben
    drv_uart_gnss_send((uint8_t *)GNSS_UM980_UNMAKS_ALL, strlen(GNSS_UM980_UNMAKS_ALL));
    vTaskDelay(pdMS_TO_TICKS(150));

    // Konfiguration abspeichern
    drv_uart_gnss_send((uint8_t *)GNSS_UM980_SAVE, strlen(GNSS_UM980_SAVE));
    vTaskDelay(pdMS_TO_TICKS(300));

    set_gnss_cmd_mode(false);
}


void gnss_init()
{
    gnss_reset_low();
    vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));

    gnss_reset_high();
    vTaskDelay(pdMS_TO_TICKS(GNSS_CMD_DELAY_MS));

    // UM980 cleanup, before set config
    gnss_clear_config();

    config_gnss_base();

}