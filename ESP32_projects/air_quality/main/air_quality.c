/* Air Quality Project
 */

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_smartconfig.h"
#include "esp_spi_flash.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_wpa2.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lwip/err.h"
#include "lwip/sys.h"

#include "air_quality_i2c.h"

#define WIFI_MODE_CUSTOMER_DEFAULT 0
#define WIFI_MODE_CUSTOMER_SMARTCONFIG 1
#define WIFI_MODE_CUSTOMER_STA 2

#define ESP_WIFI_SSID "season_503"
#define ESP_WIFI_PASS "helloworld23503"

/* FreeRTOS event group to signal when we are connected & ready to make a
 * request */
static EventGroupHandle_t s_wifi_event_group;
/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */
static const int CONNECTED_BIT = BIT0;
static const int ESPTOUCH_DONE_BIT = BIT1;
static const int WIFI_FAIL_BIT = BIT1;
static const char *TAG = "air_quality";
static const int MAX_TIMEOUT_MS = 10000;
static const int EXAMPLE_ESP_MAXIMUM_RETRY = 20;

static bool wifi_connected_flag = false;
static int wifi_current_mode = WIFI_MODE_CUSTOMER_DEFAULT;
static int s_retry_num = 0;

static void smartconfig_task(void *parm) {
  EventBits_t uxBits;
  // TickType_t max_wait = pdMS_TO_TICKS(MAX_TIMEOUT_MS * 10);
  TickType_t max_wait = MAX_TIMEOUT_MS / portTICK_PERIOD_MS;
  ESP_ERROR_CHECK(esp_smartconfig_set_type(SC_TYPE_ESPTOUCH));
  smartconfig_start_config_t cfg = SMARTCONFIG_START_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_smartconfig_start(&cfg));
  while (1) {
    uxBits = xEventGroupWaitBits(s_wifi_event_group,
                                 CONNECTED_BIT | ESPTOUCH_DONE_BIT, true, false,
                                 max_wait);
    if (uxBits & CONNECTED_BIT) {
      ESP_LOGI(TAG, "WiFi Connected to ap");
      wifi_connected_flag = true;
    }
    if (uxBits & ESPTOUCH_DONE_BIT) {
      ESP_LOGI(TAG, "smartconfig over");
      esp_smartconfig_stop();
      vTaskDelete(NULL);
    }

    if (wifi_connected_flag == false) {
      ESP_LOGI(TAG, "smartconfig over with disconnection");
      esp_smartconfig_stop();
      vTaskDelete(NULL);
    }
  }
}

static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data) {
  ESP_LOGI(TAG, "%s line:%d, event_id:%d", __func__, __LINE__, event_id);
  if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
    ESP_LOGI(TAG, "%s line:%d, wifi event sta start", __func__, __LINE__);
    if (wifi_current_mode == WIFI_MODE_CUSTOMER_SMARTCONFIG) {
      xTaskCreate(smartconfig_task, "smartconfig_task", 4096, NULL, 3, NULL);
    } else if (wifi_current_mode == WIFI_MODE_CUSTOMER_STA) {
      esp_wifi_connect();
    }
  } else if (event_base == WIFI_EVENT &&
             event_id == WIFI_EVENT_STA_DISCONNECTED) {
    ESP_LOGI(TAG, "%s line:%d, wifi sta disconnected", __func__, __LINE__);
    if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
      esp_wifi_connect();
      s_retry_num++;
      ESP_LOGI(TAG, "retry to connect to the AP");
    } else {
      xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
    }
    ESP_LOGI(TAG, "connect to the AP fail");
    esp_wifi_connect();
    xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
  } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
    s_retry_num = 0;
    xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
    wifi_connected_flag = true;
  } else if (event_base == SC_EVENT && event_id == SC_EVENT_SCAN_DONE) {
    ESP_LOGI(TAG, "Scan done");
  } else if (event_base == SC_EVENT && event_id == SC_EVENT_FOUND_CHANNEL) {
    ESP_LOGI(TAG, "Found channel");
  } else if (event_base == SC_EVENT && event_id == SC_EVENT_GOT_SSID_PSWD) {
    ESP_LOGI(TAG, "Got SSID and password -->");

    smartconfig_event_got_ssid_pswd_t *evt =
        (smartconfig_event_got_ssid_pswd_t *)event_data;
    wifi_config_t wifi_config;
    uint8_t ssid[33] = {0};
    uint8_t password[65] = {0};
    uint8_t rvd_data[33] = {0};

    bzero(&wifi_config, sizeof(wifi_config_t));
    memcpy(wifi_config.sta.ssid, evt->ssid, sizeof(wifi_config.sta.ssid));
    memcpy(wifi_config.sta.password, evt->password,
           sizeof(wifi_config.sta.password));
    wifi_config.sta.bssid_set = evt->bssid_set;
    if (wifi_config.sta.bssid_set == true) {
      memcpy(wifi_config.sta.bssid, evt->bssid, sizeof(wifi_config.sta.bssid));
    }

    memcpy(ssid, evt->ssid, sizeof(evt->ssid));
    memcpy(password, evt->password, sizeof(evt->password));
    ESP_LOGI(TAG, "SSID:%s", ssid);
    ESP_LOGI(TAG, "PASSWORD:%s", password);
    if (evt->type == SC_TYPE_ESPTOUCH_V2) {
      ESP_ERROR_CHECK(esp_smartconfig_get_rvd_data(rvd_data, sizeof(rvd_data)));
      ESP_LOGI(TAG, "RVD_DATA:");
      for (int i = 0; i < 33; i++) {
        printf("%02x ", rvd_data[i]);
      }
      printf("\n");
    }

    ESP_ERROR_CHECK(esp_wifi_disconnect());
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    esp_wifi_connect();
  } else if (event_base == SC_EVENT && event_id == SC_EVENT_SEND_ACK_DONE) {
    xEventGroupSetBits(s_wifi_event_group, ESPTOUCH_DONE_BIT);
  }
}

static void initialise_wifi(void) {
  ESP_ERROR_CHECK(esp_netif_init());
  s_wifi_event_group = xEventGroupCreate();
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
  assert(sta_netif);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                             &event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                             &event_handler, NULL));
  ESP_ERROR_CHECK(esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID,
                                             &event_handler, NULL));

  wifi_current_mode = WIFI_MODE_CUSTOMER_SMARTCONFIG;
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_start());
}

void wifi_init_sta(void) {
  ESP_LOGI(TAG, "wifi_init_sta -->");

  wifi_config_t wifi_config = {
      .sta =
          {
              .ssid = ESP_WIFI_SSID,
              .password = ESP_WIFI_PASS,
              /* Setting a password implies station will connect to all security
               * modes including WEP/WPA. However these modes are deprecated and
               * not advisable to be used. Incase your Access point doesn't
               * support WPA2, these mode can be enabled by commenting below
               * line */
              .threshold.authmode = WIFI_AUTH_WPA2_PSK,

              .pmf_cfg = {.capable = true, .required = false},
          },
  };
  wifi_current_mode = WIFI_MODE_CUSTOMER_STA;
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "wifi_init_sta finished.");

  /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or
   * connection failed for the maximum number of re-tries (WIFI_FAIL_BIT). The
   * bits are set by event_handler() (see above) */
  TickType_t max_wait = pdMS_TO_TICKS(MAX_TIMEOUT_MS * 10);

  EventBits_t bits =
      xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT | WIFI_FAIL_BIT,
                          pdFALSE, pdFALSE, max_wait);

  /* xEventGroupWaitBits() returns the bits before the call returned, hence we
   * can test which event actually happened. */
  if (bits & CONNECTED_BIT) {
    ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", ESP_WIFI_SSID,
             ESP_WIFI_PASS);
    wifi_connected_flag = true;
  } else if (bits & WIFI_FAIL_BIT) {
    ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", ESP_WIFI_SSID,
             ESP_WIFI_PASS);
    wifi_connected_flag = false;
  } else {
    ESP_LOGE(TAG, "UNEXPECTED EVENT");
  }
}

static void release_wifi(void) { vEventGroupDelete(s_wifi_event_group); }

void app_main(void) {
  printf("Air Quality -->\n");

  /* Print chip information */
  esp_chip_info_t chip_info;
  esp_chip_info(&chip_info);
  printf("This is %s chip with %d CPU core(s), WiFi%s%s, ", CONFIG_IDF_TARGET,
         chip_info.cores, (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
         (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

  printf("silicon revision %d, ", chip_info.revision);

  printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
         (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded"
                                                       : "external");
  printf("Minimum free heap size: %d bytes\n",
         esp_get_minimum_free_heap_size());

  ESP_ERROR_CHECK(nvs_flash_init());

  /* about wifi*/
  initialise_wifi();

  /* wait smartconfig finish*/
  int max_cnt = MAX_TIMEOUT_MS / 1000;
  for (int i = max_cnt; i >= 0; i--) {
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 1 second
    if (wifi_connected_flag) {
      break;
    }
  }

  if (wifi_connected_flag == false) {
    ESP_ERROR_CHECK(esp_wifi_stop());

    wifi_init_sta();
  }
  release_wifi();

  /* air quality */
  start_tvoc();
  return;
}
