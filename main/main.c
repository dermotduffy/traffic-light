#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/periph_ctrl.h"
#include "lwip/sockets.h"
#include "nvs_flash.h"

// Define these, or run make as ...
//   EXTRA_CPPFLAGS='-DWIFI_SSID=\"SSID\" -DWIFI_PASSWORD=\"PASSWORD\"' make
//
//#define WIFI_SSID
//#define WIFI_PASSWORD

#define SERVER_THREAD_NAME       "server"
#define SERVER_THREAD_STACK_WORDS 2<<13
#define SERVER_THREAD_PRORITY     8
#define SERVER_RECV_BUF_LEN       1024
#define SERVER_TCP_PORT           7000

#define STATUS_THREAD_NAME       "status"
#define STATUS_THREAD_STACK_WORDS 10240
#define STATUS_THREAD_PRORITY     6
#define STATUS_BLINKS_ON_CONNECT  3

#define DELAY_STARTUP_LED_ON_MS   3000
#define DELAY_BLINK_MS            1000
#define DELAY_SERVER_ERROR_MS     1000

#define GPIO_GREEN                14
#define GPIO_ORANGE               27
#define GPIO_RED                  25

const static char *LOG_TAG = "TrafficLightServer";

static EventGroupHandle_t wifi_event_group;
const static int EVENT_CONNECTED_BIT = BIT0;  // Connected to wifi?

xTaskHandle server_handle;
xTaskHandle status_handle;

typedef struct {
  char green, orange, red;
} Colour;

Colour colour_all_off = {0,       0,    0};
Colour colour_all_on  = {0xff, 0xff, 0xff};  
Colour colour_green   = {0xff,    0,    0};
Colour colour_red     = {   0,    0, 0xff};

void pwm_init(void) {
  periph_module_enable(PERIPH_LEDC_MODULE);

  ledc_timer_config_t timer_conf = {
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .bit_num = 8, // Does not exist: LEDC_TIMER_8_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 1000,
  };
  ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

  ledc_channel_config_t default_ch_conf = {
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,
  };

  ledc_channel_config_t green_ch_conf = default_ch_conf; 
  green_ch_conf.gpio_num = GPIO_GREEN;
  green_ch_conf.channel = LEDC_CHANNEL_0;
  ESP_ERROR_CHECK(ledc_channel_config(&green_ch_conf));

  ledc_channel_config_t orange_ch_conf = default_ch_conf; 
  orange_ch_conf.gpio_num = GPIO_ORANGE;
  orange_ch_conf.channel = LEDC_CHANNEL_1;
  ESP_ERROR_CHECK(ledc_channel_config(&orange_ch_conf));

  ledc_channel_config_t red_ch_conf = default_ch_conf;
  red_ch_conf.gpio_num = GPIO_RED;
  red_ch_conf.channel = LEDC_CHANNEL_2;
  ESP_ERROR_CHECK(ledc_channel_config(&red_ch_conf));
}

void set_colours_raw(char green, char orange, char red) {
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, green);
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, orange);
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, red);

  ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
}  

void set_colours(Colour colour) {
  set_colours_raw(colour.green, colour.orange, colour.red);
}

void DelayTask(int ms) {
  vTaskDelay(ms / portTICK_PERIOD_MS);
}

static void status_task(void *p) {
  // Counter of number of connection-established blinks remaining.
  //  -1: Not connected.
  //   0: Already blinked connection success.
  // 1-3: Number of blinks remaining.
  int connected_blinks_remaining = -1;

  while (true) {
    // Is wifi connected?
    if (xEventGroupGetBits(wifi_event_group) & EVENT_CONNECTED_BIT) {
      // Already blinked that we are connected?
      if (connected_blinks_remaining == 0) {
        DelayTask(DELAY_BLINK_MS);
      } else {
        // Need to do blinks to show connection. If the connection
        // has just been established, set the blink counter to the
        // desired number of blinks.
        if (connected_blinks_remaining < 0) {
          connected_blinks_remaining = STATUS_BLINKS_ON_CONNECT;
        }
        set_colours(colour_green);
        DelayTask(DELAY_BLINK_MS);
        set_colours(colour_all_off);
        DelayTask(DELAY_BLINK_MS);
      
        --connected_blinks_remaining;
      }
    } else {
      set_colours(colour_red);
      DelayTask(DELAY_BLINK_MS);
      set_colours(colour_all_off);
      DelayTask(DELAY_BLINK_MS);

      // Reset blink counter so that blinks occur
      // on connection (re-)establishment.
      connected_blinks_remaining = -1;
    }
  }

  vTaskDelete(NULL);
  return;
}

static void server_task(void *p) {
  int listen_socket, client_socket;
  struct sockaddr_in serv_sock_addr, client_sock_addr;
  socklen_t client_sock_addr_len = sizeof(client_sock_addr);
  char recv_buf[SERVER_RECV_BUF_LEN];

  while (true) {
    ESP_LOGI(LOG_TAG, "Create socket ...");
    listen_socket = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_socket < 0) {
      ESP_LOGW(LOG_TAG, "socket(...) failed");
      DelayTask(DELAY_SERVER_ERROR_MS);
      continue;
    }

    int reuse = 1;
    if (setsockopt(listen_socket,
                   SOL_SOCKET,
                   SO_REUSEADDR,
                   &reuse, sizeof(reuse)) < 0) {
      DelayTask(DELAY_SERVER_ERROR_MS);
      continue;
    }

    ESP_LOGI(LOG_TAG, "Bind ...");
    memset(&serv_sock_addr, 0, sizeof(serv_sock_addr));
    serv_sock_addr.sin_family = AF_INET;
    serv_sock_addr.sin_addr.s_addr = 0;
    serv_sock_addr.sin_port = htons(SERVER_TCP_PORT);
  
    if (bind(listen_socket, (struct sockaddr*)&serv_sock_addr, sizeof(serv_sock_addr))) {
      ESP_LOGW(LOG_TAG, "bind(...) failed");
      close(listen_socket);

      DelayTask(DELAY_SERVER_ERROR_MS);
      continue;
    }

    ESP_LOGI(LOG_TAG, "Listen ...");
    if (listen(listen_socket, 32)) {
      ESP_LOGW(LOG_TAG, "listen(...) failed");
      close(listen_socket); 

      DelayTask(DELAY_SERVER_ERROR_MS);
      continue;
    }

    memset(recv_buf, 0, SERVER_RECV_BUF_LEN);
    while (true) {
      client_socket = accept(listen_socket, (struct sockaddr*)&client_sock_addr, &client_sock_addr_len);
      if (client_socket < 0) {
        ESP_LOGW(LOG_TAG, "accept(...) failed");
        DelayTask(DELAY_SERVER_ERROR_MS);
        break;
      }

      char* ip_addr_string = inet_ntoa(client_sock_addr.sin_addr);
      ESP_LOGI(LOG_TAG, "Established connection from %s", ip_addr_string)

      char bytes = recv(client_socket, recv_buf, SERVER_RECV_BUF_LEN - 1, 0);
      if (bytes == 3) {
        ESP_LOGI(LOG_TAG, "Received (%x, %x, %x)", recv_buf[0], recv_buf[1], recv_buf[2])
        set_colours_raw(recv_buf[0], recv_buf[1], recv_buf[2]);
      } 
      close(client_socket);
    }

    close(listen_socket);
  }

  // Never reached.
  vTaskDelete(NULL);
  return;
}

static void server_task_create(void) {
  if (server_handle) {
    ESP_LOGI(LOG_TAG, "Server already created");
    return;
  }
  ESP_LOGI(LOG_TAG, "Server created");

  configASSERT(xTaskCreate(server_task,
                           SERVER_THREAD_NAME,
                           SERVER_THREAD_STACK_WORDS,
                           NULL,
                           SERVER_THREAD_PRORITY,
                           &server_handle) == pdTRUE);
}

static void status_task_create(void) {
  configASSERT(xTaskCreate(status_task,
                           STATUS_THREAD_NAME,
                           STATUS_THREAD_STACK_WORDS,
                           NULL,
                           STATUS_THREAD_PRORITY,
                           &status_handle) == pdTRUE);
}

static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
  switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
      ESP_LOGI(LOG_TAG, "WIFI event: Start ...");
      esp_wifi_connect();
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      ESP_LOGI(LOG_TAG, "WIFI event: Connected, got IP...");
      xEventGroupSetBits(wifi_event_group, EVENT_CONNECTED_BIT);
      server_task_create();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      ESP_LOGI(LOG_TAG, "WIFI event: Disconnect...");
      xEventGroupClearBits(wifi_event_group, EVENT_CONNECTED_BIT);

      /* This is a workaround as ESP32 WiFi libs don't currently
         auto-reassociate. */
      esp_wifi_connect(); 
      break;
    default:
      break;
  }
  return ESP_OK;
}

void app_main(void) {
  // Boot.
  nvs_flash_init();
  tcpip_adapter_init();
  wifi_event_group = xEventGroupCreate();
  ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));

  // Light all LEDs and pause at startup (test LEDs).
  pwm_init();
  set_colours(colour_all_on);
  DelayTask(DELAY_STARTUP_LED_ON_MS);
  set_colours(colour_all_off);

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

  wifi_config_t wifi_config = {
    .sta = {
      .ssid = WIFI_SSID,
      .password = WIFI_PASSWORD,
      .bssid_set = false,
    },
  };

  ESP_LOGI(LOG_TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  status_task_create();
}
