#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include "math.h"

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

#define SERVER_TASK_NAME             "server"
#define SERVER_TASK_STACK_WORDS      2<<13
#define SERVER_TASK_PRORITY          6
#define SERVER_RECV_BUF_LEN          1024
#define SERVER_TCP_PORT              7000

#define STATUS_TASK_NAME             "status"
#define STATUS_TASK_STACK_WORDS      2<<11
#define STATUS_TASK_PRORITY          4
#define STATUS_BLINKS_ON_CONNECT     3

#define PULSATE_TASK_NAME            "pulsate"
#define PULSATE_TASK_STACK_WORDS     2<<11
#define PULSATE_TASK_PRORITY         8

#define PWM_FREQUENCY_HZ             1000
// Total desired fade length (in ticks @ pwm_frequency)
#define FADE_DURATION_TOTAL_TICKS    2000

#define DELAY_STARTUP_LED_ON_MS      3000
#define DELAY_BLINK_MS               1000
#define DELAY_SERVER_ERROR_MS        1000
#define DELAY_WAIT_FOR_FADE_EVENT_MS 10000
#define DELAY_WAIT_FOR_SEMAPHORE     10000

#define GPIO_GREEN                   14
#define GPIO_ORANGE                  27
#define GPIO_RED                     25

const static char *LOG_TAG = "TrafficLightServer";

static EventGroupHandle_t wifi_event_group;
// Connected to wifi?
const static int WIFI_EVENT_CONNECTED_BIT = BIT0;

static EventGroupHandle_t pulsate_event_group;

// Request to start a pulsation.
const static int PULSATE_EVENT_START_BIT = BIT0;
// Request to stop a pulsation.
const static int PULSATE_EVENT_STOP_BIT = BIT1;
// Reached end of a fade within the pulsation (green).
const static int PULSATE_EVENT_FADE_END_GREEN_BIT = BIT2;
// Reached end of a fade within the pulsation (orange).
const static int PULSATE_EVENT_FADE_END_ORANGE_BIT = BIT3;
// Reached end of a fade within the pulsate (red).
const static int PULSATE_EVENT_FADE_END_RED_BIT = BIT4;

static xTaskHandle server_handle;
static xTaskHandle status_handle;
static xTaskHandle pulsate_handle;

typedef struct {
  char green, orange, red;
} Colour;

const static Colour colour_all_off = {0,       0,    0};
const static Colour colour_all_on  = {0xff, 0xff, 0xff};
const static Colour colour_green   = {0xff,    0,    0};
const static Colour colour_red     = {   0,    0, 0xff};

static SemaphoreHandle_t pulsate_colour_mutex;
static Colour pulsate_colour;  // Protected by pulsate_colour_mutex.

static void fade_end_interrupt() {
  // Read LEDC interrupt state.
  uint32_t intr_st = LEDC.int_st.val;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // Fade change end event for high speed channel 0.
  if (LEDC.int_st.duty_chng_end_hsch0) {
    xEventGroupSetBitsFromISR(
        pulsate_event_group,
        PULSATE_EVENT_FADE_END_GREEN_BIT,
        &xHigherPriorityTaskWoken);
  }
  if (LEDC.int_st.duty_chng_end_hsch1) {
    xEventGroupSetBitsFromISR(
        pulsate_event_group,
        PULSATE_EVENT_FADE_END_ORANGE_BIT,
        &xHigherPriorityTaskWoken);
  }
  if (LEDC.int_st.duty_chng_end_hsch2) {
    xEventGroupSetBitsFromISR(
        pulsate_event_group,
        PULSATE_EVENT_FADE_END_RED_BIT,
        &xHigherPriorityTaskWoken);
  }

  // Actual set happens in timer (RTOS) daemon task, so a context
  // switch may be necesssary to do the actual set promptly. If so,
  // ask for a context switch upon return from ISR.
  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }

  // Clear LEDC interrupts.
  LEDC.int_clr.val = intr_st;
}

static void pwm_init(void) {
  periph_module_enable(PERIPH_LEDC_MODULE);

  ledc_timer_config_t default_timer_conf = {
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .bit_num = 8, // Does not exist: LEDC_TIMER_8_BIT,
    .freq_hz = PWM_FREQUENCY_HZ,
    .timer_num = LEDC_TIMER_0,
  };

  ESP_ERROR_CHECK(ledc_timer_config(&default_timer_conf));

  ledc_channel_config_t default_ch_conf = {
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .intr_type = LEDC_INTR_FADE_END, //LEDC_INTR_DISABLE,
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

  // Enable interrupt for end of fades.
  ledc_isr_register(
      &fade_end_interrupt, NULL, ESP_INTR_FLAG_SHARED, NULL);
}

static void update_duty() {
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
  ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2);
}

static void set_colours_raw(char green, char orange, char red) {
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, green);
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, orange);
  ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, red);

  update_duty();
}  

static void set_fade(ledc_duty_direction_t direction, Colour max_values) {
  ESP_LOGD(LOG_TAG, "set_fade: direction=%i, max=(%i,%i,%i)",
      direction, max_values.green, max_values.orange, max_values.red);

  Colour start_values;
  if (direction == LEDC_DUTY_DIR_INCREASE) {
    start_values = colour_all_off;
  } else {      // LEDC_DUTY_DIR_DECREASE
    start_values = max_values;
  }

  uint32_t ticks;

  if (max_values.green > 0) {
    ticks = lroundf(FADE_DURATION_TOTAL_TICKS / (float)max_values.green);

    // Cannot stretch the gradient beyond the duty cycle.
    ticks = ticks > LEDC_DUTY_CYCLE_HSCH0 ?  LEDC_DUTY_CYCLE_HSCH0 : ticks;
    ledc_set_fade(LEDC_HIGH_SPEED_MODE,
                  LEDC_CHANNEL_0,
                  start_values.green,
                  direction,
                  max_values.green,
                  ticks,
                  1);
    ESP_LOGD(LOG_TAG, "set_fade: green fade %i ticks per step", ticks);
  } else {
    // Turn the channel off if not part of the fade.
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
  }

  if (max_values.orange > 0) {
    ticks = lroundf(FADE_DURATION_TOTAL_TICKS / (float)max_values.orange);

    // Cannot stretch the gradient beyond the duty cycle.
    ticks = ticks > LEDC_DUTY_CYCLE_HSCH1 ?  LEDC_DUTY_CYCLE_HSCH1 : ticks;
    ledc_set_fade(LEDC_HIGH_SPEED_MODE,
                  LEDC_CHANNEL_1,
                  start_values.orange,
                  direction,
                  max_values.orange,
                  ticks,
                  1);
    ESP_LOGD(LOG_TAG, "set_fade: orange fade %i ticks per step", ticks);
  } else {
    // Turn the channel off if not part of the fade.
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 0);
  }

  if (max_values.red > 0) {
    ticks = lroundf(FADE_DURATION_TOTAL_TICKS / (float)max_values.red);

    // Cannot stretch the gradient beyond the duty cycle.
    ticks = ticks > LEDC_DUTY_CYCLE_HSCH2 ?  LEDC_DUTY_CYCLE_HSCH2 : ticks;
    ledc_set_fade(LEDC_HIGH_SPEED_MODE,
                  LEDC_CHANNEL_2,
                  start_values.red,
                  direction,
                  max_values.red,
                  ticks,
                  1);
    ESP_LOGD(LOG_TAG, "set_fade: red fade %i ticks per step", ticks);
  } else {
    // Turn the channel off if not part of the fade.
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_2, 0);
  }

  // Commit.
  update_duty();
}

static void set_colours(Colour colour) {
  set_colours_raw(colour.green, colour.orange, colour.red);
}

static void delay_task(int ms) {
  vTaskDelay(ms / portTICK_PERIOD_MS);
}

static void mutex_lock(SemaphoreHandle_t* mutex) {
  while (xSemaphoreTake(
      *mutex,
      DELAY_WAIT_FOR_SEMAPHORE / portTICK_PERIOD_MS) == pdFALSE) {
  }
}

static void mutex_unlock(SemaphoreHandle_t* mutex) {
  while (xSemaphoreGive(*mutex) == pdFALSE);
}

static void pulsate_task(void *p) {
  bool pulsing = false;

/* This is the bit number from the PULSATE_EVENT_FADE_END_* family that
 * corresponds to the 'last' colour that will complete the fade (i.e. the
 * largest colour value). This marks the point at which the pulsation should
 * reverse direction. This is used, rather than each colour having independent
 * reversing, to ensure that the colours stay in sync which would not otherwise
 * be guaranteed (due to the maximum value allowed of duty_cycle_num in LEDC) */
  uint32_t sentinel_event_bit = 0;

  ledc_duty_direction_t direction;
  Colour colour;

  ESP_LOGD(LOG_TAG, "pulsate_task: Task created");

  while (true) {
    EventBits_t bits = xEventGroupWaitBits(
        pulsate_event_group,
        PULSATE_EVENT_START_BIT | PULSATE_EVENT_STOP_BIT |
            PULSATE_EVENT_FADE_END_GREEN_BIT |
            PULSATE_EVENT_FADE_END_ORANGE_BIT |
            PULSATE_EVENT_FADE_END_RED_BIT,
        pdTRUE,     // Clear bits on return.
        pdFALSE,    // Wait for all bits.
        DELAY_WAIT_FOR_FADE_EVENT_MS / portTICK_PERIOD_MS);
    if (bits & PULSATE_EVENT_STOP_BIT) {
      ESP_LOGD(LOG_TAG, "pulsate_task: Pulsations canceled");
      pulsing = false;
      // Whatever called this is responsible for adjusting LED.
      continue;
    } else if (bits & PULSATE_EVENT_START_BIT) {
      ESP_LOGD(LOG_TAG, "pulsate_task: Pulsation started");
      // Start pulsating. Copy target colours into internal buffer.
      direction = LEDC_DUTY_DIR_INCREASE;

      mutex_lock(&pulsate_colour_mutex);
      colour = pulsate_colour;
      mutex_unlock(&pulsate_colour_mutex);

      pulsing = true;

      if (colour.green >= colour.orange && colour.green >= colour.red) {
        sentinel_event_bit = PULSATE_EVENT_FADE_END_GREEN_BIT;
      } else if (colour.orange >= colour.red) {
        sentinel_event_bit = PULSATE_EVENT_FADE_END_ORANGE_BIT;
      } else {
        sentinel_event_bit = PULSATE_EVENT_FADE_END_RED_BIT;
      }

      set_fade(direction, colour);
    } else if (pulsing && (bits & sentinel_event_bit)) {
      ESP_LOGD(LOG_TAG, "pulsate_task: Fade completed, reversing direction");
      // The last colour fade has completed (the 'sentinel' event bit has been
      // set) => Reverse direction.
      direction = (direction == LEDC_DUTY_DIR_INCREASE ?
          LEDC_DUTY_DIR_DECREASE : LEDC_DUTY_DIR_INCREASE);
      set_fade(direction, colour);
    }
  }
}

static void status_task(void *p) {
  // Counter of number of connection-established blinks remaining.
  //  -1: Not connected.
  //   0: Already blinked connection success.
  // 1-3: Number of blinks remaining.
  int connected_blinks_remaining = -1;

  while (true) {
    // Is wifi connected?
    if (xEventGroupGetBits(wifi_event_group) & WIFI_EVENT_CONNECTED_BIT) {
      // Already blinked that we are connected?
      if (connected_blinks_remaining == 0) {
        delay_task(DELAY_BLINK_MS);
      } else {
        // Need to do blinks to show connection. If the connection
        // has just been established, set the blink counter to the
        // desired number of blinks.
        if (connected_blinks_remaining < 0) {
          connected_blinks_remaining = STATUS_BLINKS_ON_CONNECT;
        }
        set_colours(colour_green);
        delay_task(DELAY_BLINK_MS);
        set_colours(colour_all_off);
        delay_task(DELAY_BLINK_MS);
      
        --connected_blinks_remaining;
      }
    } else {
      set_colours(colour_red);
      delay_task(DELAY_BLINK_MS);
      set_colours(colour_all_off);
      delay_task(DELAY_BLINK_MS);

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
      delay_task(DELAY_SERVER_ERROR_MS);
      continue;
    }

    int reuse = 1;
    if (setsockopt(listen_socket,
                   SOL_SOCKET,
                   SO_REUSEADDR,
                   &reuse, sizeof(reuse)) < 0) {
      delay_task(DELAY_SERVER_ERROR_MS);
      continue;
    }

    ESP_LOGI(LOG_TAG, "Bind ...");
    memset(&serv_sock_addr, 0, sizeof(serv_sock_addr));
    serv_sock_addr.sin_family = AF_INET;
    serv_sock_addr.sin_addr.s_addr = 0;
    serv_sock_addr.sin_port = htons(SERVER_TCP_PORT);
  
    if (bind(listen_socket,
             (struct sockaddr*)&serv_sock_addr,
             sizeof(serv_sock_addr))) {
      ESP_LOGW(LOG_TAG, "bind(...) failed");
      close(listen_socket);

      delay_task(DELAY_SERVER_ERROR_MS);
      continue;
    }

    ESP_LOGI(LOG_TAG, "Listen ...");
    if (listen(listen_socket, 32)) {
      ESP_LOGW(LOG_TAG, "listen(...) failed");
      close(listen_socket); 

      delay_task(DELAY_SERVER_ERROR_MS);
      continue;
    }

    memset(recv_buf, 0, SERVER_RECV_BUF_LEN);
    while (true) {
      client_socket = accept(
          listen_socket,
          (struct sockaddr*)&client_sock_addr,
          &client_sock_addr_len);

      if (client_socket < 0) {
        ESP_LOGW(LOG_TAG, "accept(...) failed");
        delay_task(DELAY_SERVER_ERROR_MS);
        break;
      }

      char* ip_addr_string = inet_ntoa(client_sock_addr.sin_addr);
      ESP_LOGI(LOG_TAG, "Established connection from %s", ip_addr_string)

      char bytes = recv(client_socket, recv_buf, SERVER_RECV_BUF_LEN - 1, 0);
      if (bytes == 4) {
        ESP_LOGI(LOG_TAG, "Received (%x, %x, %x, %x)",
            recv_buf[0], recv_buf[1], recv_buf[2], recv_buf[3]);
        Colour target = {recv_buf[1], recv_buf[2], recv_buf[3]};

        if (recv_buf[0] == 'S') {        // Set
          set_colours(target);
        } else if (recv_buf[0] == 'F') { // Fade
          set_fade(LEDC_DUTY_DIR_INCREASE, target);
        }

        if (recv_buf[0] == 'P') {        // Pulsate.
          mutex_lock(&pulsate_colour_mutex);
          pulsate_colour = target;
          mutex_unlock(&pulsate_colour_mutex);

          xEventGroupSetBits(pulsate_event_group, PULSATE_EVENT_START_BIT);
        } else {
          xEventGroupSetBits(pulsate_event_group, PULSATE_EVENT_STOP_BIT);
        }
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
                           SERVER_TASK_NAME,
                           SERVER_TASK_STACK_WORDS,
                           NULL,
                           SERVER_TASK_PRORITY,
                           &server_handle) == pdTRUE);
}

static void status_task_create(void) {
  configASSERT(xTaskCreate(status_task,
                           STATUS_TASK_NAME,
                           STATUS_TASK_STACK_WORDS,
                           NULL,
                           STATUS_TASK_PRORITY,
                           &status_handle) == pdTRUE);
}

static void pulsate_task_create(void) {
  configASSERT(xTaskCreate(pulsate_task,
                           PULSATE_TASK_NAME,
                           PULSATE_TASK_STACK_WORDS,
                           NULL,
                           PULSATE_TASK_PRORITY,
                           &pulsate_handle) == pdTRUE);
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
      xEventGroupSetBits(wifi_event_group, WIFI_EVENT_CONNECTED_BIT);
      server_task_create();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      ESP_LOGI(LOG_TAG, "WIFI event: Disconnect...");
      xEventGroupClearBits(wifi_event_group, WIFI_EVENT_CONNECTED_BIT);

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
  configASSERT(wifi_event_group != NULL);
  ESP_ERROR_CHECK(esp_event_loop_init(wifi_event_handler, NULL));

  pulsate_event_group = xEventGroupCreate();
  configASSERT(pulsate_event_group != NULL);
  pulsate_colour_mutex = xSemaphoreCreateMutex();
  configASSERT(pulsate_colour_mutex != NULL);

  // Light all LEDs and pause at startup (test LEDs).
  pwm_init();
  set_colours(colour_all_on);
  delay_task(DELAY_STARTUP_LED_ON_MS);
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

  ESP_LOGI(LOG_TAG, "Setting WiFi configuration SSID %s...",
      wifi_config.sta.ssid);
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  // Start helper tasks.
  status_task_create();
  pulsate_task_create();
}
