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
#include "nvs.h"

// Define these, or run make as ...
//   EXTRA_CPPFLAGS='-DWIFI_SSID=\"SSID\" -DWIFI_PASSWORD=\"PASSWORD\"' make
//
//#define WIFI_SSID
//#define WIFI_PASSWORD

#define SERVER_TASK_NAME             "server"
#define SERVER_TASK_STACK_WORDS      2<<13
#define SERVER_TASK_PRORITY          8
#define SERVER_UDP_PORT              7000
#define SERVER_COMMAND_LEN           4
#define SERVER_RECV_BUF_LEN          SERVER_COMMAND_LEN + 1
#define SERVER_RECV_TIMEOUT_SECS     10
#define SERVER_ERROR_BUFFER_LEN      80

#define STATUS_TASK_NAME             "status"
#define STATUS_TASK_STACK_WORDS      2<<11
#define STATUS_TASK_PRORITY          4
#define STATUS_BLINKS_ON_CONNECT     3

#define PULSATE_TASK_NAME            "pulsate"
#define PULSATE_TASK_STACK_WORDS     2<<11
#define PULSATE_TASK_PRORITY         6

#define BLINK_TASK_NAME              "blink"
#define BLINK_TASK_STACK_WORDS       2<<11
#define BLINK_TASK_PRORITY           6

#define PWM_FREQUENCY_HZ             1000
// Total desired fade length (in ticks @ pwm_frequency)
#define FADE_DURATION_TOTAL_TICKS    2000

#define DELAY_STARTUP_LED_ON_MS      3000
#define DELAY_BLINK_MS               1000
#define DELAY_SERVER_ERROR_MS        1000
#define DELAY_ARB_LONGWAIT_MS        10000

#define GPIO_GREEN                   14
#define GPIO_ORANGE                  27
#define GPIO_RED                     25

#define NVS_NAMESPACE                "traffic-light"
#define NVS_KEY                      "last-command"

const static char *LOG_TAG = "TrafficLightServer";

static EventGroupHandle_t wifi_event_group;
// Connected to wifi?
const static int WIFI_EVENT_CONNECTED_BIT = BIT0;
// Change to the state of the wifi connection?
const static int WIFI_EVENT_CHANGE_BIT = BIT1;

static EventGroupHandle_t pulsate_event_group;
// Request start/stop of pulsation.
const static int PULSATE_EVENT_START_REQ_BIT = BIT0;
const static int PULSATE_EVENT_STOP_REQ_BIT = BIT1;
// When the respective channel fades have ended.
const static int PULSATE_EVENT_DUTY_DONE_GREEN_BIT = BIT2;
const static int PULSATE_EVENT_DUTY_DONE_ORANGE_BIT = BIT3;
const static int PULSATE_EVENT_DUTY_DONE_RED_BIT = BIT4;
// When the pulsation has stopped.
const static int PULSATE_EVENT_STOPPED_BIT = BIT5;

static EventGroupHandle_t blink_event_group;
const static int BLINK_EVENT_START_REQ_BIT = BIT0;
const static int BLINK_EVENT_STOP_REQ_BIT = BIT1;
const static int BLINK_EVENT_STOPPED_BIT = BIT2;

static xTaskHandle server_handle;
static xTaskHandle status_handle;
static xTaskHandle pulsate_handle;
static xTaskHandle blink_handle;

typedef struct {
  uint8_t green, orange, red;
} Colour;

const static Colour colour_all_off = {0,       0,    0};
const static Colour colour_all_on  = {0xff, 0xff, 0xff};
const static Colour colour_green   = {0xff,    0,    0};
const static Colour colour_red     = {   0,    0, 0xff};

static SemaphoreHandle_t shared_colour_mutex;
static Colour shared_colour;  // Protected by shared_colour_mutex.

static void fade_end_interrupt() {
  // Read LEDC interrupt state.
  uint32_t intr_st = LEDC.int_st.val;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // Fade change end event for high speed channel 0.
  if (LEDC.int_st.duty_chng_end_hsch0) {
    xEventGroupSetBitsFromISR(
        pulsate_event_group,
        PULSATE_EVENT_DUTY_DONE_GREEN_BIT,
        &xHigherPriorityTaskWoken);
  }
  if (LEDC.int_st.duty_chng_end_hsch1) {
    xEventGroupSetBitsFromISR(
        pulsate_event_group,
        PULSATE_EVENT_DUTY_DONE_ORANGE_BIT,
        &xHigherPriorityTaskWoken);
  }
  if (LEDC.int_st.duty_chng_end_hsch2) {
    xEventGroupSetBitsFromISR(
        pulsate_event_group,
        PULSATE_EVENT_DUTY_DONE_RED_BIT,
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

static void set_colours_raw(uint8_t green, uint8_t orange, uint8_t red) {
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
      DELAY_ARB_LONGWAIT_MS / portTICK_PERIOD_MS) == pdFALSE) {
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
        PULSATE_EVENT_START_REQ_BIT | PULSATE_EVENT_STOP_REQ_BIT |
            PULSATE_EVENT_DUTY_DONE_GREEN_BIT |
            PULSATE_EVENT_DUTY_DONE_ORANGE_BIT |
            PULSATE_EVENT_DUTY_DONE_RED_BIT,
        pdTRUE,     // Clear bits on return.
        pdFALSE,    // Wait for all bits.
        DELAY_ARB_LONGWAIT_MS / portTICK_PERIOD_MS);

    if (bits & PULSATE_EVENT_STOP_REQ_BIT) {
      ESP_LOGD(LOG_TAG, "pulsate_task: Pulsation stop requested");
      pulsing = false;
      xEventGroupSetBits(pulsate_event_group, PULSATE_EVENT_STOPPED_BIT);
      continue;
    } else if (bits & PULSATE_EVENT_START_REQ_BIT) {
      ESP_LOGD(LOG_TAG, "pulsate_task: Pulsation start requested");

      xEventGroupClearBits(pulsate_event_group, PULSATE_EVENT_STOPPED_BIT);
      pulsing = true;
      direction = LEDC_DUTY_DIR_INCREASE;

      {
        // Copy target colours into internal buffer.
        mutex_lock(&shared_colour_mutex);
        colour = shared_colour;
        mutex_unlock(&shared_colour_mutex);
      }

      // Work out which colour has the further 'distance' to fade, that
      // will mark the point at which the pulsate is reversed. See comment
      // at top of file.
      if (colour.green >= colour.orange && colour.green >= colour.red) {
        sentinel_event_bit = PULSATE_EVENT_DUTY_DONE_GREEN_BIT;
      } else if (colour.orange >= colour.red) {
        sentinel_event_bit = PULSATE_EVENT_DUTY_DONE_ORANGE_BIT;
      } else {
        sentinel_event_bit = PULSATE_EVENT_DUTY_DONE_RED_BIT;
      }

      // Set everything to off to avoid transition artefacts.
      set_colours(colour_all_off);

      // Wait for all duty updates to be completed.
      xEventGroupWaitBits(pulsate_event_group,
          PULSATE_EVENT_DUTY_DONE_GREEN_BIT |
              PULSATE_EVENT_DUTY_DONE_ORANGE_BIT |
              PULSATE_EVENT_DUTY_DONE_RED_BIT,
          pdTRUE,   // Clear bits on return.
          pdTRUE,   // Wait for all bits.
          DELAY_ARB_LONGWAIT_MS / portTICK_PERIOD_MS);

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

static void stop_blinking() {
  xEventGroupSetBits(blink_event_group, BLINK_EVENT_STOP_REQ_BIT);
  while (xEventGroupWaitBits(
      blink_event_group, BLINK_EVENT_STOPPED_BIT,
      pdFALSE, pdTRUE, DELAY_ARB_LONGWAIT_MS / portTICK_PERIOD_MS) == 0) {
    ESP_LOGD(LOG_TAG, "stop_blinking: Waiting for blink task to stop");
  }
}

static void stop_pulsating() {
  xEventGroupSetBits(pulsate_event_group, PULSATE_EVENT_STOP_REQ_BIT);
  while (xEventGroupWaitBits(
      pulsate_event_group, PULSATE_EVENT_STOPPED_BIT,
      pdFALSE, pdTRUE, DELAY_ARB_LONGWAIT_MS / portTICK_PERIOD_MS) == 0) {
    ESP_LOGD(LOG_TAG, "stop_pulsating: Waiting for pulsate task to stop");
  }
}

static bool parse_command(uint8_t recv_buf[SERVER_COMMAND_LEN]) {
  ESP_LOGI(LOG_TAG, "parse_command: Parsing (0x%x, 0x%x, 0x%x, 0x%x)",
      recv_buf[0], recv_buf[1], recv_buf[2], recv_buf[3]);

  uint8_t command = recv_buf[0];
  Colour target = {recv_buf[1], recv_buf[2], recv_buf[3]};

  switch (command) {
    case 'S':   // Set.
    case 'F':   // Fade.
    case 'P':   // Pulsate.
    case 'B':   // Blink.
      break;
    default:
      return false;
  }

  // Request a stop of helper tasks (pulsate + blink) that might be working on
  // the LEDs.
  if (command == 'S' || command == 'F' || command == 'P') {
    stop_blinking();
  }

  if (command == 'S' || command == 'F' || command == 'B') {
    stop_pulsating();
  }

  switch (command) {
    case 'S':
      ESP_LOGI(LOG_TAG, "parse_command: Set colour.")
      set_colours(target);
      break;
    case 'F':
      ESP_LOGI(LOG_TAG, "parse_command: Fade colour.")
      set_fade(LEDC_DUTY_DIR_INCREASE, target);
      break;
    case 'P':
      ESP_LOGI(LOG_TAG, "parse_command: Pulsate colour.")
      {
        mutex_lock(&shared_colour_mutex);
        shared_colour = target;
        mutex_unlock(&shared_colour_mutex);
      }
      xEventGroupSetBits(pulsate_event_group, PULSATE_EVENT_START_REQ_BIT);
      break;
    case 'B':
      ESP_LOGI(LOG_TAG, "parse_command: Blink colour.")
      {
        mutex_lock(&shared_colour_mutex);
        shared_colour = target;
        mutex_unlock(&shared_colour_mutex);
      }
      xEventGroupSetBits(blink_event_group, BLINK_EVENT_START_REQ_BIT);
      break;
  }

  return true;
}

static bool write_to_nvs(uint8_t buffer[SERVER_COMMAND_LEN]) {
  ESP_LOGI(LOG_TAG, "write_to_nvs: Writing (%x, %x, %x, %x)",
      buffer[0], buffer[1], buffer[2], buffer[3]);

  nvs_handle handle;

  if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
    ESP_LOGE(LOG_TAG,
             "write_to_nvs: Unable to open NVS for writing, skipping...");
    return false;
  }

  if (nvs_set_blob(handle, NVS_KEY, buffer, SERVER_COMMAND_LEN) != ESP_OK) {
    ESP_LOGE(LOG_TAG,
             "write_to_nvs: Unable to write data, skipping ...");
    nvs_close(handle);
    return false;
  }

  if (nvs_commit(handle) != ESP_OK) {
    ESP_LOGE(LOG_TAG,
             "write_to_nvs: Unable to commit writes, skipping ...");
    nvs_close(handle);
    return false;
  }

  nvs_close(handle);
  return true;
}

static bool read_from_nvs(uint8_t buffer[SERVER_COMMAND_LEN]) {
  ESP_LOGI(LOG_TAG, "read_from_nvs: Reading");

  nvs_handle handle;

  if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
    ESP_LOGE(LOG_TAG,
             "read_from_nvs: Unable to open NVS for reading, skipping...");
    return false;
  }

  size_t command_size = SERVER_COMMAND_LEN;
  if (nvs_get_blob(handle, NVS_KEY, buffer, &command_size) != ESP_OK) {
    ESP_LOGE(LOG_TAG,
             "read_from_nvs: Unable to read data, skipping ...");
    nvs_close(handle);
    return false;
  }

  nvs_close(handle);
  return true;
}

static void blink_task(void* p) {
  // Is the blink task actively blinking?
  bool blinking_active = false;

  // Are the lights lit, or is this an off cycle?
  bool blink_lit_now = false;

  Colour colour;
  int wait_time_ms = DELAY_BLINK_MS;

  while (true) {
    // This wait serves both as the wait for a blink start,
    // but also the timer between blinks -- the wait_time_ms is
    // altered to do so.
    EventBits_t bits = xEventGroupWaitBits(
        blink_event_group,
        BLINK_EVENT_START_REQ_BIT | BLINK_EVENT_STOP_REQ_BIT,
        pdTRUE,   // Clear on exit.
        pdFALSE,  // Wait for all.
        wait_time_ms / portTICK_PERIOD_MS);

    if (bits & BLINK_EVENT_STOP_REQ_BIT) {
      ESP_LOGD(LOG_TAG, "blink_task: Blink stop requested");
      blinking_active = false;
      wait_time_ms = DELAY_ARB_LONGWAIT_MS;
      xEventGroupSetBits(blink_event_group, BLINK_EVENT_STOPPED_BIT);
      continue;
    } else if (bits & BLINK_EVENT_START_REQ_BIT) {
      ESP_LOGD(LOG_TAG, "blink_task: Blink start requested");
      xEventGroupClearBits(blink_event_group, BLINK_EVENT_STOPPED_BIT);
      blinking_active = true;
      wait_time_ms = DELAY_BLINK_MS;

      {
        mutex_lock(&shared_colour_mutex);
        colour = shared_colour;
        mutex_unlock(&shared_colour_mutex);
      }

      blink_lit_now = true;
      ESP_LOGD(LOG_TAG, "blink_task: First blink (%i, %i, %i) -> on!",
          colour.green, colour.orange, colour.red);
      set_colours(colour);
    } else if (blinking_active) {
      if (blink_lit_now) {
        ESP_LOGD(LOG_TAG, "blink_task: Blink (%i, %i, %i) -> on!",
            colour.green, colour.orange, colour.red);
        set_colours(colour_all_off);
      } else {
        ESP_LOGD(LOG_TAG, "blink_task: Blink off!");
        set_colours(colour);
      }
      blink_lit_now = !blink_lit_now;
    }
  }
}

static void status_task(void *p) {
  uint8_t buffer[SERVER_COMMAND_LEN];
  bool acted_on_disconnect = false, acted_on_connect = false;

  while (true) {
    EventBits_t bits = xEventGroupGetBits(wifi_event_group);

    if (!acted_on_connect && (bits & WIFI_EVENT_CONNECTED_BIT)) {
      acted_on_connect = true;
      acted_on_disconnect = false;
      ESP_LOGI(LOG_TAG,
               "status_task: Wifi available, restoring state if any");

      xEventGroupSetBits(blink_event_group, BLINK_EVENT_STOP_REQ_BIT);
      stop_blinking();
      if (read_from_nvs(buffer) && !parse_command(buffer)) {
        set_colours(colour_all_off);
      }
    } else if (!acted_on_disconnect &&
        ((bits & WIFI_EVENT_CONNECTED_BIT) == 0)) {
      stop_pulsating();

      acted_on_disconnect = true;
      acted_on_connect = false;
      {
        mutex_lock(&shared_colour_mutex);
        shared_colour = colour_red;
        mutex_unlock(&shared_colour_mutex);
      }
      xEventGroupSetBits(blink_event_group, BLINK_EVENT_START_REQ_BIT);
    }
    xEventGroupWaitBits(wifi_event_group,
        WIFI_EVENT_CHANGE_BIT,
        pdTRUE,
        pdTRUE,
        DELAY_ARB_LONGWAIT_MS / portTICK_PERIOD_MS);
  }

  // Never reached.
  vTaskDelete(NULL);
  return;
}

static void server_task(void *p) {
  int server_socket;
  struct sockaddr_in serv_sock_addr, client_addr;
  socklen_t client_addr_len = sizeof(client_addr);
  uint8_t recv_buf[SERVER_RECV_BUF_LEN];
  char error_buffer[SERVER_ERROR_BUFFER_LEN];

  while (true) {
    ESP_LOGD(LOG_TAG, "server_task: Create socket ...");
    server_socket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (server_socket < 0) {
      ESP_LOGW(LOG_TAG, "server_task: socket(...) failed");
      delay_task(DELAY_SERVER_ERROR_MS);
      continue;
    }

    ESP_LOGD(LOG_TAG, "server_task: Bind ...");
    memset(&serv_sock_addr, 0, sizeof(serv_sock_addr));
    serv_sock_addr.sin_family = AF_INET;
    serv_sock_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    serv_sock_addr.sin_port = htons(SERVER_UDP_PORT);
  
    if (bind(server_socket,
             (struct sockaddr*)&serv_sock_addr,
             sizeof(serv_sock_addr))) {
      ESP_LOGW(LOG_TAG, "server_task: bind(...) failed");
      close(server_socket);

      delay_task(DELAY_SERVER_ERROR_MS);
      continue;
    }

    memset(recv_buf, 0, SERVER_RECV_BUF_LEN);
    while (true) {
      ssize_t bytes = recvfrom(
          server_socket,
          recv_buf,
          SERVER_RECV_BUF_LEN-1,
          0,
          (struct sockaddr *)&client_addr,
          &client_addr_len);

      char* ip_addr_string = inet_ntoa(client_addr.sin_addr);
      ESP_LOGI(LOG_TAG, "server_task: Received data from %s", ip_addr_string)

      if (bytes == SERVER_COMMAND_LEN) {
        parse_command(recv_buf);
        write_to_nvs(recv_buf);
      } else if (bytes < 0) {
        strerror_r(errno, error_buffer, SERVER_ERROR_BUFFER_LEN);
        ESP_LOGW(LOG_TAG, "server_task: recvfrom error (%i): %s",
            bytes, error_buffer);
      } else {
        ESP_LOGI(LOG_TAG,
            "server_task: recvfrom received %i bytes, expected %i",
            bytes, SERVER_COMMAND_LEN);
      }
    }

    close(server_socket);
  }

  // Never reached.
  vTaskDelete(NULL);
  return;
}

static void server_task_create(void) {
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

static void blink_task_create(void) {
  configASSERT(xTaskCreate(blink_task,
                           BLINK_TASK_NAME,
                           BLINK_TASK_STACK_WORDS,
                           NULL,
                           BLINK_TASK_PRORITY,
                           &blink_handle) == pdTRUE);
}


static esp_err_t wifi_event_handler(void *ctx, system_event_t *event)
{
  switch(event->event_id) {
    case SYSTEM_EVENT_STA_START:
      ESP_LOGD(LOG_TAG, "wifi_event_handler: WIFI start ...");
      esp_wifi_connect();
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      ESP_LOGD(LOG_TAG, "wifi_event_handler: WIFI got IP...");
      xEventGroupSetBits(
          wifi_event_group, WIFI_EVENT_CONNECTED_BIT | WIFI_EVENT_CHANGE_BIT);
      server_task_create();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      ESP_LOGD(LOG_TAG, "wifi_event_hanlder: Disconnected...");
      xEventGroupClearBits(
          wifi_event_group, WIFI_EVENT_CONNECTED_BIT);
      xEventGroupSetBits(
          wifi_event_group, WIFI_EVENT_CHANGE_BIT);

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
  blink_event_group = xEventGroupCreate();
  configASSERT(blink_event_group != NULL);
  shared_colour_mutex = xSemaphoreCreateMutex();
  configASSERT(shared_colour_mutex != NULL);

  // Light all LEDs and pause at startup (to test the LEDs).
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

  ESP_LOGI(LOG_TAG, "app_main: Setting WiFi configuration SSID %s...",
      wifi_config.sta.ssid);
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());

  // Start helper tasks.
  blink_task_create();
  status_task_create();
  pulsate_task_create();
}
