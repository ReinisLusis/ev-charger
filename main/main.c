#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "mqtt_client.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/adc.h"
#include "driver/gptimer.h"
#include "soc/gpio_sig_map.h"
#include "esp_adc_cal.h"
#include "secrets.h"

#define MQTT_TOPIC      "ev_charger/control"
#define MQTT_TOPIC_S    "ev_charger/state"

#define LEDC_LS_TIMER          LEDC_TIMER_1
#define LEDC_LS_MODE           LEDC_LOW_SPEED_MODE
#define LED_PIN                GPIO_NUM_2
#define LEDC_LS_CH0_GPIO       (LED_PIN)
#define LEDC_LS_CH0_CHANNEL    LEDC_CHANNEL_0
#define LED_DUTY_ON             (4000)
#define LED_TICK_MS             (50)

#define LEDC_TEST_CH_NUM       1
static ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
    {
        .channel    = LEDC_LS_CH0_CHANNEL,
        .duty       = 0,
        .gpio_num   = LEDC_LS_CH0_GPIO,
        .speed_mode = LEDC_LS_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_LS_TIMER,
        .flags.output_invert = 0
    },
};

#define ADC_CHANNEL     ADC_CHANNEL_6
#define ADC_WIDTH       ADC_WIDTH_BIT_12
#define ADC_ATTEN       ADC_ATTEN_DB_12

#define TIMER_FREQUENCY 1000 // Timer frequency hz

#define GPIO_SOLID_STATE_RELAY_12V 25
#define GPIO_SOLID_STATE_RELAY_CHARGE 26
#define GPIO_OPTOCOUPLER_12V_POS 27
#define GPIO_OPTOCOUPLER_12V_NEG 32

typedef enum {
    TIMER_STATE_ON_1 = 0,
    TIMER_STATE_ON_2 = 1,
    TIMER_STATE_OFF_1 = 2,
    TIMER_STATE_OFF_2 = 3,
    TIMER_STATE_SIZE = TIMER_STATE_OFF_2 + 1
} timer_state_t;

typedef enum {
    CP_LINE_STATE_A,      // 12V: No vehicle connected, Standby, Charging not possible
    CP_LINE_STATE_B,      // 9V: Vehicle connected, Charging not possible
    CP_LINE_STATE_C,      // 6V: Charging allowed, Charging possible
    CP_LINE_STATE_D,      // 3V: Ventilation required, Charging possible
    CP_LINE_STATE_E,      // 0V: EVSE shutdown, Charging not possible
    CP_LINE_STATE_F,      // -12V: Error, EVSE not available, Charging not possible
    CP_LINE_STATE_ERROR   // Custom state for error handling
} cp_line_state_t;

typedef enum {
    CHARGER_STATE_INITIALIZING,   // 0V output
    CHARGER_STATE_OFF,            // 0V output
    CHARGER_STATE_WAIT_CAR,       // 12V output, waiting for car to pull down
    CHARGER_STATE_NEGOTIATE,      // +-12V PWM, negotiating charge current
    CHARGER_STATE_CHARGING,       // +-12V PWM, contactor on, charging
    CHARGER_STATE_ERROR           // 0v output
} charger_state_t;

// Blinked out on the status LED while in CHARGER_STATE_ERROR, as N short pulses (see led_tick_cb).
// Values are the order faults are checked in, not a severity ranking.
typedef enum {
    LED_ERROR_NONE = 0,
    LED_ERROR_INVALID_COMMAND,      // MQTT control message outside the accepted current range
    LED_ERROR_NO_PILOT_VOLTAGE,     // no 12V/9V seen while waiting for a vehicle
    LED_ERROR_NO_DIODE,             // no -12V half-cycle seen when negotiation should start
    LED_ERROR_LOST_DIODE,           // -12V half-cycle disappeared mid-charge
    LED_ERROR_UNEXPECTED_CP,        // CP settled on a voltage that isn't valid while charging
} led_error_code_t;

static charger_state_t charger_state = CHARGER_STATE_INITIALIZING;
static led_error_code_t led_error_code = LED_ERROR_NONE;
static int ticks_since_state_change = 0;

uint64_t timer_alarm_values[TIMER_STATE_SIZE] = { 0 };
timer_state_t timer_state = TIMER_STATE_OFF_1;
gptimer_handle_t gptimer = NULL;

// Define custom event base
ESP_EVENT_DEFINE_BASE(TASK_EVENTS);

static const char *TAG = "EV_CHARGER";
enum {
    TASK_ADC_ON_EVENT,
    TASK_ADC_OFF_EVENT
};

static EventGroupHandle_t s_wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

static esp_mqtt_client_handle_t client;

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void update_charger_state(charger_state_t new_charger_state) {
    // error state is latched - only reset_charger_error() may clear it
    if (charger_state == CHARGER_STATE_ERROR) {
        return;
    }
    charger_state = new_charger_state;
    ticks_since_state_change = 0;

    const bool enable12V = charger_state == CHARGER_STATE_WAIT_CAR || 
        charger_state == CHARGER_STATE_NEGOTIATE ||
        charger_state == CHARGER_STATE_CHARGING;
    const bool enableContactor = charger_state == CHARGER_STATE_CHARGING;

    gpio_set_level(GPIO_SOLID_STATE_RELAY_12V, enable12V ? 1 : 0);
    gpio_set_level(GPIO_SOLID_STATE_RELAY_CHARGE, enableContactor ? 1 : 0);

    char buffer[16];
    snprintf(buffer, sizeof(buffer), "%d", charger_state);
    esp_mqtt_client_enqueue(client, MQTT_TOPIC_S, buffer, 0, 1, 1, false);

    ESP_LOGI(TAG, "::update_charger_state %s", buffer);
}

// Explicit escape hatch for the ERROR latch - only an OFF/"0" command should call this,
// never a plain retry, so a real fault always requires a deliberate command to clear.
static void reset_charger_error(void) {
    if (charger_state == CHARGER_STATE_ERROR) {
        led_error_code = LED_ERROR_NONE;
        charger_state = CHARGER_STATE_INITIALIZING;
    }
}

static void wifi_init() {
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();
}

static void set_timer_alarm_values(float duty_cycle) {
    uint32_t resolution;
    ESP_ERROR_CHECK(gptimer_get_resolution(gptimer, &resolution));
    const uint32_t total_duration = resolution / TIMER_FREQUENCY;
    const uint32_t on_duration = duty_cycle * total_duration;
    timer_alarm_values[TIMER_STATE_ON_1] = on_duration / 2;
    timer_alarm_values[TIMER_STATE_ON_2] = on_duration - timer_alarm_values[TIMER_STATE_ON_1];
    const uint32_t off_duration = total_duration - on_duration;
    timer_alarm_values[TIMER_STATE_OFF_1] = off_duration / 2;
    timer_alarm_values[TIMER_STATE_OFF_2] = off_duration - timer_alarm_values[TIMER_STATE_OFF_1];
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = event_data;
    switch (event_id) {
        case MQTT_EVENT_CONNECTED: {
                esp_mqtt_client_subscribe(client, MQTT_TOPIC, 0);
                char buffer[16];
                snprintf(buffer, sizeof(buffer), "%d", charger_state);
                esp_mqtt_client_enqueue(client, MQTT_TOPIC_S, buffer, 0, 1, 1, false);
                ESP_LOGI(TAG, "::mqtt_event_handler MQTT_EVENT_CONNECTED");
            }
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "::mqtt_event_handler MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_DATA:
            if (strncmp(event->topic, MQTT_TOPIC, event->topic_len) == 0) {
                char *message = strndup(event->data, event->data_len);
                ESP_LOGI(TAG, "Received message: '%s'", message);

                const bool isOff = strcmp(message, "OFF") == 0 || strcmp(message, "0") == 0;
                if (isOff) {
                    set_timer_alarm_values(0.0f);
                    reset_charger_error();
                    update_charger_state(CHARGER_STATE_OFF);
                } else {
                    const int current = atoi(message);
                    if (current > 5 && current <= 24) {
                        const int pwm_value = 10 * current / 6;
                        set_timer_alarm_values(pwm_value / 100.0f);
                        if (charger_state == CHARGER_STATE_INITIALIZING || charger_state == CHARGER_STATE_OFF) {
                            update_charger_state(CHARGER_STATE_WAIT_CAR);
                        }
                    } else {
                        set_timer_alarm_values(0.0f);
                        led_error_code = LED_ERROR_INVALID_COMMAND;
                        update_charger_state(CHARGER_STATE_ERROR);
                    }
                }
                free(message);
            }
            break;
        default:
            break;
    }
}

static void mqtt_app_start(void) {
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_URI,
        /*.credentials.username = MQTT_USER,
        .credentials.authentication.password = MQTT_PASS,*/
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, client);
    esp_mqtt_client_start(client);
}

// Timer interrupt service routine
static bool IRAM_ATTR timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    BaseType_t higher_priority_task_woken = pdFALSE;
    bool send_adc_on_event = false;
    bool send_adc_off_event = false;
    do {
        timer_state = (timer_state + 1) % TIMER_STATE_SIZE;
        send_adc_on_event = send_adc_on_event || (timer_state == TIMER_STATE_ON_2);
        send_adc_off_event = send_adc_off_event || (timer_state == TIMER_STATE_OFF_2);
    } while(timer_alarm_values[timer_state] == 0);

    const bool is_on = timer_state == TIMER_STATE_ON_1 || timer_state == TIMER_STATE_ON_2;
    
    if (charger_state == CHARGER_STATE_WAIT_CAR) {
        gpio_set_level(GPIO_OPTOCOUPLER_12V_POS, 1);
        gpio_set_level(GPIO_OPTOCOUPLER_12V_NEG, 0);
    } else if (charger_state == CHARGER_STATE_NEGOTIATE || charger_state == CHARGER_STATE_CHARGING) {
        gpio_set_level(GPIO_OPTOCOUPLER_12V_POS, is_on ? 1 : 0);
        gpio_set_level(GPIO_OPTOCOUPLER_12V_NEG, is_on ? 0 : 1);
    } else {
        gpio_set_level(GPIO_OPTOCOUPLER_12V_POS, 0);
        gpio_set_level(GPIO_OPTOCOUPLER_12V_NEG, 0);
    }

    // reconfigure alarm value
    gptimer_alarm_config_t alarm_config = {
        .alarm_count = edata->alarm_value + timer_alarm_values[timer_state],
    };
    gptimer_set_alarm_action(timer, &alarm_config);
    
    if (send_adc_on_event) {
        const int adc_value = adc1_get_raw(ADC_CHANNEL);
        esp_event_isr_post(TASK_EVENTS, TASK_ADC_ON_EVENT, &adc_value, sizeof(adc_value), &higher_priority_task_woken);
    }
    if (send_adc_off_event) {
        const int adc_value = adc1_get_raw(ADC_CHANNEL);
        esp_event_isr_post(TASK_EVENTS, TASK_ADC_OFF_EVENT, &adc_value, sizeof(adc_value), &higher_priority_task_woken);
    }

    return higher_priority_task_woken == pdTRUE;
}

typedef struct {
    int count;
    int sum;
    int error_count;
} adc_data_t;

static adc_data_t adc_on_data = {0, 0, 0};
static adc_data_t adc_off_data = {0, 0, 0};
static cp_line_state_t cp_line_state_off = CP_LINE_STATE_E;
static cp_line_state_t cp_line_state_on = CP_LINE_STATE_E;

static cp_line_state_t get_cp_line_state(int adc_value) {
    const float voltage = -12 + 24 * (adc_value / 2450.0);
    if (voltage >= 11.0 && voltage <= 13.0) {
        return CP_LINE_STATE_A;
    } else  if (voltage >= 8.0 && voltage <= 10.0) {
        return CP_LINE_STATE_B;
    } else if (voltage >= 5.0 && voltage <= 7.0) {
        return CP_LINE_STATE_C;
    } else if (voltage >= 2.0 && voltage <= 4.0) {
        return CP_LINE_STATE_D;
    } else if (voltage >= -1.0 && voltage <= 1.0) {
        return CP_LINE_STATE_E;
    } else if (voltage >= -13.0 && voltage <= -11.0) {
        return CP_LINE_STATE_F;
    } else {
        return CP_LINE_STATE_ERROR;
    }
}

// Event handler for custom event
static void task_event_handler(void* event_handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (ticks_since_state_change < 100 && event_id == TASK_ADC_ON_EVENT) {
        ticks_since_state_change = ticks_since_state_change + 1;
        if (ticks_since_state_change == 100) {
            if (charger_state == CHARGER_STATE_WAIT_CAR && 
                ((cp_line_state_off != CP_LINE_STATE_A && cp_line_state_off != CP_LINE_STATE_B) ||
                 (cp_line_state_on != CP_LINE_STATE_A && cp_line_state_on != CP_LINE_STATE_B))) {
                // no 12V or 9V voltage, error out
                ESP_LOGE(TAG, "no 12V or 9V voltage, error out");
                led_error_code = LED_ERROR_NO_PILOT_VOLTAGE;
                update_charger_state(CHARGER_STATE_ERROR);
            } else if (charger_state == CHARGER_STATE_WAIT_CAR && cp_line_state_on == CP_LINE_STATE_B) {
                update_charger_state(CHARGER_STATE_NEGOTIATE);
            } else if ((charger_state == CHARGER_STATE_NEGOTIATE || charger_state == CHARGER_STATE_CHARGING) && 
                        cp_line_state_off != CP_LINE_STATE_F) {
                // no -12V negative voltage
                ESP_LOGE(TAG, "no -12V negative voltage");
                led_error_code = LED_ERROR_NO_DIODE;
                update_charger_state(CHARGER_STATE_ERROR);
            } else {
                ESP_LOGI(TAG, "ticks_since_state_change == 100, charger_state - %d, cp_line_state_on - %d, cp_line_state_off - %d", charger_state, cp_line_state_on, cp_line_state_off);
            }
        }
    }
    
    if (event_id == TASK_ADC_ON_EVENT || event_id == TASK_ADC_OFF_EVENT) {
        adc_data_t *adc_data = (event_id == TASK_ADC_ON_EVENT) ? &adc_on_data : &adc_off_data;
        
        // Start ADC conversion
        const int adc_value = *((int*)event_data);
        const int adc_interval = 25; // 25ms since event raised at PWM rate
        
        adc_data->count++;
        adc_data->sum += adc_value;

        if (adc_data->count >= adc_interval) {
            cp_line_state_t *active_cp_line_state = (event_id == TASK_ADC_ON_EVENT) ? &cp_line_state_on : &cp_line_state_off;
            const cp_line_state_t cp_line_state = get_cp_line_state(adc_data->sum / adc_data->count);
            const int max_error_count = 2; // allow two errors before updating state to error
            adc_data->error_count = cp_line_state == CP_LINE_STATE_ERROR ? adc_data->error_count + 1 : 0;

            if (*active_cp_line_state != cp_line_state && (cp_line_state != CP_LINE_STATE_ERROR || adc_data->error_count > max_error_count)) {
                *active_cp_line_state = cp_line_state;
                ESP_LOGI(TAG, "CP_LINE_STATE %s changed to: %i, adc value: %d", event_id == TASK_ADC_ON_EVENT ? "ON" : "OFF", (int)(*active_cp_line_state), adc_data->sum / adc_data->count);

                if (ticks_since_state_change == 100 && charger_state == CHARGER_STATE_WAIT_CAR && cp_line_state_on == CP_LINE_STATE_B && cp_line_state_off == CP_LINE_STATE_B) {
                    update_charger_state(CHARGER_STATE_NEGOTIATE);
                }
                else if (charger_state == CHARGER_STATE_NEGOTIATE && cp_line_state_on == CP_LINE_STATE_C && cp_line_state_off == CP_LINE_STATE_F) {
                    update_charger_state(CHARGER_STATE_CHARGING);
                } else if (charger_state == CHARGER_STATE_CHARGING) {
                    if (cp_line_state_off != CP_LINE_STATE_F) {
                        ESP_LOGE(TAG, "lost -12V negative voltage");
                        led_error_code = LED_ERROR_LOST_DIODE;
                        update_charger_state(CHARGER_STATE_ERROR);
                    } else if (cp_line_state_on == CP_LINE_STATE_B) {
                        update_charger_state(CHARGER_STATE_NEGOTIATE);
                    } else if (cp_line_state_on == CP_LINE_STATE_A) {
                        update_charger_state(CHARGER_STATE_WAIT_CAR);
                    } else if (cp_line_state_on != CP_LINE_STATE_C && cp_line_state_on != CP_LINE_STATE_D) {
                        ESP_LOGE(TAG, "unexpected cp_line_state_on during charging");
                        led_error_code = LED_ERROR_UNEXPECTED_CP;
                        update_charger_state(CHARGER_STATE_ERROR);
                    }
                }
            }

            adc_data->sum = 0;
            adc_data->count = 0;
        }
    }
}

// Drives the status LED. Runs every LED_TICK_MS from the esp_timer task (not an ISR),
// so plain LEDC calls are fine here. See the state table in charger_state_t / led_error_code_t
// for what each pattern means.
static void led_tick_cb(void *arg) {
    static uint32_t tick = 0;
    tick++;

    uint32_t duty = 0;
    switch (charger_state) {
        case CHARGER_STATE_INITIALIZING:
            // fast blink (200ms/200ms): booted, no command received yet
            duty = ((tick % 8) < 4) ? LED_DUTY_ON : 0;
            break;
        case CHARGER_STATE_OFF:
            // heartbeat: brief pulse every 2s - alive and idle
            duty = ((tick % 40) < 2) ? LED_DUTY_ON : 0;
            break;
        case CHARGER_STATE_WAIT_CAR: {
            // slow breathing triangle wave, ~3s period - ready, waiting for a vehicle
            const uint32_t period = 60;
            const uint32_t half = period / 2;
            const uint32_t phase = tick % period;
            const uint32_t level = (phase < half) ? phase : (period - phase);
            duty = (LED_DUTY_ON * level) / half;
            break;
        }
        case CHARGER_STATE_NEGOTIATE:
            // fast blink (100ms/100ms): negotiating charge current with the vehicle
            duty = ((tick % 4) < 2) ? LED_DUTY_ON : 0;
            break;
        case CHARGER_STATE_CHARGING:
            // solid on: actively charging
            duty = LED_DUTY_ON;
            break;
        case CHARGER_STATE_ERROR: {
            // blink out led_error_code as N short pulses (150ms/150ms), then a 1.2s pause, repeat
            const uint32_t pulse_ticks = 6;
            const uint32_t pause_ticks = 24;
            const uint32_t n = (led_error_code > 0) ? (uint32_t)led_error_code : 1;
            const uint32_t cycle = n * pulse_ticks + pause_ticks;
            const uint32_t pos = tick % cycle;
            if (pos < n * pulse_ticks) {
                duty = ((pos % pulse_ticks) < pulse_ticks / 2) ? LED_DUTY_ON : 0;
            }
            break;
        }
        default:
            break;
    }

    ledc_set_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel, duty);
    ledc_update_duty(ledc_channel[0].speed_mode, ledc_channel[0].channel);
}

static esp_adc_cal_characteristics_t adc1_chars;

static bool adc_calibration_init(void)
{
    esp_err_t ret;
    bool cali_enable = false;

    ret = esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Calibration scheme not supported, skip software calibration");
    } else if (ret == ESP_ERR_INVALID_VERSION) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else if (ret == ESP_OK) {
        cali_enable = true;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN, ADC_WIDTH, 0, &adc1_chars);
    } else {
        ESP_LOGE(TAG, "Invalid arg");
    }

    return cali_enable;
}

static void initOutputPort(gpio_num_t gpio_num) {
    ESP_ERROR_CHECK(gpio_reset_pin(gpio_num));
    ESP_ERROR_CHECK(gpio_set_direction(gpio_num, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(gpio_num, 0));
}

void app_main(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize WiFi and MQTT
    wifi_init();
    mqtt_app_start();

    // Configure the control pin
    initOutputPort(LED_PIN);

    // Configure solid-state relay control pins
    initOutputPort(GPIO_SOLID_STATE_RELAY_12V);
    initOutputPort(GPIO_SOLID_STATE_RELAY_CHARGE);

    // Configure optocoupler control pins
    initOutputPort(GPIO_OPTOCOUPLER_12V_POS);
    initOutputPort(GPIO_OPTOCOUPLER_12V_NEG);

    // Configure ADC
    if (!adc_calibration_init()) {
        ESP_LOGW(TAG, "ADC calibration unavailable, using raw counts");
    }

    //ADC1 config
    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN));

    /*
     * Prepare and set configuration of timers
     * that will be used by LED Controller
     */
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 4000,                      // frequency of PWM signal
        .speed_mode = LEDC_LS_MODE,           // timer mode
        .timer_num = LEDC_LS_TIMER,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    // Set LED Controller with previously prepared configuration
    for (int ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
        ledc_channel_config(&ledc_channel[ch]);
    }

    // Start the status LED pattern driver - blinks/breathes/pulses according to charger_state
    const esp_timer_create_args_t led_timer_args = {
        .callback = &led_tick_cb,
        .name = "led_tick",
    };
    esp_timer_handle_t led_timer;
    ESP_ERROR_CHECK(esp_timer_create(&led_timer_args, &led_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(led_timer, LED_TICK_MS * 1000));

    // Register event handler for custom events
    esp_event_handler_instance_register(TASK_EVENTS, ESP_EVENT_ANY_ID, &task_event_handler, NULL, NULL);

    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000, // 1MHz, 1 tick = 1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    
    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_callback, // register user callback
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, NULL));

    // start with duty cycle 0 (off)
    set_timer_alarm_values(0.0f);
        
    gptimer_alarm_config_t alarm_config = {
        .reload_count = 0,
        .alarm_count = timer_alarm_values[timer_state],
        .flags.auto_reload_on_alarm = false,
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));

    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
}
