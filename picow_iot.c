#include <stdio.h>
#include <string.h>
#include <time.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"

#include "lwip/pbuf.h"
#include "lwip/dns.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"

#include "FreeRTOS.h"
#include "task.h"

#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/structs/rosc.h"

#include "hardware/uart.h"
#include "pico/binary_info.h"

// MQTT parameters
#define WIFI_SSID "MrBeast"
#define WIFI_PASSWORD "111222333"
#define MQTT_SERVER_HOST "io.adafruit.com"
#define MQTT_CLIENT_ID "light _2024"
#define MQTT_PORT LWIP_IANA_PORT_MQTT
#define MQTT_USER "BASEL_H"
#define MQTT_PASS "placeholder"
#define DASHBOARDS "BASEL_H/dashboards/light-2024"

#define LED0 0
#define LED1 3
#define LED2 5

// Motion Sensor GPIO
#define MOTION_SENSOR_PIN 6

// Track Wi-Fi and mqtt connection status
bool wifi_connected = false;
bool mqtt_connected = false;

uint16_t fade = 100;
bool led0_on = false;
bool led1_on = false;
bool led2_on = false;
bool motion_on = false;
bool motion_detected = false;

/*-----------------------------------------------------------*/

void prvSetupHardware(void);
void wifi_task(void);
void second_core_code(void);
// Light strip
void on_pwm_wrap();
void lightStrip_Task(void *pvParameters);
// Mqtt
static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status);
void mqtt_sub_request_cb(void *arg, err_t err);
void dns_found(const char *name, const ip_addr_t *ipaddr, void *callback_arg);
static void mqtt_pub_start_cb(void *arg, const char *topic, u32_t tot_len);
static void mqtt_pub_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags);
static void subscribe_topics(mqtt_client_t *client, void *arg);
// FreeRTOS hooks
void vApplicationMallocFailedHook(void);
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName);
void vApplicationIdleHook(void);

/*-----------------------------------------------------------*/

void prvSetupHardware(void)
{
    stdio_init_all();
    // Initialize GPIO for LEDs
    gpio_init(LED0);
    gpio_set_dir(LED0, GPIO_OUT);
    gpio_init(LED1);
    gpio_set_dir(LED1, GPIO_OUT);
    gpio_init(LED2);
    gpio_set_dir(LED2, GPIO_OUT);

    // Initialize GPIO for Motion Sensor
    gpio_init(MOTION_SENSOR_PIN);
    gpio_set_dir(MOTION_SENSOR_PIN, GPIO_IN);
}

/*-----------------------------------------------------------*/

void wifi_task(void)
{
    // Connect to Wi-Fi
    cyw43_arch_init();
    cyw43_arch_enable_sta_mode();

    // Connect to the Wi-Fi network - loop until connected
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) != 0)
    {
        printf("Attempting to connect...\n");
    }

    // Print a success message once connected
    printf("Connected to Wi-Fi\n");
    wifi_connected = true;
}

/*-----------------------------------------------------------*/

// Motion Detection Task
void motion_sensor_task(void *pvParameters)
{
    while (true)
    {
        if (motion_on)
        {
            if (gpio_get(MOTION_SENSOR_PIN)) // If motion detected
            {
                gpio_put(LED0, 1); // Turn on LED0 as an example response
                gpio_put(LED1, 1); // Turn on LED1 as an example response
                gpio_put(LED2, 1); // Turn on LED2 as an example response
                led0_on = true;
                led1_on = true;
                led2_on = true;

                // Optionally, publish to MQTT or trigger another event
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}


typedef struct MQTT_CLIENT_T_
{
    ip_addr_t remote_addr;
    mqtt_client_t *mqtt_client;
    u32_t received;
    u32_t counter;
    u32_t reconnect;
} MQTT_CLIENT_T;

// Perform initialisation
static MQTT_CLIENT_T *mqtt_client_init(void)
{
    MQTT_CLIENT_T *state = calloc(1, sizeof(MQTT_CLIENT_T));
    if (!state)
    {
        printf("failed to allocate state\n");
        return NULL;
    }
    state->received = 0;
    state->mqtt_client = mqtt_client_new();
    return state;
}

void dns_found(const char *name, const ip_addr_t *ipaddr, void *callback_arg)
{
    MQTT_CLIENT_T *state = (MQTT_CLIENT_T *)callback_arg;
    printf("DNS query finished with resolved addr of %s.\n", ip4addr_ntoa(ipaddr));
    state->remote_addr = *ipaddr;
}

void run_dns_lookup(MQTT_CLIENT_T *state)
{
    printf("Running DNS query for %s.\n", MQTT_SERVER_HOST);

    cyw43_arch_lwip_begin();
    err_t err = dns_gethostbyname(MQTT_SERVER_HOST, &(state->remote_addr), dns_found, state);
    cyw43_arch_lwip_end();

    if (err == ERR_ARG)
    {
        printf("failed to start DNS query\n");
        return;
    }

    if (err == ERR_OK)
    {
        printf("no lookup needed");
        return;
    }

    while (state->remote_addr.addr == 0)
    {
        cyw43_arch_poll();
        sleep_ms(1);
    }
}

u32_t data_in = 0;
u8_t buffer[1025];
u8_t data_len = 0;

static void mqtt_pub_start_cb(void *arg, const char *topic, u32_t tot_len)
{
    printf("mqtt_pub_start_cb: topic %s\n", topic);

    if (tot_len > 1024)
    {
        printf("Message length exceeds buffer size, discarding");
    }
    else
    {
        data_in = tot_len;
        data_len = 0;
    }
}

static void mqtt_pub_data_cb(void *arg, const u8_t *data, u16_t len, u8_t flags)
{
    if (data_in > 0)
    {
        data_in -= len;
        memcpy(&buffer[data_len], data, len);
        data_len += len;

        if (data_in == 0)
        {
            buffer[data_len] = '\0'; // Ensure null termination for string operations
            printf("Message received: %s\n", buffer);

            // Check if the message is related to LED control
            if (strcmp((char *)buffer, "All_ON") == 0)
            {
                gpio_put(LED0, 1);
                gpio_put(LED1, 1);
                gpio_put(LED2, 1);
                led0_on = true;
                led1_on = true;
                led2_on = true;
            }
            else if (strcmp((char *)buffer, "All_OFF") == 0)
            {
                gpio_put(LED0, 0);
                gpio_put(LED1, 0);
                gpio_put(LED2, 0);
                led0_on = false;
                led1_on = false;
                led2_on = false;
            }
            else if (strcmp((char *)buffer, "LED0_ON") == 0)
            {
                gpio_put(LED0, 1);
                led0_on = true;
            }
            else if (strcmp((char *)buffer, "LED0_OFF") == 0)
            {
                gpio_put(LED0, 0);
                led0_on = false;
            }
            else if (strcmp((char *)buffer, "LED1_ON") == 0)
            {
                gpio_put(LED1, 1);
                led1_on = true;
            }
            else if (strcmp((char *)buffer, "LED1_OFF") == 0)
            {
                gpio_put(LED1, 0);
                led1_on = false;
            }
            else if (strcmp((char *)buffer, "LED2_ON") == 0)
            {
                gpio_put(LED2, 1);
                led2_on = true;
            }
            else if (strcmp((char *)buffer, "LED2_OFF") == 0)
            {
                gpio_put(LED2, 0);
                led2_on = false;
            }
            // Handle motion sensor control
            else if (strcmp((char *)buffer, "MOTION_ON") == 0)
            {
                motion_on = true;
                printf("Motion sensor turned ON\n");
            }
            else if (strcmp((char *)buffer, "MOTION_OFF") == 0)
            {
                motion_on = false;
                printf("Motion sensor turned OFF\n");
            }
            else
            {
                // Check if the message is an integer for fade level
                char *endptr;
                long int_value = strtol((char *)buffer, &endptr, 10);

                if (*endptr == '\0')
                {
                    // Successfully parsed as integer
                    fade = (uint16_t)int_value;
                    printf("Fade level updated to %d\n", fade);
                }
                else {
                    printf("Wierd message");
                }
            }
            // Reset buffer for next message
            data_len = 0;
        }
    }
}


// Subscribe to the topics after connecting
static void subscribe_topics(mqtt_client_t *client, void *arg)
{
    sleep_ms(100);
    mqtt_subscribe(client, "BASEL_H/feeds/all-leds", 0, mqtt_sub_request_cb, arg);
    sleep_ms(100);
    mqtt_subscribe(client, "BASEL_H/feeds/led0", 0, mqtt_sub_request_cb, arg);
    sleep_ms(100);
    mqtt_subscribe(client, "BASEL_H/feeds/led1", 0, mqtt_sub_request_cb, arg);
    sleep_ms(100);
    mqtt_subscribe(client, "BASEL_H/feeds/led2", 0, mqtt_sub_request_cb, arg);
    sleep_ms(100);
    mqtt_subscribe(client, "BASEL_H/feeds/light-brightness", 0, mqtt_sub_request_cb, arg);
    sleep_ms(100);
    mqtt_subscribe(client, "BASEL_H/feeds/motion-control", 0, mqtt_sub_request_cb, arg);  // Add this line
    sleep_ms(100);
}

static void mqtt_connection_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_t status)
{
    if (status != 0)
    {
        printf("Error during connection: err %d.\n", status);
    }
    else
    {
        printf("MQTT connected.\n");
        mqtt_connected = true;
    }
}

void mqtt_sub_request_cb(void *arg, err_t err)
{
    printf("mqtt_sub_request_cb: err %d\n", err);
}

err_t mqtt_connect(MQTT_CLIENT_T *state)
{
    struct mqtt_connect_client_info_t ci;
    err_t err;

    memset(&ci, 0, sizeof(ci));

    ci.client_id = MQTT_CLIENT_ID;
    ci.client_user = MQTT_USER;
    ci.client_pass = MQTT_PASS;
    ci.keep_alive = 60;
    ci.will_topic = DASHBOARDS;
    ci.will_msg = NULL;
    ci.will_retain = 0;
    ci.will_qos = 0;

    const struct mqtt_connect_client_info_t *client_info = &ci;

    err = mqtt_client_connect(state->mqtt_client, &(state->remote_addr), MQTT_PORT, mqtt_connection_cb, state, client_info);

    if (err == ERR_OK)
    {
        // Successfully connected, now subscribe to topics
        subscribe_topics(state->mqtt_client, state);
    }
    else
    {
        printf("mqtt_connect return %d\n", err);
    }

    return err;
}

void publishToMqtt(MQTT_CLIENT_T *state)
{
    state->mqtt_client = mqtt_client_new();
    state->counter = 0;

    if (!state->mqtt_client)
    {
        printf("Failed to create mqtt client\n");
        return;
    }

    if (mqtt_connect(state) == ERR_OK)
    {
        mqtt_set_inpub_callback(state->mqtt_client, mqtt_pub_start_cb, mqtt_pub_data_cb, 0);
        while (true)
        {
            cyw43_arch_poll();
            if (mqtt_client_is_connected(state->mqtt_client))
            {
                err_t err;
                cyw43_arch_lwip_begin();
                cyw43_arch_lwip_end();
                if (err != ERR_OK)
                {
                    printf("Publish err: %d\n", err);
                }
                else
                {
                    if (state->counter != 0)
                    {
                        printf("published %d\n", state->counter);
                    }
                    sleep_ms(10000); // 8-second delay
                    state->counter++;
                }
            }
        }
    }
}

void on_pwm_wrap() {
    // Clear interrupts for all LEDs
    pwm_clear_irq(pwm_gpio_to_slice_num(LED0));
    pwm_clear_irq(pwm_gpio_to_slice_num(LED1));
    pwm_clear_irq(pwm_gpio_to_slice_num(LED2));

    // Apply brightness control even if LEDs are off to ensure proper updates
    pwm_set_gpio_level(LED0, led0_on ? fade * fade : 0);
    pwm_set_gpio_level(LED1, led1_on ? fade * fade : 0);
    pwm_set_gpio_level(LED2, led2_on ? fade * fade : 0);
}


void light_Task(void *pvParameters) {
    // Initialize PWM for each LED
    gpio_set_function(LED0, GPIO_FUNC_PWM);
    gpio_set_function(LED1, GPIO_FUNC_PWM);
    gpio_set_function(LED2, GPIO_FUNC_PWM);

    uint slice_num0 = pwm_gpio_to_slice_num(LED0);
    uint slice_num1 = pwm_gpio_to_slice_num(LED1);
    uint slice_num2 = pwm_gpio_to_slice_num(LED2);

    // Clear and enable IRQ
    pwm_clear_irq(slice_num0);
    pwm_set_irq_enabled(slice_num0, true);

    pwm_clear_irq(slice_num1);
    pwm_set_irq_enabled(slice_num1, true);

    pwm_clear_irq(slice_num2);
    pwm_set_irq_enabled(slice_num2, true);

    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv(&config, 4.f);

    pwm_init(slice_num0, &config, true);
    pwm_init(slice_num1, &config, true);
    pwm_init(slice_num2, &config, true);

    // Main loop
    while (true) {
        // Update PWM level immediately in the main loop
    if (led0_on) {
        pwm_set_gpio_level(LED0, fade * fade);
    } else if (!led0_on) {
        pwm_set_gpio_level(LED0, 0);
    }
    
    if (led1_on) {
        pwm_set_gpio_level(LED1, fade * fade);
    } else if (!led1_on) {
        pwm_set_gpio_level(LED1, 0);
    }

    if (led2_on) {
        pwm_set_gpio_level(LED2, fade * fade);
    } else if (!led2_on) {
        pwm_set_gpio_level(LED2, 0);
    }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/*-----------------------------------------------------------*/
// FreeRTOS hooks
void vApplicationMallocFailedHook(void)
{
    configASSERT((volatile void *)NULL);
}

void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    (void)pcTaskName;
    (void)pxTask;
    configASSERT((volatile void *)NULL);
}

void vApplicationIdleHook(void)
{
    volatile size_t xFreeHeapSpace;
    xFreeHeapSpace = xPortGetFreeHeapSize();
    (void)xFreeHeapSpace;
}

/*-----------------------------------------------------------*/

void second_core_code(void)
{
    // Start Light task
    xTaskCreate(light_Task, "lightTask", 2048, NULL, 1, NULL);

    // Start the motion sensor task
    xTaskCreate(motion_sensor_task, "MotionTask", 1024, NULL, 2, NULL);

    // Start scheduler
    vTaskStartScheduler();
}


int main()
{

    // Initialize the hardware
    prvSetupHardware();
    gpio_put(LED0, 1);

    gpio_put(LED1, 1);

    gpio_put(LED2, 1);


    multicore_launch_core1(second_core_code);

    if (!wifi_connected)
    {
        wifi_task();
    }

    // Wait for Wi-Fi connection before proceeding
    while (!wifi_connected)
    {
        sleep_ms(100);
    }

    if (wifi_connected && !mqtt_connected)
    {
        MQTT_CLIENT_T *state = mqtt_client_init();

        run_dns_lookup(state);
        publishToMqtt(state);
        cyw43_arch_deinit();
    }

    // Code should never reach here
    while (1)
    {
        // Handle errors
    }
}
