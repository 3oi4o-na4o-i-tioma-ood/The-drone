#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "BLDC.h"

static const char *TAG = "BLDC";

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_FREQUENCY (4000)           // Frequency in Hertz. Set frequency at 4 kHz

void BLDC_create(BLDC *bldc, int gpio, int channel)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY, // Set output frequency at 4 kHz
        .clk_cfg = LEDC_AUTO_CLK};
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = channel,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = gpio,
        //.duty = 0, // Set duty to 50% -> 4096
        .hpoint = 0};

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    bldc->GPIO = gpio;
    bldc->throttle = 0;
    bldc->channel = channel;
}

void BLDC_set_throttle(BLDC *bldc, float throttle)
{
    if (throttle < 0 || throttle > 1)
    {
        ESP_LOGE(TAG, "Invalid value for throttle: %f. Expected a value between 0 and 1", throttle);
        return;
    }

    int duty = (1 << LEDC_DUTY_RES) * throttle;
    ESP_LOGI(TAG, "Duty: %d", duty);
    //ledc_set_duty(LEDC_MODE, bldc->channel, duty);
    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = bldc->channel,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = bldc->GPIO,
        .duty = duty, // Set duty to 50% -> 4096
        .hpoint = 0};

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    bldc->throttle = throttle;
}