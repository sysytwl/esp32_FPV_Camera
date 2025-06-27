/* Modified by JackShenYt*/

#pragma once

#include <stdint.h>

#include "driver/ledc.h"
#include "sdkconfig.h"
#include "esp_log.h"

#if __has_include("esp_private/periph_ctrl.h")
# include "esp_private/periph_ctrl.h"
#endif
#if __has_include("esp_private/gdma.h")
# include "esp_private/gdma.h"
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

    #define NO_CAMERA_LEDC_CHANNEL 0xFF

#define CAM_CHECK(a, str, ret) if (!(a)) {                                          \
        ESP_LOGE(TAG,"%s(%d): %s", __FUNCTION__, __LINE__, str);                    \
        return (ret);                                                               \
        }

#define LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE  (4092)

static const char *TAG = "esp32 ll_cam";

class ll_esp_cam {
public:
    /**
     * @brief Starts DMA buffer and DMA buffer EOF intr
     */
    virtual void ll_cam_start();
    virtual void ll_cam_stop();

    /**
     * @brief Configures the camera data and synchronization pins.
     *
     * This function sets the pin assignments for the camera interface, including the VSYNC, PCLK, and data pins (D0-D7).
     * It also allows specifying whether the VSYNC signal should be inverted.
     *
     * @param vsync_invert  Set to 1 to invert the VSYNC signal, 0 for normal polarity.
     * @param pin_vsync     GPIO number for the VSYNC signal.
     * @param pin_pclk      GPIO number for the PCLK (pixel clock) signal.
     * @param pin_d0        GPIO number for data line D0.
     * @param pin_d1        GPIO number for data line D1.
     * @param pin_d2        GPIO number for data line D2.
     * @param pin_d3        GPIO number for data line D3.
     * @param pin_d4        GPIO number for data line D4.
     * @param pin_d5        GPIO number for data line D5.
     * @param pin_d6        GPIO number for data line D6.
     * @param pin_d7        GPIO number for data line D7.
     * @return
     *     - ESP_OK on success
     *     - Appropriate error code otherwise
     */
    virtual esp_err_t ll_cam_set_pin(bool vsync_invert, int pin_vsync, int pin_pclk, int pin_d0, int pin_d1, int pin_d2, int pin_d3, int pin_d4, int pin_d5, int pin_d6, int pin_d7);

    /**
     * @brief Configures the I2S0 on esp32 / Configure cam interface on esp32s3
     */
    virtual void ll_cam_config();

    /**
     * @brief Configures the XCLK by using pwm.
     */
    virtual esp_err_t camera_enable_out_clock(int ledc_timer, int xclk_freq_hz, int ledc_channel, int pin_xclk) {
        esp_err_t err = xclk_timer_conf(ledc_timer, xclk_freq_hz);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ledc_timer_config failed, rc=%x", err);
            return err;
        }

        g_ledc_channel = (ledc_channel_t)ledc_channel;
        ledc_channel_config_t ch_conf;
        ch_conf.gpio_num = pin_xclk;
        ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
        ch_conf.channel = (ledc_channel_t)ledc_channel;
        ch_conf.intr_type = LEDC_INTR_DISABLE;
        ch_conf.timer_sel = (ledc_timer_t)ledc_timer;
        ch_conf.duty = 1;
        ch_conf.hpoint = 0;
        err = ledc_channel_config(&ch_conf);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ledc_channel_config failed, rc=%x", err);
            return err;
        }
        return ESP_OK;
    }

    virtual void camera_disable_out_clock() {
        if (g_ledc_channel != NO_CAMERA_LEDC_CHANNEL) {
            ledc_stop(LEDC_LOW_SPEED_MODE, g_ledc_channel, 0);
            g_ledc_channel = (ledc_channel_t)NO_CAMERA_LEDC_CHANNEL;
        }
    }

    // virtual esp_err_t ll_cam_deinit(cam_obj_t *cam);
    // virtual void ll_cam_vsync_intr_enable(cam_obj_t *cam, bool en);

    // virtual esp_err_t ll_cam_init_isr(cam_obj_t *cam);
    // virtual void ll_cam_do_vsync(cam_obj_t *cam);
    // virtual uint8_t ll_cam_get_dma_align(cam_obj_t *cam);
    // virtual bool ll_cam_dma_sizes(cam_obj_t *cam);
    // virtual size_t ll_cam_memcpy(cam_obj_t *cam, uint8_t *out, const uint8_t *in, size_t len);

private:
    ledc_channel_t g_ledc_channel = (ledc_channel_t)NO_CAMERA_LEDC_CHANNEL;

    virtual esp_err_t xclk_timer_conf(int ledc_timer, int xclk_freq_hz) {
        ledc_timer_config_t timer_conf;
        timer_conf.duty_resolution = LEDC_TIMER_1_BIT;
        timer_conf.freq_hz = xclk_freq_hz;
        timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 0)   
        timer_conf.deconfigure = false;
#endif
    
#if ESP_IDF_VERSION_MAJOR >= 4
        timer_conf.clk_cfg = LEDC_AUTO_CLK;
#endif
        timer_conf.timer_num = (ledc_timer_t)ledc_timer;
        esp_err_t err = ledc_timer_config(&timer_conf);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ledc_timer_config failed for freq %d, rc=%x", xclk_freq_hz, err);
        }
        return err;
    }

    typedef enum {
        CAM_IN_SUC_EOF_EVENT = 0,
        CAM_VSYNC_EVENT
    } cam_event_t;

    typedef enum {
        CAM_STATE_IDLE = 0,
        CAM_STATE_READ_BUF = 1,
    } cam_state_t;


    uint32_t dma_bytes_per_item;
    uint32_t dma_buffer_cnt;
    uint32_t dma_node_buffer_size;
    uint32_t dma_node_cnt;

    QueueHandle_t event_queue;
    //QueueHandle_t frame_buffer_queue;
    //TaskHandle_t task_handle;
    intr_handle_t cam_intr_handle;

    uint8_t dma_num;//ESP32-S3
    intr_handle_t dma_intr_handle;//ESP32-S3

    //uint32_t frame_cnt;
    //uint32_t recv_size;
    //bool swap_data;
    bool psram_mode;  //s3 DMA can access psram

    //for RGB/YUV modes
    uint16_t width;
    uint16_t height;

    uint8_t in_bytes_per_pixel;
    uint8_t fb_bytes_per_pixel;

    cam_state_t state;

};
