// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
/*
 * Example Use
 *
    static camera_config_t camera_example_config = {
        .pin_pwdn       = PIN_PWDN,
        .pin_reset      = PIN_RESET,
        .pin_xclk       = PIN_XCLK,
        .pin_sccb_sda   = PIN_SIOD,
        .pin_sccb_scl   = PIN_SIOC,
        .pin_d7         = PIN_D7,
        .pin_d6         = PIN_D6,
        .pin_d5         = PIN_D5,
        .pin_d4         = PIN_D4,
        .pin_d3         = PIN_D3,
        .pin_d2         = PIN_D2,
        .pin_d1         = PIN_D1,
        .pin_d0         = PIN_D0,
        .pin_vsync      = PIN_VSYNC,
        .pin_href       = PIN_HREF,
        .pin_pclk       = PIN_PCLK,

        .xclk_freq_hz   = 20000000,
        .ledc_timer     = LEDC_TIMER_0,
        .ledc_channel   = LEDC_CHANNEL_0,
        .pixel_format   = PIXFORMAT_JPEG,
        .frame_size     = FRAMESIZE_SVGA,
        .jpeg_quality   = 10,
        .fb_count       = 2,
        .grab_mode      = CAMERA_GRAB_WHEN_EMPTY
    };

    esp_err_t camera_example_init(){
        return esp_camera_init(&camera_example_config);
    }

    esp_err_t camera_example_capture(){
        //capture a frame
        camera_fb_t * fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Frame buffer could not be acquired");
            return ESP_FAIL;
        }

        //replace this with your own function
        display_image(fb->width, fb->height, fb->pixformat, fb->buf, fb->len);

        //return the frame buffer back to be reused
        esp_camera_fb_return(fb);

        return ESP_OK;
    }
*/

/* Modified by JackShenYt, removed sensors does not support jpeg mode, DMA call back to external function, default running on core 0*/

#pragma once

#include "esp_err.h"
#include "driver/ledc.h"
#include "sys/time.h"
#include "sdkconfig.h"

#include "sensor.h"

/**
 * @brief define for if chip supports camera
 */
#define ESP_CAMERA_SUPPORTED (CONFIG_IDF_TARGET_ESP32 | CONFIG_IDF_TARGET_ESP32S3)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Configuration structure for camera initialization
 */
typedef struct {
    int pin_pwdn;                   /*!< GPIO pin for camera power down line */
    int pin_reset;                  /*!< GPIO pin for camera reset line */
    int pin_xclk;                   /*!< GPIO pin for camera XCLK line */
    union {
        int pin_sccb_sda;           /*!< GPIO pin for camera SDA line */
        int pin_sscb_sda __attribute__((deprecated("please use pin_sccb_sda instead")));           /*!< GPIO pin for camera SDA line (legacy name) */
    };
    union {
        int pin_sccb_scl;           /*!< GPIO pin for camera SCL line */
        int pin_sscb_scl __attribute__((deprecated("please use pin_sccb_scl instead")));           /*!< GPIO pin for camera SCL line (legacy name) */
    };
    int pin_d7;                     /*!< GPIO pin for camera D7 line */
    int pin_d6;                     /*!< GPIO pin for camera D6 line */
    int pin_d5;                     /*!< GPIO pin for camera D5 line */
    int pin_d4;                     /*!< GPIO pin for camera D4 line */
    int pin_d3;                     /*!< GPIO pin for camera D3 line */
    int pin_d2;                     /*!< GPIO pin for camera D2 line */
    int pin_d1;                     /*!< GPIO pin for camera D1 line */
    int pin_d0;                     /*!< GPIO pin for camera D0 line */
    int pin_vsync;                  /*!< GPIO pin for camera VSYNC line */
    int pin_href;                   /*!< GPIO pin for camera HREF line */
    int pin_pclk;                   /*!< GPIO pin for camera PCLK line */

    int xclk_freq_hz;               /*!< Frequency of XCLK signal, in Hz. EXPERIMENTAL: Set to 16MHz on ESP32-S2 or ESP32-S3 to enable EDMA mode */

    ledc_timer_t ledc_timer;        /*!< LEDC timer to be used for generating XCLK  */
    ledc_channel_t ledc_channel;    /*!< LEDC channel to be used for generating XCLK  */

    framesize_t frame_size;         /*!< Size of the output image: FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA  */

    int jpeg_quality;               /*!< Quality of JPEG output. 0-63 lower means higher quality  */
    //size_t fb_count;                /*!< Number of frame buffers to be allocated. If more than one, then each frame will be acquired (double speed)  */

    int sccb_i2c_port;              /*!< If pin_sccb_sda is -1, use the already configured I2C bus by number */

    void (*data_available_callback)(void * cam_obj,const uint8_t* data, size_t count, bool last); // DMA call back function, Non blocking intterupt

} camera_config_t;

/**
 * @brief Data structure of camera frame buffer
 */
typedef struct {
    uint8_t * buf;              /*!< Pointer to the pixel data */
    size_t len;                 /*!< Length of the buffer in bytes */
    size_t width;               /*!< Width of the buffer in pixels */
    size_t height;              /*!< Height of the buffer in pixels */
    pixformat_t format;         /*!< Format of the pixel data */
    struct timeval timestamp;   /*!< Timestamp since boot of the first DMA buffer of the frame */
} camera_fb_t;


/*************************  WAS PRIVATE  *******************************/
typedef struct {
    sensor_t sensor;
    camera_fb_t fb;
} camera_state_t;

typedef struct {
    int (*detect)(int slv_addr, sensor_id_t *id);
    int (*init)(sensor_t *sensor);
} sensor_func_t;

static camera_state_t *s_state = NULL;
/************************************************************************/

#define ESP_ERR_CAMERA_BASE 0x20000
#define ESP_ERR_CAMERA_NOT_DETECTED             (ESP_ERR_CAMERA_BASE + 1)
#define ESP_ERR_CAMERA_FAILED_TO_SET_FRAME_SIZE (ESP_ERR_CAMERA_BASE + 2)
#define ESP_ERR_CAMERA_FAILED_TO_SET_OUT_FORMAT (ESP_ERR_CAMERA_BASE + 3)
#define ESP_ERR_CAMERA_NOT_SUPPORTED            (ESP_ERR_CAMERA_BASE + 4)

/**
 * @brief Initialize the camera driver
 *
 * @note call camera_probe before calling this function
 *
 * This function detects and configures camera over I2C interface,
 * allocates framebuffer and DMA buffers,
 * initializes parallel I2S input, and sets up DMA descriptors.
 *
 * Currently this function can only be called once and there is
 * no way to de-initialize this module.
 *
 * @param config  Camera configuration parameters
 *
 * @return ESP_OK on success
 */
esp_err_t esp_camera_init(const camera_config_t* config);

/**
 * @brief Deinitialize the camera driver
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if the driver hasn't been initialized yet
 */
esp_err_t esp_camera_deinit(void);

/**
 * @brief Get a pointer to the image sensor control structure
 *
 * @return pointer to the sensor
 */
sensor_t * esp_camera_sensor_get(void);

/**
 * @brief Save camera settings to non-volatile-storage (NVS)
 *
 * @param key   A unique nvs key name for the camera settings
 */
esp_err_t esp_camera_save_to_nvs(const char *key);

/**
 * @brief Load camera settings from non-volatile-storage (NVS)
 *
 * @param key   A unique nvs key name for the camera settings
 */
esp_err_t esp_camera_load_from_nvs(const char *key);

#ifdef __cplusplus
}
#endif
