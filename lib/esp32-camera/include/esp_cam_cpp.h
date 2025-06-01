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
/* Modified by JackShenYt, removed sensors does not support jpeg mode, DMA call back to external function, default running on core 0*/

#pragma once

#include "esp_err.h"
#include "driver/ledc.h"
#include "sys/time.h"
#include "sdkconfig.h"
#include "sensor.h"

#include "esp_heap_caps.h"

static const char *TAG = "cam";
#include <esp_log.h>

/**
 * @brief define for if chip supports camera
 */
#if CONFIG_IDF_TARGET_ESP32
    #include "ll_esp32_cam.h"
    ll_esp32_cam ll_cam;
#elif CONFIG_IDF_TARGET_ESP32S3
    #include "ll_esp32s3_cam.h"
    ll_esp32s3_cam ll_cam;
#else
#error "This chip does not support."
#endif

#define CAM_CHECK(a, str, ret) if (!(a)) {                                          \
        ESP_LOGE(TAG,"%s(%d): %s", __FUNCTION__, __LINE__, str);                    \
        return (ret);                                                               \
        }

#define CAM_CHECK_GOTO(a, str, lab) if (!(a)) {                                     \
        ESP_LOGE(TAG,"%s(%d): %s", __FUNCTION__, __LINE__, str);                    \
        goto lab;                                                                   \
        }

class esp_cam {
public:
    /**
     * @brief Configuration structure for camera initialization
     */
    typedef struct {

    } camera_config_t;

    typedef struct {
        uint32_t dma_bytes_per_item;
        uint32_t dma_buffer_size;
        uint32_t dma_buffer_cnt;
        uint32_t dma_node_buffer_size;
        uint32_t dma_node_cnt;

        //for JPEG mode
        lldesc_t *dma; //DMA descriptors
        uint8_t  *dma_buffer; //DMA buffer

        QueueHandle_t event_queue;
        //QueueHandle_t frame_buffer_queue;
        //TaskHandle_t task_handle;
        intr_handle_t cam_intr_handle;

        uint8_t dma_num;//ESP32-S3
        intr_handle_t dma_intr_handle;//ESP32-S3
    #if SOC_GDMA_SUPPORTED
        gdma_channel_handle_t dma_channel_handle;//ESP32-S3
    #endif

        //uint8_t jpeg_mode;
        uint8_t vsync_pin;
        uint8_t vsync_invert;
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
    } cam_obj_t;

    /**
     * @brief Data structure of camera frame buffer
     */
    typedef struct {
        uint8_t * buf;
        size_t len;
        size_t width;
        size_t height;
        pixformat_t format;
        struct timeval timestamp;
    } camera_fb_t;

    /**
     * @brief Initialize the camera driver
     */
    static esp_err_t init(int pin_pwdn, int pin_reset, int pin_xclk, int pin_sccb_sda, int pin_sccb_scl, int pin_d7, int pin_d6, int pin_d5, int pin_d4, int pin_d3, int pin_d2, int pin_d1, int pin_d0, int pin_vsync, int pin_href, int pin_pclk, int xclk_freq_hz, ledc_timer_t ledc_timer, ledc_channel_t ledc_channel, framesize_t frame_size, int jpeg_quality, int sccb_i2c_port, uint32_t data_pack_size, uint32_t data_pack_num){

        cam_obj_t *cam_obj = (cam_obj_t *)heap_caps_calloc(1, sizeof(cam_obj_t), MALLOC_CAP_DMA);
        CAM_CHECK(NULL != cam_obj, "lcd_cam object malloc error", ESP_ERR_NO_MEM);

        ll_cam.ll_cam_set_pin(true, pin_vsync, pin_pclk, pin_d0, pin_d1, pin_d2, pin_d3, pin_d4, pin_d5, pin_d6, pin_d7);
        ll_cam.ll_cam_config();

        camera_model_t camera_model = CAMERA_NONE;
        err = camera_probe(config, &camera_model);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Camera probe failed with error 0x%x(%s)", err, esp_err_to_name(err));
            goto fail;
        }

    framesize_t frame_size = (framesize_t) config->frame_size;
    if (frame_size > camera_sensor[camera_model].max_size) {
        ESP_LOGW(TAG, "The frame size exceeds the maximum for this sensor, it will be forced to the maximum possible value");
        frame_size = camera_sensor[camera_model].max_size;
    }

    err = cam_config(config, frame_size, s_state->sensor.id.PID);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera config failed with error 0x%x", err);
        goto fail;
    }

    s_state->sensor.status.framesize = frame_size;
    ESP_LOGD(TAG, "Setting frame size to %dx%d", resolution[frame_size].width, resolution[frame_size].height);
    if (s_state->sensor.set_framesize(&s_state->sensor, frame_size) != 0) {
        ESP_LOGE(TAG, "Failed to set frame size");
        err = ESP_ERR_CAMERA_FAILED_TO_SET_FRAME_SIZE;
        goto fail;
    }

    s_state->sensor.set_pixformat(&s_state->sensor, PIXFORMAT_JPEG);
    s_state->sensor.set_quality(&s_state->sensor, config->jpeg_quality);

    if (s_state->sensor.id.PID == OV2640_PID) {
        s_state->sensor.set_gainceiling(&s_state->sensor, GAINCEILING_2X);
        s_state->sensor.set_bpc(&s_state->sensor, false);
        s_state->sensor.set_wpc(&s_state->sensor, true);
        s_state->sensor.set_lenc(&s_state->sensor, true);
    }

    s_state->sensor.init_status(&s_state->sensor);

    cam_start();

    return ESP_OK;

fail:

free(cam_obj);

    esp_camera_deinit();
    return err;

    }

    /**
     * @brief Deinitialize the camera driver
     */
    static esp_err_t deinit();

    /**
     * @brief Get a pointer to the image sensor control structure
     */
    static sensor_t * sensor_get();

    /**
     * @brief Save camera settings to non-volatile-storage (NVS)
     */
    static esp_err_t save_to_nvs(const char *key);

    /**
     * @brief Load camera settings from non-volatile-storage (NVS)
     */
    static esp_err_t load_from_nvs(const char *key);

    /**
     * @brief ESP CAM TICK (DMA DATA PROCESSING)
     */
    static bool tick(uint8_t *data, bool *last);

    /**
     * @brief CAM JPEG IMG Quality auto change
     */
    int calculateAdaptiveQualityValue() {
        int quality1 = (int)(8 + (63-8) * (1 - _s_quality_framesize_K1 * _s_quality_framesize_K2 * _s_quality_framesize_K3));
        if (quality1 < 8) quality1 = 8;
        if (quality1 > 63) quality1 = 63;

        static const uint8_t recode[64-8] = {8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 11, 11, 12, 12, 13, 13, 14, 15, 15, 16, 17, 18, 19, 20, 20, 21, 23, 24, 25, 26, 27, 29, 30, 31, 33, 34, 36, 37, 39, 41, 42, 44, 46, 48, 50, 52, 54, 56, 58, 60, 63};

        return recode[quality1-8];
    }

private:
    typedef struct {
        sensor_t sensor;
        camera_fb_t fb;
    } camera_state_t;

    typedef struct {
        int (*detect)(int slv_addr, sensor_id_t *id);
        int (*init)(sensor_t *sensor);
    } sensor_func_t;

    static camera_state_t *s_state;

    float _s_quality_framesize_K1=0, _s_quality_framesize_K2=1, _s_quality_framesize_K3=1;

    static esp_err_t camera_probe(const camera_config_t *config, camera_model_t *out_camera_model){
        esp_err_t ret = ESP_OK;
        *out_camera_model = CAMERA_NONE;
        if (s_state != NULL) {
            return ESP_ERR_INVALID_STATE;
        }

        s_state = (camera_state_t *) calloc(sizeof(camera_state_t), 1);
        if (!s_state) {
            return ESP_ERR_NO_MEM;
        }

        if (pin_xclk >= 0) {
            ESP_LOGD(TAG, "Enabling XCLK output");
            CAMERA_ENABLE_OUT_CLOCK(config);
        }

        if (pin_sccb_sda != -1) {
            ESP_LOGD(TAG, "Initializing SCCB");
            ret = SCCB_Init(config->pin_sccb_sda, config->pin_sccb_scl);
        } else {
            ESP_LOGD(TAG, "Using existing I2C port");
            ret = SCCB_Use_Port(config->sccb_i2c_port);
        }

        if(ret != ESP_OK) {
            ESP_LOGE(TAG, "sccb init err");
            goto err;
        }

        if (config->pin_pwdn >= 0) {
            ESP_LOGD(TAG, "Resetting camera by power down line");
            gpio_config_t conf = { 0 };
            conf.pin_bit_mask = 1LL << config->pin_pwdn;
            conf.mode = GPIO_MODE_OUTPUT;
            gpio_config(&conf);

            // carefull, logic is inverted compared to reset pin
            gpio_set_level(config->pin_pwdn, 1);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            gpio_set_level(config->pin_pwdn, 0);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        } else if (config->pin_reset >= 0) {
            ESP_LOGD(TAG, "Resetting camera");
            gpio_config_t conf = { 0 };
            conf.pin_bit_mask = 1LL << config->pin_reset;
            conf.mode = GPIO_MODE_OUTPUT;
            gpio_config(&conf);

            gpio_set_level(config->pin_reset, 0);
            vTaskDelay(10 / portTICK_PERIOD_MS);
            gpio_set_level(config->pin_reset, 1);
            vTaskDelay(10 / portTICK_PERIOD_MS);
        } else {
            ESP_LOGE(TAG, "No Reset pin!");
        }

        ESP_LOGD(TAG, "Searching for camera address");
        vTaskDelay(10 / portTICK_PERIOD_MS);

        uint8_t slv_addr = SCCB_Probe();

        if (slv_addr == 0) {
            ret = ESP_ERR_NOT_FOUND;
            goto err;
        }

        ESP_LOGI(TAG, "Detected camera at address=0x%02x", slv_addr);
        s_state->sensor.slv_addr = slv_addr;
        s_state->sensor.xclk_freq_hz = config->xclk_freq_hz;

        /**
         * Read sensor ID and then initialize sensor
         * Attention: Some sensors have the same SCCB address. Therefore, several attempts may be made in the detection process
         */
        sensor_id_t *id = &s_state->sensor.id;
        for (size_t i = 0; i < sizeof(g_sensors) / sizeof(sensor_func_t); i++) {
            if (g_sensors[i].detect(slv_addr, id)) {
                camera_sensor_info_t *info = esp_camera_sensor_get_info(id);
                if (NULL != info) {
                    *out_camera_model = info->model;
                    ESP_LOGI(TAG, "Detected %s camera", info->name);
                    g_sensors[i].init(&s_state->sensor);
                    break;
                }
            }
        }

        if (CAMERA_NONE == *out_camera_model) { //If no supported sensors are detected
            ESP_LOGE(TAG, "Detected camera not supported.");
            ret = ESP_ERR_NOT_SUPPORTED;
            goto err;
        }

        ESP_LOGI(TAG, "Camera PID=0x%02x VER=0x%02x MIDL=0x%02x MIDH=0x%02x",
                id->PID, id->VER, id->MIDH, id->MIDL);

        ESP_LOGD(TAG, "Doing SW reset of sensor");
        vTaskDelay(10 / portTICK_PERIOD_MS);

        return s_state->sensor.reset(&s_state->sensor);
    err :
        CAMERA_DISABLE_OUT_CLOCK();
        return ret;
    }

};

// Static member definitions
inline esp_cam::camera_state_t* esp_cam::s_state = nullptr;
