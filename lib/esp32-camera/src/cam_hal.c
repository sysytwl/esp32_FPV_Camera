// Copyright 2010-2020 Espressif Systems (Shanghai) PTE LTD
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

#include <stdio.h>
#include <string.h>
#include <stdalign.h>
#include "esp_heap_caps.h"
#include "ll_cam.h"
#include "cam_hal.h"

#if (ESP_IDF_VERSION_MAJOR == 3) && (ESP_IDF_VERSION_MINOR == 3)
#include "rom/ets_sys.h"
#else
#include "esp_timer.h"
#if CONFIG_IDF_TARGET_ESP32
#include "esp32/rom/ets_sys.h"  // will be removed in idf v5.0
#elif CONFIG_IDF_TARGET_ESP32S2
#include "esp32s2/rom/ets_sys.h"
#elif CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/rom/ets_sys.h"
#endif
#endif // ESP_IDF_VERSION_MAJOR
#define ESP_CAMERA_ETS_PRINTF ets_printf

#if CONFIG_CAMERA_TASK_STACK_SIZE
#define CAM_TASK_STACK             CONFIG_CAMERA_TASK_STACK_SIZE
#else
#define CAM_TASK_STACK             (2*1024)
#endif

static const char *TAG = "cam_hal";
static cam_obj_t *cam_obj = NULL;

static bool s_ovf_flag = false;

static const uint32_t JPEG_SOI_MARKER = 0xFFD8FF;  // written in little-endian for esp32
static const uint16_t JPEG_EOI_MARKER = 0xD9FF;  // written in little-endian for esp32
uint8_t (*data_available_callback)(bool last);

static int cam_verify_jpeg_soi(const uint8_t *inbuf, uint32_t length){
    for (uint32_t i = 0; i < length; i++) {
        if (memcmp(&inbuf[i], &JPEG_SOI_MARKER, 3) == 0) {
            //ESP_LOGW(TAG, "SOI: %d", (int) i);
            return i;
        }
    }
    ESP_LOGW(TAG, "NO-SOI");
    return -1;
}

static int cam_verify_jpeg_eoi(const uint8_t *inbuf, uint32_t length){
    int offset = -1;
    uint8_t *dptr = (uint8_t *)inbuf + length - 2;
    while (dptr > inbuf) {
        if (memcmp(dptr, &JPEG_EOI_MARKER, 2) == 0) {
            offset = dptr - inbuf;
            //ESP_LOGW(TAG, "EOI: %d", length - (offset + 2));
            return offset;
        }
        dptr--;
    }
    return -1;
}

//Copy fram from DMA dma_buffer to fram dma_buffer
int cnt = 0;
//int frame_pos = 0;
cam_event_t cam_event = 0;

bool esp_cam_tick(uint8_t *data, bool *last){
    xQueueReceive(cam_obj->event_queue, (void *)&cam_event, portMAX_DELAY);
    switch (cam_obj->state) {
        case CAM_STATE_IDLE: {
            if (cam_event == CAM_VSYNC_EVENT) {
                //cam_obj->frames[*frame_pos].fb.timestamp.tv_sec = (uint64_t)esp_timer_get_time() / 1000000UL;
                cam_obj->state = CAM_STATE_READ_BUF;
                cnt = 0;
            }
            *last = false;
        }
        return false;

        case CAM_STATE_READ_BUF: {
            if (cam_event == CAM_IN_SUC_EOF_EVENT) {
                //uint8_t buf = data_available_callback(false);
                last = false;
                size_t data_len = ll_cam_memcpy(cam_obj, data, &cam_obj->dma_buffer[(cnt % cam_obj->dma_buffer_cnt) * cam_obj->dma_buffer_size], cam_obj->dma_buffer_size);
                cnt++;

                //Check for JPEG SOI in the first buffer. stop if not found
                if (cnt == 0 && cam_verify_jpeg_soi(data, data_len) != 0) { //missed the SOI marker, skip the frame
                    ll_cam_stop(cam_obj);
                    cam_obj->state = CAM_STATE_IDLE;
                    return false;
                }

            } else if (cam_event == CAM_VSYNC_EVENT) {//Stop DMA and get the data
                ll_cam_stop(cam_obj);

                if (cnt) {
                    //uint8_t buf = data_available_callback(true);
                    *last = true;
                    size_t data_len = ll_cam_memcpy(cam_obj, data, &cam_obj->dma_buffer[(cnt % cam_obj->dma_buffer_cnt) * cam_obj->dma_buffer_size], cam_obj->dma_buffer_size);
                    cnt++;

                    // find the end marker for JPEG. Data after that can be discarded edge case - 0xFF at the end of prev block, 0xD9 on the start of this
                    int offset_e = cam_verify_jpeg_eoi(data, data_len);
                    if (offset_e == 0) {
                        ESP_LOGW(TAG, "NO-EOI");
                        // adjust buffer length
                        //data_len = offset_e + sizeof(JPEG_EOI_MARKER);
                    }                        
                }
            }
            cam_obj->state = CAM_STATE_IDLE;
            cnt = 0;

            *last = true;
        }
        return true;
    }
    //vTaskDelete(NULL); // end of the task
    return false;
};

void IRAM_ATTR ll_cam_send_event(cam_obj_t *cam, cam_event_t cam_event, BaseType_t * HPTaskAwoken){
    if (xQueueSendFromISR(cam->event_queue, (void *)&cam_event, HPTaskAwoken) != pdTRUE) {
        ll_cam_stop(cam);
        cam->state = CAM_STATE_IDLE;
        ESP_CAMERA_ETS_PRINTF(DRAM_STR("cam_hal: EV-%s-OVF\r\n"), cam_event==CAM_IN_SUC_EOF_EVENT ? DRAM_STR("EOF") : DRAM_STR("VSYNC"));
    }
}

static lldesc_t * allocate_dma_descriptors(uint32_t count, uint16_t size, uint8_t *buffer){
    lldesc_t *dma = (lldesc_t *)heap_caps_malloc(count * sizeof(lldesc_t), MALLOC_CAP_DMA);
    if(dma == NULL){
        return dma;
    }

    for(int x = 0; x < count; x++){
        dma[x].size = size;
        dma[x].length = 0;
        dma[x].sosf = 0;
        dma[x].eof = 0;
        dma[x].owner = 1;
        dma[x].buf = (buffer + size * x);
        dma[x].empty = (uint32_t)&dma[(x + 1) % count];
    }
    return dma;
}

static esp_err_t cam_dma_config(const camera_config_t *config){
    cam_obj->dma_buffer_size = config->data_pack_size;
    cam_obj->dma_buffer_cnt = config->data_pack_num;

    ll_cam_dma_sizes(cam_obj);
    ESP_LOGI(TAG, "buffer_size: %d, buffer_cnt: %d, node_buffer_size: %d, node_cnt: %d",(int) cam_obj->dma_buffer_size, (int) cam_obj->dma_buffer_cnt, (int) cam_obj->dma_node_buffer_size, (int) cam_obj->dma_node_cnt);

    cam_obj->dma_buffer = NULL;
    cam_obj->dma = NULL;

    //uint8_t dma_align = 0;
    if (cam_obj->psram_mode) {//for s3 which the dma can access psram
        // dma_align = ll_cam_get_dma_align(cam_obj);
        // if (cam_obj->fb_size < cam_obj->recv_size) {
        //     fb_size = cam_obj->recv_size;
        // }
    }else{
        cam_obj->dma_buffer = (uint8_t *)heap_caps_malloc(cam_obj->dma_buffer_size * sizeof(uint8_t), MALLOC_CAP_DMA); // DMA malloc
        if(NULL == cam_obj->dma_buffer) {
            ESP_LOGE(TAG,"%s(%d): DMA buffer %d Byte malloc failed, the current largest free block:%d Byte", __FUNCTION__, __LINE__,
                     (int) cam_obj->dma_buffer_size, (int) heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
            return ESP_FAIL;
        }

        cam_obj->dma = allocate_dma_descriptors(cam_obj->dma_node_cnt, cam_obj->dma_node_buffer_size, cam_obj->dma_buffer); // DMA descriptors
        CAM_CHECK(cam_obj->dma != NULL, "dma malloc failed", ESP_FAIL);
    }

    return ESP_OK;
}

esp_err_t cam_init(const camera_config_t *config){
    CAM_CHECK(NULL != config, "config pointer is invalid", ESP_ERR_INVALID_ARG);

    esp_err_t ret = ESP_OK;
    cam_obj = (cam_obj_t *)heap_caps_calloc(1, sizeof(cam_obj_t), MALLOC_CAP_DMA);
    CAM_CHECK(NULL != cam_obj, "lcd_cam object malloc error", ESP_ERR_NO_MEM);
    
    //cam_obj->swap_data = 0; //?
    cam_obj->vsync_pin = config->pin_vsync;
    cam_obj->vsync_invert = true;

    ll_cam_set_pin(cam_obj, config);
    ret = ll_cam_config(cam_obj, config);
    CAM_CHECK_GOTO(ret == ESP_OK, "ll_cam initialize failed", err);

    ESP_LOGI(TAG, "cam init ok");
    return ESP_OK;

err:
    free(cam_obj);
    cam_obj = NULL;
    return ESP_FAIL;
}

esp_err_t cam_config(const camera_config_t *config, framesize_t frame_size, uint16_t sensor_pid){
    CAM_CHECK(NULL != config, "config pointer is invalid", ESP_ERR_INVALID_ARG);
    esp_err_t ret = ESP_OK;

#if CONFIG_IDF_TARGET_ESP32
    cam_obj->psram_mode = false;
#elif CONFIG_IDF_TARGET_ESP32S3
    cam_obj->psram_mode = (config->xclk_freq_hz == 16000000);
#endif

    cam_obj->width = resolution[frame_size].width;
    cam_obj->height = resolution[frame_size].height;
    cam_obj->in_bytes_per_pixel = 1;
    cam_obj->fb_bytes_per_pixel = 1;
    //cam_obj->recv_size = cam_obj->width * cam_obj->height * 5;
    ret = cam_dma_config(config);
    CAM_CHECK_GOTO(ret == ESP_OK, "cam_dma_config failed", err);

    size_t queue_size = cam_obj->dma_buffer_cnt - 1;
    if (queue_size == 0) {
        queue_size = 1;
    }
    cam_obj->event_queue = xQueueCreate(queue_size, sizeof(cam_event_t));
    CAM_CHECK_GOTO(cam_obj->event_queue != NULL, "event_queue create failed", err);

    ret = ll_cam_init_isr(cam_obj);
    CAM_CHECK_GOTO(ret == ESP_OK, "cam intr alloc failed", err);

    cam_obj->state = CAM_STATE_IDLE;
    xQueueReset(cam_obj->event_queue);
    //xTaskCreatePinnedToCore(cam_task, "cam_task", CAM_TASK_STACK, NULL, configMAX_PRIORITIES - 2, NULL, 0);

    data_available_callback=config->data_available_callback; //assign external call back, blocking

    ESP_LOGI(TAG, "cam config ok");
    return ESP_OK;

err:
    cam_deinit();
    return ESP_FAIL;
}

esp_err_t cam_deinit(void){
    if (!cam_obj) {
        return ESP_FAIL;
    }

    cam_stop();
    // if (cam_obj->task_handle) {
    //     vTaskDelete(cam_obj->task_handle);
    // }
    // if (cam_obj->event_queue) {
    //     vQueueDelete(cam_obj->event_queue);
    // }


    ll_cam_deinit(cam_obj);

    if (cam_obj->dma) {
        free(cam_obj->dma);
    }
    if (cam_obj->dma_buffer) {
        free(cam_obj->dma_buffer);
    }

    free(cam_obj);
    cam_obj = NULL;
    return ESP_OK;
}

void cam_stop(void){
    ll_cam_vsync_intr_enable(cam_obj, false);
    ll_cam_stop(cam_obj);
}

void cam_start(void){
    ll_cam_vsync_intr_enable(cam_obj, true);
}
