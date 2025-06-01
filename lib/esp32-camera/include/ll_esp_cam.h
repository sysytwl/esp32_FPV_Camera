#pragma once

#include <stdint.h>

#include "sdkconfig.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define CAM_CHECK(a, str, ret) if (!(a)) {                                          \
        ESP_LOGE(TAG,"%s(%d): %s", __FUNCTION__, __LINE__, str);                    \
        return (ret);                                                               \
        }

#define CAM_CHECK_GOTO(a, str, lab) if (!(a)) {                                     \
        ESP_LOGE(TAG,"%s(%d): %s", __FUNCTION__, __LINE__, str);                    \
        goto lab;                                                                   \
        }

#define LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE  (4092)

class ll_esp_cam {
public:
    virtual bool ll_cam_stop(cam_obj_t *cam);
    virtual bool ll_cam_start(cam_obj_t *cam, int frame_pos);
    virtual esp_err_t ll_cam_config(cam_obj_t *cam, const camera_config_t *config);
    virtual esp_err_t ll_cam_deinit(cam_obj_t *cam);
    virtual void ll_cam_vsync_intr_enable(cam_obj_t *cam, bool en);
    virtual esp_err_t ll_cam_set_pin(cam_obj_t *cam, const camera_config_t *config);
    virtual esp_err_t ll_cam_init_isr(cam_obj_t *cam);
    virtual void ll_cam_do_vsync(cam_obj_t *cam);
    virtual uint8_t ll_cam_get_dma_align(cam_obj_t *cam);
    virtual bool ll_cam_dma_sizes(cam_obj_t *cam);
    virtual size_t ll_cam_memcpy(cam_obj_t *cam, uint8_t *out, const uint8_t *in, size_t len);

private:

    typedef enum {
        CAM_IN_SUC_EOF_EVENT = 0,
        CAM_VSYNC_EVENT
    } cam_event_t;

    typedef enum {
        CAM_STATE_IDLE = 0,
        CAM_STATE_READ_BUF = 1,
    } cam_state_t;


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

};
