#pragma once

#include "ll_esp_cam.h"

#if CONFIG_IDF_TARGET_ESP32S3
#include "esp32s3/rom/lldesc.h"

#if __has_include("esp_private/periph_ctrl.h")
# include "esp_private/periph_ctrl.h"
#endif
#if __has_include("esp_private/gdma.h")
# include "esp_private/gdma.h"
#endif

class CamESP32S3 : public ll_esp_cam {
public:
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
        gdma_channel_handle_t dma_channel_handle;//ESP32-S3


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

    void dma_print_state();
    void dma_reset();

private:
    gdma_channel_handle_t dma_channel_handle;//ESP32-S3
};

#endif