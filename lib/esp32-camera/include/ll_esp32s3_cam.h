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
    void dma_print_state();
    void dma_reset();

private:
    gdma_channel_handle_t dma_channel_handle;//ESP32-S3
};

#endif