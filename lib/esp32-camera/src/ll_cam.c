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
#include "soc/i2s_struct.h"
#include "esp_idf_version.h"
#if (ESP_IDF_VERSION_MAJOR >= 4) && (ESP_IDF_VERSION_MINOR > 1)
#include "hal/gpio_ll.h"
#else
#include "soc/gpio_periph.h"
#define esp_rom_delay_us ets_delay_us
static inline int gpio_ll_get_level(gpio_dev_t *hw, int gpio_num)
{
    if (gpio_num < 32) {
        return (hw->in >> gpio_num) & 0x1;
    } else {
        return (hw->in1.data >> (gpio_num - 32)) & 0x1;
    }
}
#endif
#include "ll_cam.h"
#include "xclk.h"
#include "cam_hal.h"

#if (ESP_IDF_VERSION_MAJOR >= 4) && (ESP_IDF_VERSION_MINOR >= 3)
#include "esp_rom_gpio.h"
#endif

#if (ESP_IDF_VERSION_MAJOR >= 5)
#define GPIO_PIN_INTR_POSEDGE GPIO_INTR_POSEDGE
#define GPIO_PIN_INTR_NEGEDGE GPIO_INTR_NEGEDGE
#define gpio_matrix_in(a,b,c) esp_rom_gpio_connect_in_signal(a,b,c)
#endif

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 2) 
#define ets_delay_us esp_rom_delay_us
#endif

static const char *TAG = "esp32 ll_cam";

#define I2S_ISR_ENABLE(i) {I2S0.int_clr.i = 1;I2S0.int_ena.i = 1;}
#define I2S_ISR_DISABLE(i) {I2S0.int_ena.i = 0;I2S0.int_clr.i = 1;}

typedef union {
    struct {
        uint32_t sample2:8;
        uint32_t unused2:8;
        uint32_t sample1:8;
        uint32_t unused1:8;
    };
    uint32_t val;
} dma_elem_t;

typedef enum {
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s2 00 s3, 00 s3 00 s4, ...
     */
    SM_0A0B_0B0C = 0,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 s2, 00 s3 00 s4, ...
     */
    SM_0A0B_0C0D = 1,
    /* camera sends byte sequence: s1, s2, s3, s4, ...
     * fifo receives: 00 s1 00 00, 00 s2 00 00, 00 s3 00 00, ...
     */
    SM_0A00_0B00 = 3,
} i2s_sampling_mode_t;
static i2s_sampling_mode_t sampling_mode = SM_0A00_0B00;

static size_t ll_cam_bytes_per_sample(i2s_sampling_mode_t mode){
    switch(mode) {
    case SM_0A00_0B00:
        return 4;
    case SM_0A0B_0B0C:
        return 4;
    case SM_0A0B_0C0D:
        return 2;
    default:
        assert(0 && "invalid sampling mode");
        return 0;
    }
}

static size_t IRAM_ATTR ll_cam_dma_filter_jpeg(uint8_t* dst, const uint8_t* src, size_t len){
    const dma_elem_t* dma_el = (const dma_elem_t*)src;
    size_t elements = len / sizeof(dma_elem_t);
    size_t end = elements / 4;
    // manually unrolling 4 iterations of the loop here
    for (size_t i = 0; i < end; ++i) {
        dst[0] = dma_el[0].sample1;
        dst[1] = dma_el[1].sample1;
        dst[2] = dma_el[2].sample1;
        dst[3] = dma_el[3].sample1;
        dma_el += 4;
        dst += 4;
    }
    return elements;
}

static void IRAM_ATTR ll_cam_vsync_isr(void *arg){
    cam_obj_t *cam = (cam_obj_t *)arg;

    // filter
    ets_delay_us(1);
    if (gpio_ll_get_level(&GPIO, cam->vsync_pin) == !cam->vsync_invert) {
        ll_cam_send_event(cam, CAM_VSYNC_EVENT, &HPTaskAwoken);
    }
}

static void IRAM_ATTR ll_cam_dma_isr(void *arg){
    cam_obj_t *cam = (cam_obj_t *)arg;

    typeof(I2S0.int_st) status = I2S0.int_st;
    if (status.val == 0) {
        return;
    }

    I2S0.int_clr.val = status.val;

    if (status.in_suc_eof) {
        ll_cam_send_event(cam, CAM_IN_SUC_EOF_EVENT, &HPTaskAwoken);
    }
}

bool IRAM_ATTR ll_cam_stop(cam_obj_t *cam){
    I2S0.conf.rx_start = 0;
    I2S_ISR_DISABLE(in_suc_eof);
    I2S0.in_link.stop = 1;
    return true;
}

esp_err_t ll_cam_deinit(cam_obj_t *cam){
    gpio_isr_handler_remove(cam->vsync_pin);

    if (cam->cam_intr_handle) {
        esp_intr_free(cam->cam_intr_handle);
        cam->cam_intr_handle = NULL;
    }
    gpio_uninstall_isr_service();
    return ESP_OK;
}

bool ll_cam_start(cam_obj_t *cam, int frame_pos){
    I2S0.conf.rx_start = 0;

    I2S_ISR_ENABLE(in_suc_eof);

    I2S0.conf.rx_reset = 1;
    I2S0.conf.rx_reset = 0;
    I2S0.conf.rx_fifo_reset = 1;
    I2S0.conf.rx_fifo_reset = 0;
    I2S0.lc_conf.in_rst = 1;
    I2S0.lc_conf.in_rst = 0;
    I2S0.lc_conf.ahbm_fifo_rst = 1;
    I2S0.lc_conf.ahbm_fifo_rst = 0;
    I2S0.lc_conf.ahbm_rst = 1;
    I2S0.lc_conf.ahbm_rst = 0;

    I2S0.rx_eof_num = cam->dma_half_buffer_size / sizeof(dma_elem_t);
    I2S0.in_link.addr = ((uint32_t)&cam->dma[0]) & 0xfffff;

    I2S0.in_link.start = 1;
    I2S0.conf.rx_start = 1;
    return true;
}

esp_err_t ll_cam_config(cam_obj_t *cam, const camera_config_t *config){
    // Enable and configure I2S peripheral
    periph_module_enable(PERIPH_I2S0_MODULE);

    I2S0.conf.rx_reset = 1;
    I2S0.conf.rx_reset = 0;
    I2S0.conf.rx_fifo_reset = 1;
    I2S0.conf.rx_fifo_reset = 0;
    I2S0.lc_conf.in_rst = 1;
    I2S0.lc_conf.in_rst = 0;
    I2S0.lc_conf.ahbm_fifo_rst = 1;
    I2S0.lc_conf.ahbm_fifo_rst = 0;
    I2S0.lc_conf.ahbm_rst = 1;
    I2S0.lc_conf.ahbm_rst = 0;

    I2S0.conf.rx_slave_mod = 1;
    I2S0.conf.rx_right_first = 0;
    I2S0.conf.rx_msb_right = 0;
    I2S0.conf.rx_msb_shift = 0;
    I2S0.conf.rx_mono = 0;
    I2S0.conf.rx_short_sync = 0;

    I2S0.conf2.lcd_en = 1;
    I2S0.conf2.camera_en = 1;

    // Configure clock divider
    I2S0.clkm_conf.clkm_div_a = 0;
    I2S0.clkm_conf.clkm_div_b = 0;
    I2S0.clkm_conf.clkm_div_num = 2;

    I2S0.fifo_conf.dscr_en = 1;
    I2S0.fifo_conf.rx_fifo_mod = sampling_mode;
    I2S0.fifo_conf.rx_fifo_mod_force_en = 1;

    I2S0.conf_chan.rx_chan_mod = 1;
    I2S0.sample_rate_conf.rx_bits_mod = 0;
    I2S0.timing.val = 0;
    I2S0.timing.rx_dsync_sw = 1;

    return ESP_OK;
}

void ll_cam_vsync_intr_enable(cam_obj_t *cam, bool en){
    if (en) {
        gpio_intr_enable(cam->vsync_pin);
    } else {
        gpio_intr_disable(cam->vsync_pin);
    }
}

esp_err_t ll_cam_set_pin(cam_obj_t *cam, const camera_config_t *config){
    gpio_config_t io_conf = {0};
    io_conf.intr_type = cam->vsync_invert ? GPIO_PIN_INTR_NEGEDGE : GPIO_PIN_INTR_POSEDGE;
    io_conf.pin_bit_mask = 1ULL << config->pin_vsync;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 1;
    io_conf.pull_down_en = 0;
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(config->pin_vsync, ll_cam_vsync_isr, cam); // vsync isr
    gpio_intr_disable(config->pin_vsync);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin_pclk], PIN_FUNC_GPIO);
    gpio_set_direction(config->pin_pclk, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->pin_pclk, GPIO_FLOATING);
    gpio_matrix_in(config->pin_pclk, I2S0I_WS_IN_IDX, false);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin_vsync], PIN_FUNC_GPIO);
    gpio_set_direction(config->pin_vsync, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->pin_vsync, GPIO_FLOATING);
    gpio_matrix_in(config->pin_vsync, I2S0I_V_SYNC_IDX, false);

    PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[config->pin_href], PIN_FUNC_GPIO);
    gpio_set_direction(config->pin_href, GPIO_MODE_INPUT);
    gpio_set_pull_mode(config->pin_href, GPIO_FLOATING);
    gpio_matrix_in(config->pin_href, I2S0I_H_SYNC_IDX, false);
    gpio_matrix_in(0x38, I2S0I_H_ENABLE_IDX, false); // transmission_start = (I2Sn_H_SYNC == 1)&&(I2Sn_V_SYNC == 1)&&(I2Sn_H_ENABLE == 1)

    int data_pins[8] = {
        config->pin_d0, config->pin_d1, config->pin_d2, config->pin_d3, config->pin_d4, config->pin_d5, config->pin_d6, config->pin_d7,
    };
    for (int i = 0; i < 8; i++) {
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[data_pins[i]], PIN_FUNC_GPIO);
        gpio_set_direction(data_pins[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(data_pins[i], GPIO_FLOATING);
        gpio_matrix_in(data_pins[i], I2S0I_DATA_IN0_IDX + i, false);
    }

    return ESP_OK;
}

esp_err_t ll_cam_init_isr(cam_obj_t *cam){
    return esp_intr_alloc(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, ll_cam_dma_isr, cam, &cam->cam_intr_handle);
}

uint8_t ll_cam_get_dma_align(cam_obj_t *cam){
    return 0;
}

#define min(a, b) ((a)<(b) ? (a):(b))
bool ll_cam_dma_sizes(cam_obj_t *cam){
    cam->dma_bytes_per_item = ll_cam_bytes_per_sample(sampling_mode);

    cam->dma_half_buffer_cnt = 4;
    cam->dma_node_buffer_size =  min((cam->width) * (cam->dma_bytes_per_item), 2048);
    cam->dma_half_buffer_size = cam->dma_node_buffer_size * 2;

    cam->dma_buffer_size = cam->dma_half_buffer_cnt * cam->dma_half_buffer_size;

    return 1;
}

size_t IRAM_ATTR ll_cam_memcpy(cam_obj_t *cam, uint8_t *out, const uint8_t *in, size_t len){
    size_t r = ll_cam_dma_filter_jpeg(out, in, len);

    return r;
}
