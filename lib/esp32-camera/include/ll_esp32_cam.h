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
#pragma once

#include <stdio.h>
#include <string.h>

#if CONFIG_IDF_TARGET_ESP32
    #include "esp_idf_version.h"
    #if ESP_IDF_VERSION_MAJOR >= 4
        #include "esp32/rom/lldesc.h"
    #else
        #include "rom/lldesc.h"
    #endif

    #include "soc/i2s_struct.h"
    #include "esp_idf_version.h"

    #include "driver/gpio.h"
    #include "driver/ledc.h"
    #include "esp_system.h"
    #define NO_CAMERA_LEDC_CHANNEL 0xFF

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

    #include "ll_esp_cam.h"
    #include "sensor.h"

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

class ll_esp32_cam : public ll_esp_cam {
public:

    esp_err_t xclk_timer_conf(int ledc_timer, int xclk_freq_hz) override {
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

    esp_err_t camera_enable_out_clock(int ledc_timer, int xclk_freq_hz, int ledc_channel, int pin_xclk) override {
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

    void camera_disable_out_clock() override {
        if (g_ledc_channel != NO_CAMERA_LEDC_CHANNEL) {
            ledc_stop(LEDC_LOW_SPEED_MODE, g_ledc_channel, 0);
            g_ledc_channel = (ledc_channel_t)NO_CAMERA_LEDC_CHANNEL;
        }
    }

    void ll_cam_config() override {
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
        I2S0.fifo_conf.rx_fifo_mod = _sampling_mode; //not necessary, as the _sampling_mode is NULL
        I2S0.fifo_conf.rx_fifo_mod_force_en = 1;

        I2S0.conf_chan.rx_chan_mod = 1;
        I2S0.sample_rate_conf.rx_bits_mod = 0;
        I2S0.timing.val = 0;
        I2S0.timing.rx_dsync_sw = 1;
    }

    esp_err_t ll_cam_set_sample_mode(pixformat_t pix_format, uint32_t xclk_freq_hz, uint16_t sensor_pid, uint8_t* in_bytes_per_pixel) {
        if (pix_format == PIXFORMAT_GRAYSCALE) {
            if (sensor_pid == OV3660_PID || sensor_pid == OV5640_PID || sensor_pid == NT99141_PID || sensor_pid == SC031GS_PID || sensor_pid == BF20A6_PID || sensor_pid == GC0308_PID) {
                if (xclk_freq_hz > 10000000) {
                    _sampling_mode = SM_0A00_0B00;
                    _dma_filter = ll_cam_dma_filter_yuyv_highspeed;
                } else {
                    _sampling_mode = SM_0A0B_0C0D;
                    _dma_filter = ll_cam_dma_filter_yuyv;
                }
                *in_bytes_per_pixel = 1;       // camera sends Y8
            } else {
                if (xclk_freq_hz > 10000000 && sensor_pid != OV7725_PID) {
                    _sampling_mode = SM_0A00_0B00;
                    _dma_filter = ll_cam_dma_filter_grayscale_highspeed;
                } else {
                    _sampling_mode = SM_0A0B_0C0D;
                    _dma_filter = ll_cam_dma_filter_grayscale;
                }
                *in_bytes_per_pixel = 2;       // camera sends YU/YV
            }
        } else if (pix_format == PIXFORMAT_YUV422 || pix_format == PIXFORMAT_RGB565) {
                if (xclk_freq_hz > 10000000 && sensor_pid != OV7725_PID) {
                    if (sensor_pid == OV7670_PID) {
                        _sampling_mode = SM_0A0B_0B0C;
                    } else {
                        _sampling_mode = SM_0A00_0B00;
                    }
                    _dma_filter = ll_cam_dma_filter_yuyv_highspeed;
                } else {
                    _sampling_mode = SM_0A0B_0C0D;
                    _dma_filter = ll_cam_dma_filter_yuyv;
                }
                *in_bytes_per_pixel = 2;       // camera sends YU/YV    // frame buffer stores YU/YV/RGB565
        } else if (pix_format == PIXFORMAT_JPEG) {
            *in_bytes_per_pixel = 1;
            _dma_filter = ll_cam_dma_filter_jpeg;
            _sampling_mode = SM_0A00_0B00;
        } else {
            ESP_LOGE(TAG, "Requested format is not supported");
            return ESP_ERR_NOT_SUPPORTED;
        }
        I2S0.fifo_conf.rx_fifo_mod = _sampling_mode;
        return ESP_OK;
    }

    esp_err_t cam_dma_config(uint32_t dma_node_buffer_size, uint32_t dma_node_cnt){
        _eof_isr_size = dma_node_buffer_size;

        dma_node_buffer_size /= ll_cam_bytes_per_sample(_sampling_mode);
        CAM_CHECK(dma_node_buffer_size >= LCD_CAM_DMA_NODE_BUFFER_MAX_SIZE, "dma_node_buffer_size is too large", ESP_ERR_INVALID_SIZE);
        
        _fifo_depth = dma_node_cnt;
        dma_node_cnt *= ll_cam_bytes_per_sample(_sampling_mode);

        uint32_t dma_buffer_size = dma_node_buffer_size * dma_node_cnt;
        CAM_CHECK(dma_buffer_size >= CONFIG_CAMERA_DMA_BUFFER_SIZE_MAX, "dma_buffer_size is too large", ESP_ERR_INVALID_SIZE);
        _dma_buffer = (uint8_t *)heap_caps_malloc(dma_buffer_size * sizeof(uint8_t), MALLOC_CAP_DMA); // DMA malloc
        if(NULL == _dma_buffer) {
            ESP_LOGE(TAG,"%s(%d): DMA buffer %d Byte malloc failed, the current largest free block:%d Byte", __FUNCTION__, __LINE__, (int) dma_buffer_size, (int) heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
            return ESP_FAIL;
        }

        _dma = allocate_dma_descriptors(dma_node_cnt, dma_node_buffer_size, _dma_buffer); // DMA descriptors
        CAM_CHECK(_dma != NULL, "dma malloc failed", ESP_FAIL);

        return ESP_OK;
    }

    static void IRAM_ATTR ll_cam_vsync_isr(void *arg){
        ets_delay_us(1);
        if (gpio_ll_get_level(&GPIO, _vsync_pin) == !cam->vsync_invert) {
            ll_cam_send_event(cam, CAM_VSYNC_EVENT, &HPTaskAwoken);
        }
    }

    static void IRAM_ATTR ll_cam_dma_isr(void *arg){
        if (I2S0.int_st.val == 0) {
            return;
        }
        I2S0.int_clr.val = I2S0.int_st.val;
        if (I2S0.int_st.in_suc_eof){
            ll_cam_send_event(cam, CAM_IN_SUC_EOF_EVENT, &HPTaskAwoken);
        }
    }

    esp_err_t ll_cam_deinit(){
        gpio_isr_handler_remove(_vsync_pin);

        if (_cam_intr_handle) {
            esp_intr_free(this->_cam_intr_handle);
            _cam_intr_handle = NULL;
        }
        gpio_uninstall_isr_service();
        return ESP_OK;
    }

    void ll_cam_start() override {//dma start
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

        I2S0.rx_eof_num = _eof_isr_size / sizeof(dma_elem_t); // eof irs trigger size
        I2S0.in_link.addr = ((uint32_t)&_dma[0]) & 0xfffff; //first descriptor address

        I2S0.in_link.start = 1;
        I2S0.conf.rx_start = 1;
    }

    void ll_cam_stop() override {//dma stop
        I2S0.conf.rx_start = 0;
        I2S_ISR_DISABLE(in_suc_eof);
        I2S0.in_link.stop = 1;
    }

    esp_err_t ll_cam_set_pin(bool vsync_invert, int pin_vsync, int pin_pclk, int pin_d0, int pin_d1, int pin_d2, int pin_d3, int pin_d4, int pin_d5, int pin_d6, int pin_d7) override {
        gpio_config_t io_conf = {0};
        io_conf.intr_type = vsync_invert ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE;
        io_conf.pin_bit_mask = 1ULL << pin_vsync;
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
        io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
        gpio_config(&io_conf);

        gpio_install_isr_service(ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM);
        gpio_isr_handler_add((gpio_num_t)pin_vsync, ll_cam_vsync_isr, this->_dma_buffer); // vsync isr
        gpio_intr_disable((gpio_num_t)pin_vsync);

        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[pin_pclk], PIN_FUNC_GPIO);
        gpio_set_direction((gpio_num_t)pin_pclk, GPIO_MODE_INPUT);
        gpio_set_pull_mode((gpio_num_t)pin_pclk, GPIO_FLOATING);
        gpio_matrix_in(pin_pclk, I2S0I_WS_IN_IDX, false);

        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[pin_vsync], PIN_FUNC_GPIO);
        gpio_set_direction((gpio_num_t)pin_vsync, GPIO_MODE_INPUT);
        gpio_set_pull_mode((gpio_num_t)pin_vsync, GPIO_FLOATING);
        gpio_matrix_in(pin_vsync, I2S0I_V_SYNC_IDX, false);

        //PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[pin_href], PIN_FUNC_GPIO);
        //gpio_set_direction((gpio_num_t)pin_href, GPIO_MODE_INPUT);
        //gpio_set_pull_mode((gpio_num_t)pin_href, GPIO_FLOATING);
        //gpio_matrix_in(pin_href, I2S0I_H_SYNC_IDX, false);
        gpio_matrix_in(0x38, I2S0I_H_ENABLE_IDX, false); // transmission_start = (I2Sn_H_SYNC == 1)&&(I2Sn_V_SYNC == 1)&&(I2Sn_H_ENABLE == 1)

        int data_pins[8] = {
            pin_d0, pin_d1, pin_d2, pin_d3, pin_d4, pin_d5, pin_d6, pin_d7,
        };
        for (int i = 0; i < 8; i++) {
            PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[data_pins[i]], PIN_FUNC_GPIO);
            gpio_set_direction((gpio_num_t)data_pins[i], GPIO_MODE_INPUT);
            gpio_set_pull_mode((gpio_num_t)data_pins[i], GPIO_FLOATING);
            gpio_matrix_in(data_pins[i], I2S0I_DATA_IN0_IDX + i, false);
        }

        return ESP_OK;
    }

    esp_err_t ll_cam_init_isr(){//init the dma interrupt
        return esp_intr_alloc(ETS_I2S0_INTR_SOURCE, ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM, ll_esp32_cam::ll_cam_dma_isr, this->_dma_buffer, &this->_cam_intr_handle);
    }

    void ll_cam_vsync_intr_enable(gpio_num_t vsync_pin, bool en){
        if (en) {
            gpio_intr_enable(vsync_pin);
        } else {
            gpio_intr_disable(vsync_pin);
        }
    }

    inline size_t IRAM_ATTR ll_cam_memcpy(uint8_t *out, const uint8_t *in, size_t len){
        return _dma_filter(out, in, len);
    }

private:
    ledc_channel_t g_ledc_channel = (ledc_channel_t)NO_CAMERA_LEDC_CHANNEL;

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
    static i2s_sampling_mode_t _sampling_mode;

    lldesc_t *_dma = NULL; //DMA descriptors
    uint8_t *_dma_buffer = NULL; //DMA buffer
    uint32_t _eof_isr_size = NULL; // size of the data to be transferred before EOF interrupt is triggered
    intr_handle_t _cam_intr_handle = NULL; // dma interrupt handle, use to free the interrupt

    size_t _fifo_cnt = 0; // FIFO output pointer, used to track the current position in the FIFO buffer
    size_t _fifo_depth = 0; // FIFO depth, used to track the number of elements in the FIFO buffer

    static size_t ll_cam_bytes_per_sample(i2s_sampling_mode_t mode){
        switch(mode) {
            case SM_0A00_0B00: return 4;
            case SM_0A0B_0B0C: return 4;
            case SM_0A0B_0C0D: return 2;
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

    static size_t IRAM_ATTR ll_cam_dma_filter_grayscale(uint8_t* dst, const uint8_t* src, size_t len){
        const dma_elem_t* dma_el = (const dma_elem_t*)src;
        size_t elements = len / sizeof(dma_elem_t);
        size_t end = elements / 4;
        for (size_t i = 0; i < end; ++i) {
            // manually unrolling 4 iterations of the loop here
            dst[0] = dma_el[0].sample1;
            dst[1] = dma_el[1].sample1;
            dst[2] = dma_el[2].sample1;
            dst[3] = dma_el[3].sample1;
            dma_el += 4;
            dst += 4;
        }
        return elements;
    }

    static size_t IRAM_ATTR ll_cam_dma_filter_grayscale_highspeed(uint8_t* dst, const uint8_t* src, size_t len){
        const dma_elem_t* dma_el = (const dma_elem_t*)src;
        size_t elements = len / sizeof(dma_elem_t);
        size_t end = elements / 8;
        for (size_t i = 0; i < end; ++i) {
            // manually unrolling 4 iterations of the loop here
            dst[0] = dma_el[0].sample1;
            dst[1] = dma_el[2].sample1;
            dst[2] = dma_el[4].sample1;
            dst[3] = dma_el[6].sample1;
            dma_el += 8;
            dst += 4;
        }
        // the final sample of a line in SM_0A0B_0B0C sampling mode needs special handling
        if ((elements & 0x7) != 0) {
            dst[0] = dma_el[0].sample1;
            dst[1] = dma_el[2].sample1;
            elements += 1;
        }
        return elements / 2;
    }

    static size_t IRAM_ATTR ll_cam_dma_filter_yuyv(uint8_t* dst, const uint8_t* src, size_t len){
        const dma_elem_t* dma_el = (const dma_elem_t*)src;
        size_t elements = len / sizeof(dma_elem_t);
        size_t end = elements / 4;
        for (size_t i = 0; i < end; ++i) {
            dst[0] = dma_el[0].sample1;//y0
            dst[1] = dma_el[0].sample2;//u
            dst[2] = dma_el[1].sample1;//y1
            dst[3] = dma_el[1].sample2;//v

            dst[4] = dma_el[2].sample1;//y0
            dst[5] = dma_el[2].sample2;//u
            dst[6] = dma_el[3].sample1;//y1
            dst[7] = dma_el[3].sample2;//v
            dma_el += 4;
            dst += 8;
        }
        return elements * 2;
    }

    static size_t IRAM_ATTR ll_cam_dma_filter_yuyv_highspeed(uint8_t* dst, const uint8_t* src, size_t len){
        const dma_elem_t* dma_el = (const dma_elem_t*)src;
        size_t elements = len / sizeof(dma_elem_t);
        size_t end = elements / 8;
        for (size_t i = 0; i < end; ++i) {
            dst[0] = dma_el[0].sample1;//y0
            dst[1] = dma_el[1].sample1;//u
            dst[2] = dma_el[2].sample1;//y1
            dst[3] = dma_el[3].sample1;//v

            dst[4] = dma_el[4].sample1;//y0
            dst[5] = dma_el[5].sample1;//u
            dst[6] = dma_el[6].sample1;//y1
            dst[7] = dma_el[7].sample1;//v
            dma_el += 8;
            dst += 8;
        }
        if ((elements & 0x7) != 0) {
            dst[0] = dma_el[0].sample1;//y0
            dst[1] = dma_el[1].sample1;//u
            dst[2] = dma_el[2].sample1;//y1
            dst[3] = dma_el[2].sample2;//v
            elements += 4;
        }
        return elements;
    }

    static lldesc_t* allocate_dma_descriptors(uint32_t count, uint16_t size, uint8_t *buffer){
        lldesc_t *dma = (lldesc_t *)heap_caps_malloc(count * sizeof(lldesc_t), MALLOC_CAP_DMA);
        if(dma == NULL){
            ESP_LOGE(TAG, "DMA descriptors %d Byte malloc failed, the current largest free block:%d Byte", (int)(count * sizeof(lldesc_t)), (int)heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
            return dma;
        }

        for(int x = 0; x < count; x++){
            dma[x].size = size;
            dma[x].length = 0;
            dma[x].sosf = 0;
            dma[x].eof = 0;
            dma[x].owner = 1;
            dma[x].buf = (buffer + size * x);
            dma[x].empty = (uint32_t)&dma[(x + 1) % count];// ring buffer
        }
        return dma;
    }

    typedef size_t (*dma_filter_t)(uint8_t* dst, const uint8_t* src, size_t len);
    dma_filter_t _dma_filter = ll_cam_dma_filter_jpeg;
};

#endif