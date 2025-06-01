// IRAM (Instruction RAM): 0x40080000 - 0x400A0000
// DRAM (Data RAM): 0x3FFAE000 - 0x3FFFFFFF


// #include "esp_event.h"
// 
// #include <driver/adc.h>
// #include "esp_adc_cal.h"
// #include "esp_wifi_types.h"
// #include "driver/sdmmc_host.h"
// #include "driver/sdmmc_defs.h"
// #include "sdmmc_cmd.h"
// #include "esp_vfs_fat.h"

// #include "esp_heap_caps.h"
// #include "esp_task_wdt.h"
// #include "esp_private/wifi.h"
// #include "esp_task_wdt.h"
// #include "esp_timer.h"

// #include "freertos/FreeRTOS.h"
// #include "freertos/queue.h"
// #include "freertos/task.h"
// #include "freertos/semphr.h"
// #include "driver/uart.h"
// #include <unistd.h>
// #include <fcntl.h>
// #include "esp_random.h"

// #include "fec_codec.h"
// #include "packets.h"
// #include "safe_printf.h"
// #include "structures.h"
// #include "crc.h"
// #include "driver/gpio.h"
// #include "queue.h"
// #include "circular_buffer.h"

#include "main.h"
#include "esp_heap_caps.h"
#include "config.h"
#include "esp_camera.h"
#include "fec_buffer.h"
#include "fec.h"

//#include "osd.h"
//#include "msp.h"
//#include "avi.h"
//#include "vcd_profiler.h"

//#include "jpeg_parser.h"
//#include "util.h"

//static int s_stats_last_tp = -10000;

//#define MJPEG_PATTERN_SIZE 512 
/////////////////////////////////////////////////////////////////////////


#ifdef STATUS_LED_PIN
//#include "leds.h"
#endif

#ifdef REC_BUTTON_PIN
//#include "button.h"
#endif

#ifdef UART_MAVLINK
#include "mavlink.h"
#endif
/////////////////////////////////////////////////////////////////////////

//static size_t s_video_frame_data_size = 0;
//static uint32_t s_video_frame_index = 0;
//static uint8_t s_video_part_index = 0;
//static bool s_video_frame_started = false;
//static size_t s_video_full_frame_size = 0;
//static uint8_t s_osdUpdateCounter = 0;
//static bool s_lastByte_ff = false;

//static int s_actual_capture_fps = 0;
//static int s_actual_capture_fps_expected = 0;

//static int s_quality = 20;

//static int s_max_frame_size = 0;
//static int s_sharpness = 20;


ZFE_FEC fec;
FEC_Block_buffer fec_buf;


void applyAdaptiveQuality(){
    sensor_t* s = esp_camera_sensor_get(); 
    //s->set_quality(s, s_quality); 
}

#define coding_k 6
#define coding_n 8



void setup(){
    //Initialize NVS
    // ESP_ERROR_CHECK(nvs_flash_init());
    // nvs_handle_t fpv_cam_config;
    // esp_err_t err = nvs_open("fpv_cam_config", NVS_READWRITE, &fpv_cam_config);
    // if (err != ESP_OK){// first boot
    //     nvs_set_i32(fpv_cam_config, "config", 10);
    //     nvs_set_blob(fpv_cam_config, "camera_config", &fpv_cam_config_class.camera_config, sizeof(fpv_cam_config_class.camera_config));
    // } else {
    //     int32_t counter = 0;
    //     nvs_get_i32(fpv_cam_config, "counter", &counter);
    //     nvs_get_blob(fpv_cam_config, "camera_config", &fpv_cam_config_class.camera_config, NULL);
    // }
    // nvs_close(fpv_cam_config);

    fec_buf.init(1500, coding_k, 20, coding_n);

    // setup camera
    camera_config_t config;
        config.ledc_channel = LEDC_CHANNEL_0;
        config.ledc_timer = LEDC_TIMER_0;
        config.pin_d0 = Y2_GPIO_NUM;
        config.pin_d1 = Y3_GPIO_NUM;
        config.pin_d2 = Y4_GPIO_NUM;
        config.pin_d3 = Y5_GPIO_NUM;
        config.pin_d4 = Y6_GPIO_NUM;
        config.pin_d5 = Y7_GPIO_NUM;
        config.pin_d6 = Y8_GPIO_NUM;
        config.pin_d7 = Y9_GPIO_NUM;
        config.pin_xclk = XCLK_GPIO_NUM;
        config.pin_pclk = PCLK_GPIO_NUM;
        config.pin_vsync = VSYNC_GPIO_NUM;
        config.pin_href = HREF_GPIO_NUM;
        config.pin_sccb_sda = SIOD_GPIO_NUM;
        config.pin_sccb_scl = SIOC_GPIO_NUM;
        config.pin_pwdn = PWDN_GPIO_NUM;
        config.pin_reset = RESET_GPIO_NUM;
        config.xclk_freq_hz = CAM_XCLK;  
        config.frame_size = FRAMESIZE_CIF;
        config.jpeg_quality = 63;  //start from lowest quality to decrease pressure at startup
    ESP_ERROR_CHECK(esp_camera_init(&config));


    //FEC init
    fec.init_fec();
    fec.fec_free(&fec.fec_type);
    fec.fec_new(6, 8, &fec.fec_type);
}

size_t count = 1500;
void loop(){
    static bool last = false;
    if(esp_cam_tick(fec_buf.get_block_pointer(), &last)){//check for DMA/VSYNC IRS, add to the fec buffer
        //send the pack
        //wifi injection
        if(fec_buf.fec_buf_ready()){
            uint8_t* fec_dst_ptr = (uint8_t*)heap_caps_malloc(count * (coding_k-coding_n), MALLOC_CAP_8BIT); //need to be zero?
            for (int i = 0; i < coding_n - coding_k; i++){
                fec.fec_encode_block(&fec.fec_type, (const uint8_t**)fec_buf.get_block_pointers(), fec_dst_ptr + (count * i), BLOCK_NUMS + coding_k, i, count);
            };
        }
        fec_buf.block_index_add();
    }
}

/*
Air receive:
1) packet_received_cb()- called by Wifi library when wifi packet is received
  - feeds data to s_fec_decoder.decode_data(). No data type checking at all.

2) s_fec_decoder.decode_data()
 - allocates item in m_decoder.packet_pool
 - concatenates small packets until mtu size (because wifi packets may be broken to smaller parts by wifi layer)
 - enquees packets into m_decoder.packet_queue

3) decoder_task_proc()  
 - retrives item from m_decoder.packet_queue
 - inserts either in m_decoder.block_packets or m_decoder.block_fec_packets
 - inserts into s_wlan_incoming_queue either received or restored packets
 - signals s_wifi_rx_task

4) s_wifi_rx_task
  - parses packets: Ground2Air_Header::Type::Config, Ground2Air_Header::Type::Data
 

Air send:
1) esp cam tick 
 - true: send the pack, add to the fec buffer, 
 - false: pass
 
2) call fec if fec buffer num == coding_k

3) wifi_tx_proc
 - calls esp_wifi_80211_tx() 

TX packet structure:
|--------------------------
| WLAN_IEEE_HEADER = 24 bytes
|-------------------------
| FEC_HEADER = 4 bytes
|-------------------------
| Air2Ground_Header = 8 bytes
|-------------------------
| Air2Ground_Video_Packet = 16 bytes  |  Air2Ground_OSD_Packet = 16 bytes
|-------------------------
| JPEG data | OSD data | MAVLINK data | TELEMETRY data  (ADD zeros if not enough data)
*/
