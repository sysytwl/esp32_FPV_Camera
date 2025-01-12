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
#include "config.h"
#include "esp_camera.h"

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
static uint8_t s_video_part_index = 0;
//static bool s_video_frame_started = false;
//static size_t s_video_full_frame_size = 0;
//static uint8_t s_osdUpdateCounter = 0;
//static bool s_lastByte_ff = false;

//static int s_actual_capture_fps = 0;
//static int s_actual_capture_fps_expected = 0;

static int s_quality = 20;

//static int s_max_frame_size = 0;
//static int s_sharpness = 20;


ZFE_FEC fec;



void applyAdaptiveQuality(){
    sensor_t* s = esp_camera_sensor_get(); 
    s->set_quality(s, s_quality); 
}

#define coding_k 6
#define coding_n 8

/* 
'bool last' does not mark last JPEG block reliably.

ov2640: if VSYNC interrupt occurs near the end of the last block, we receive another 1Kb block of garbage.
Note that ov2460 sends up zero bytes after FF D9 till 16 byte boundary, then a garbage to the end of the block, 
so if we receive extra block, the end of proper 'last' block is filled by zeros after FF D9

ov5640: we always receive up to 3 more zero 1kb blocks (uncertain: can we receive +7 blocks if VSYNK interrupt occurs at the end of 4th block?)
The end of proper 'last' block is filled by zeros after FF D9 (up to 1 kb)

There are two problems:
1) As we may receive garbage block, searching for the end marker in garbage may lead to incorrect JPEG size calculation and inclusion of garbage bytes 
  (possibly with misleading JPEG markers) in the stream.
2) with 0v5640 we waste bandwidth for 1..3 1kb blocks, it's 10% video bandwidth wasted!

Proper solution - jpeg chunks parsing - is possible but waste of CPU resources, because jpeg has to be processed byte-by-byte.

We apply very fast workaround instead.

We always zero first 16 bytes in the DMA buffer after processing.

ov2640: instead of receiving garbage block, we receive block with 16 zeros at start. This way we know that the whole buffer has to be skipped; 
jpeg is terminated already in the previous block. We waste only few bytes at the end of the block, which are filled by zeros.

ov5640: we are able to finish frame at the first completely zero block,
and even on the block wich has 16 zeros at end
*/
IRAM_ATTR void camera_data_available(void * cam_obj, const uint8_t* data, size_t count, bool last){ 
    // if(getOVFFlagAndReset()){//over flow, reduce size
    //     s_quality_framesize_K3 = 0.05;
    //     cam_ovf_count++;
    //     s_stats.video_frames_expected++;
    //     applyAdaptiveQuality();
    // }


    //the amount of packets that need decoding is the smallest of:
    // 1. Block size (coding_k)
    // 2. Fec packets (coding_n - coding_k)
    uint8_t* fec_dst_ptr = new uint8_t[count * (coding_n - coding_k)];
    for (int i = 0; i < coding_n - coding_k; i++){
        fec.fec_encode_block(&fec.fec_type, data, fec_dst_ptr + (count * i), BLOCK_NUMS + coding_k, i, count);
    };

#ifdef DVR_SUPPORT
    if (s_air_record){
        add_to_sd_fast_buffer(clrSrc, 0, true);
#ifdef TEST_AVI_FRAMES
        if ( s_video_full_frame_size == c ){
            s_framesCounter++;
            start_ptr[14] = s_framesCounter & 0xff;
            start_ptr[15] = s_framesCounter >> 8;
        }
#endif    
    }
#endif

    if(last){//note: can occur multiple times during frame  ,   end of frame - send leftover
        //send_air2ground_video_packet(true);

        //recalculateFrameSizeQualityK(s_video_full_frame_size);
        applyAdaptiveQuality();

        //s_stats.fec_spin_count += s_fec_spin_count;
        //s_fec_spin_count = 0;

        //handle_ground2air_config_packetEx2(false);
        //if (!g_osd.isLocked() && (g_osd.isChanged() || (s_osdUpdateCounter == 15))){
        //    send_air2ground_osd_packet();
        //    s_osdUpdateCounter = 0;
        //}

#ifdef UART_MAVLINK
        send_air2ground_data_packet();
#endif

    } else {
        //send_air2ground_video_packet(false);
        s_video_part_index++;
    }

    //s_fec_encoder.unlock();
}



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
        config.data_available_callback = camera_data_available;

    ESP_ERROR_CHECK(esp_camera_init(&config));


    //FEC init
    //fec.init_fec();
    fec.fec_new(6, 8, &fec.fec_type);
}

void loop(){

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
1) camera_data_available callback from camera library
 - send_air2ground_video_packet() - passes Air2Ground_Video_Packet to s_fec_encode.
 - flush_encode_packet() - concatenates data until mtu size and passes to m_encoder.packet_queue
 
 2) encoder_task_proc()
  - gathers packets in m_encoder.block_packets
  - calls fec_encode(()
  - add_to_wlan_outgoing_queue() - places encoded packets into s_wlan_outgoing_queue

3) wifi_tx_proc
 - reads s_wlan_outgoing_queue
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
