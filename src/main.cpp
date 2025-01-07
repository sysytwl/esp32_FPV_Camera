//#include "EEPROM.h"
#include "esp_log.h"

#include "esp_event.h"
#include "esp_wifi.h"
#include <driver/adc.h>
#include "esp_adc_cal.h"
#include "esp_wifi_types.h"
#include "driver/sdmmc_host.h"
#include "driver/sdmmc_defs.h"
#include "sdmmc_cmd.h"
#include "esp_vfs_fat.h"
//#include "esp_wifi_internal.h"
#include "esp_heap_caps.h"
#include "esp_task_wdt.h"
#include "esp_private/wifi.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
//#include "bt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include <unistd.h>
#include <fcntl.h>
#include "esp_random.h"

#include "fec_codec.h"
#include "packets.h"
#include "safe_printf.h"
#include "structures.h"
#include "crc.h"
#include "driver/gpio.h"
#include "main.h"
#include "queue.h"
#include "circular_buffer.h"

//#include "ll_cam.h" // cam_obj_t defination, used in camera_data_available



#include "osd.h"
#include "msp.h"
#include "avi.h"

#include "vcd_profiler.h"

#include "jpeg_parser.h"
#include "util.h"

static int s_stats_last_tp = -10000;

#define MJPEG_PATTERN_SIZE 512 
/////////////////////////////////////////////////////////////////////////


#ifdef STATUS_LED_PIN && FLASH_LED_PIN
#include "leds.h"
#endif

#ifdef REC_BUTTON_PIN
#include "button.h"
#endif

#ifdef UART_MAVLINK
#include "mavlink.h"
#endif
/////////////////////////////////////////////////////////////////////////

static size_t s_video_frame_data_size = 0;
static uint32_t s_video_frame_index = 0;
static uint8_t s_video_part_index = 0;
static bool s_video_frame_started = false;
static size_t s_video_full_frame_size = 0;
static uint8_t s_osdUpdateCounter = 0;
static bool s_lastByte_ff = false;

static int s_actual_capture_fps = 0;
static int s_actual_capture_fps_expected = 0;

static int s_quality = 20;
static float s_quality_framesize_K1 = 0; //startup from minimum quality to decrease pressure
static float s_quality_framesize_K2 = 1;
static float s_quality_framesize_K3 = 1;
static int s_max_frame_size = 0;
static int s_sharpness = 20;



void applyAdaptiveQuality(){
    sensor_t* s = esp_camera_sensor_get(); 
    s->set_quality(s, s_quality); 
}



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
IRAM_ATTR size_t camera_data_available(void * cam_obj, const uint8_t* data, size_t count, bool last){ 
    size_t stride = ((cam_obj_t *)cam_obj)->dma_bytes_per_item;

    if(getOVFFlagAndReset()){//over flow, reduce size
        s_quality_framesize_K3 = 0.05;
        cam_ovf_count++;
        s_stats.video_frames_expected++;
        applyAdaptiveQuality();
    }

    if (data == nullptr){ //start frame

    }else{
#ifdef ESP32 //ESP32 - sample offset: 2, stride: 4
        const uint8_t* src = (const uint8_t*)data + 2;  // jump to the sample1 of DMA element
#elif ESP32S3
        //ESP32S3 - sample offset: 0, stride: 1
        const uint8_t* src = (const uint8_t*)data;  
#endif
        uint8_t* clrSrc = (uint8_t*)src;

        if (src[0] != 0xff || src[stride] != 0xd8){
            //broken frame data, no start marker
            //we probably missed the start of the frame. Have to skip it.
        }

#ifdef DVR_SUPPORT
                if (s_air_record){
                    add_to_sd_fast_buffer(clrSrc, 0, true);
                }
#endif

        count /= stride;

        //check if we missed last block due to VSYNK near the end of prev (last) block
        // if block starts with 16 zero bytes - it is either not filled at all (ov2640)
        //or is fully filled with zeros(ov5640)

#ifdef BOARD_XIAOS3SENSE
                //ov5640: check if 16 bytes at the end of the block are zero
                const uint32_t* pTail = (const uint32_t*)(&(src[count - 16]));
                if ( (pTail[0] == 0 ) && ( pTail[1] == 0 ) && ( pTail[2] == 0 ) && ( pTail[3] == 0 ) ){
                    //zeros at the end - block has to contain end marker
                    last = true;
                }
#endif

        if(last){//find the end marker for JPEG. Data after that can be discarded     edge case - 0xFF at the end of prev block, 0xD9 on the start of this
            if (s_lastByte_ff && (*src == 0xD9)){

                    count = 1;
                    
            }else{
                //search from the start of the block
                //tail of the block can contain end markers in garbage
                const uint8_t* dptr = src;
                const uint8_t* dptrEnd = src + (count - 2) * stride;
                while (dptr <= dptrEnd){
                    if (dptr[0] == 0xFF && dptr[stride] == 0xD9) {
                        count = (dptr - src) / stride + 2; //to include the 0xFFD9
                        if ((count & 0x1FF) == 0)
                            count += 1; 
                        if ((count % 100) == 0)
                            count += 1;
                        break;
                    }
                    dptr += stride;
                }
            }
                

            s_lastByte_ff = count > 0 ? src[count-1] == 0xFF : false;
            while (count > 0){
                //fill the buffer
                uint8_t* packet_data = s_fec_encoder.get_encode_packet_data(true);
                uint8_t* start_ptr = packet_data + sizeof(Air2Ground_Video_Packet) + s_video_frame_data_size;
                uint8_t* ptr = start_ptr;
                size_t c = std::min(MAX_VIDEO_DATA_PAYLOAD_SIZE - s_video_frame_data_size, count);

                count -= c;
                s_video_frame_data_size += c;
                s_video_full_frame_size += c;

#ifdef PROFILE_CAMERA_DATA    
                s_profiler.set(PF_CAMERA_DATA_SIZE, s_video_full_frame_size / 1024);
#endif

#ifdef BOARD_ESP32CAM
                //ESP32 - sample offset: 2, stride: 4
                size_t c8 = c >> 3;
                
                for (size_t i = c8; i > 0; i--)
                {
                    const uint8_t* src1  = src + 3*4;
                    uint32_t temp;

                    temp = *src1; 
                    src1 -= 4;

                    temp = (temp << 8) | *src1;
                    src1 -= 4;

                    temp = (temp << 8) | *src1;
                    src1 -= 4;

                    temp = (temp << 8) | *src1;

                    *(uint32_t*)ptr = temp;
                    ptr += 4;

                    src1  = src + 7*4;

                    temp = *src1;
                    src1 -= 4;

                    temp = (temp << 8) | *src1;
                    src1 -= 4;

                    temp = (temp << 8) | *src1;
                    src1 -= 4;

                    temp = (temp << 8) | *src1;
                    
                    src += 8*4;

                    *(uint32_t*)ptr = temp;
                    ptr+=4;

                    /*
                    *ptr++ = *src; src += stride;
                    *ptr++ = *src; src += stride;
                    *ptr++ = *src; src += stride;
                    *ptr++ = *src; src += stride;
                    *ptr++ = *src; src += stride;
                    *ptr++ = *src; src += stride;
                    *ptr++ = *src; src += stride;
                    *ptr++ = *src; src += stride;
                    */
                }
                for (size_t i = c - (c8 << 3); i > 0; i--)
                {
                    *ptr++ = *src; src += stride;
                }
#endif

#ifdef BOARD_XIAOS3SENSE
                //ESP32S3 - sample offset: 0, stride: 1
                memcpy( ptr, src, c);
                src += c;
#endif

                if (s_video_frame_data_size == MAX_VIDEO_DATA_PAYLOAD_SIZE){
                    //LOG("Flush: %d %d\n", s_video_frame_index, s_video_frame_data_size);
                    //if wifi send queue was overloaded, do not send frame data till the end of the frame. 
                    //Frame is lost anyway. 
                    //Let fec_encoder and wifi to send leftover and start with emtpy queues at the next camera frame.
                    if (!s_encoder_output_ovf_flag){ 
                        send_air2ground_video_packet(false);
                    }
                    s_video_frame_data_size = 0;
                    s_video_part_index++;
                }


#ifdef DVR_SUPPORT
                if (s_air_record){
#ifdef TEST_AVI_FRAMES
                    if ( s_video_full_frame_size == c )
                    {
                        s_framesCounter++;
                        start_ptr[14] = s_framesCounter & 0xff;
                        start_ptr[15] = s_framesCounter >> 8;
                    }
#endif            
                    add_to_sd_fast_buffer(start_ptr, c, false);
                }
#endif
            }


        if(last){//note: can occur multiple times during frame  ,   end of frame - send leftover
            send_air2ground_video_packet(true);

            //recalculateFrameSizeQualityK(s_video_full_frame_size);
            applyAdaptiveQuality();

#ifdef PROFILE_CAMERA_DATA    
                    s_profiler.set(PF_CAMERA_FRAME_QUALITY, s_quality);
#endif

            s_stats.fec_spin_count += s_fec_spin_count;
            s_fec_spin_count = 0;

#ifdef PROFILE_CAMERA_DATA    
                    s_profiler.set(PF_CAMERA_DATA_SIZE, 0);
#endif
            handle_ground2air_config_packetEx2(false);
            if (!g_osd.isLocked() && (g_osd.isChanged() || (s_osdUpdateCounter == 15))){
                send_air2ground_osd_packet();
                s_osdUpdateCounter = 0;
            }

#ifdef UART_MAVLINK
            send_air2ground_data_packet();
#endif

        }

        //zero start of the DMA block
        for(int i = 0; i < 16; i++){
            *clrSrc = 0;
            clrSrc += stride;
        }

        s_fec_encoder.unlock();
    }

    return count;
}



void setup(){
    //Initialize NVS
    ESP_ERROR_CHECK(nvs_flash_init());
    nvs_handle_t fpv_cam_config;
    esp_err_t err = nvs_open("fpv_cam_config", NVS_READWRITE, &fpv_cam_config);
    if (err != ESP_OK){// first boot
        nvs_set_i32(fpv_cam_config, "config", 10);
        nvs_set_blob(fpv_cam_config, "camera_config", &fpv_cam_config_class.camera_config, sizeof(fpv_cam_config_class.camera_config));
    } else {
        int32_t counter = 0;
        nvs_get_i32(fpv_cam_config, "counter", &counter);
        nvs_get_blob(fpv_cam_config, "camera_config", &fpv_cam_config_class.camera_config, NULL);
    }
    nvs_close(fpv_cam_config);

    //setup_wifi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(ESP_WIFI_MODE));
    ESP_ERROR_CHECK(esp_wifi_config_80211_tx_rate(ESP_WIFI_IF, WIFI_PHY_RATE_11M_L));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    //ESP_ERROR_CHECK(esp_wifi_set_bandwidth(ESP_WIFI_IF, WIFI_BW_HT20 ));
    ESP_ERROR_CHECK(esp_wifi_set_channel(13, WIFI_SECOND_CHAN_NONE));

    // wifi_promiscuous_filter_t filter = 
    // {
    //     .filter_mask = WIFI_PROMIS_FILTER_MASK_DATA
    // };
    // ESP_ERROR_CHECK(esp_wifi_set_promiscuous_filter(&filter));
    // ESP_ERROR_CHECK(esp_wifi_set_promiscuous_ctrl_filter(&filter));
    // ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(packet_received_cb));
    // ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));

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
   
        config.pixel_format = PIXFORMAT_JPEG;
        config.frame_size = FRAMESIZE_CIF;
        config.jpeg_quality = 63;  //start from lowest quality to decrease pressure at startup
        config.fb_count = 1;
        config.grab_mode = CAMERA_GRAB_LATEST;
        config.fb_location = CAMERA_FB_IN_DRAM;
        config.data_available_callback = camera_data_available;

    ESP_ERROR_CHECK(esp_camera_init(&config));


    //fec
    setup_fec(fec_k, fec_n, s_ground2air_config_packet.fec_codec_mtu, add_to_wlan_outgoing_queue,add_to_wlan_incoming_queue);
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

*/
