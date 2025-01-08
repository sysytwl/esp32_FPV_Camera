#pragma once

#include "esp_wifi_types.h"
#include "esp_heap_caps.h"
#include "esp_task_wdt.h"
#include "esp_private/wifi.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "queue.h"

#include "packets.h"
#include "stdint.h"

//#define WIFI_AP
#if defined WIFI_AP
    #define ESP_WIFI_MODE WIFI_MODE_AP
    #define ESP_WIFI_IF WIFI_IF_AP
#else
    #define ESP_WIFI_MODE WIFI_MODE_STA
    #define ESP_WIFI_IF WIFI_IF_STA
#endif

extern SemaphoreHandle_t s_wlan_incoming_mux;
extern SemaphoreHandle_t s_wlan_outgoing_mux;
extern Ground2Air_Data_Packet s_ground2air_data_packet;
extern Ground2Air_Config_Packet s_ground2air_config_packet;
extern Ground2Air_Config_Packet s_ground2air_config_packet2;
extern TaskHandle_t s_wifi_tx_task;
extern TaskHandle_t s_wifi_rx_task;

constexpr size_t WLAN_INCOMING_BUFFER_SIZE = 1024;

//use as much memory as available
//leave ~5k for SD library to initialize correctly
#ifdef BOARD_XIAOS3SENSE
constexpr size_t WLAN_OUTGOING_BUFFER_SIZE = 101000;
#else
constexpr size_t WLAN_OUTGOING_BUFFER_SIZE = 95000;
#endif

void setup_wifi(WIFI_Rate wifi_rate,uint8_t chn,float power_dbm,void (*packet_received_cb)(void* buf, wifi_promiscuous_pkt_type_t type));

void set_ground2air_config_packet_handler(void (*handler)(Ground2Air_Config_Packet& src));
void set_ground2air_data_packet_handler(void (*handler)(Ground2Air_Data_Packet& src));
esp_err_t set_wifi_fixed_rate(WIFI_Rate value);
esp_err_t set_wlan_power_dBm(float dBm);
void setup_wifi_file_server(void);
uint8_t getMaxWlanOutgoingQueueUsage();
uint8_t getMinWlanOutgoingQueueUsageSeen();
uint8_t getMaxWlanOutgoingQueueUsageFrame();
uint8_t getMinWlanOutgoingQueueUsageFrame();


struct Stats
{
    uint32_t wlan_data_sent = 0;
    uint32_t wlan_data_received = 0;
    uint16_t wlan_error_count = 0;
    uint16_t fec_spin_count = 0;
    uint16_t wlan_received_packets_dropped = 0;
    uint32_t video_data = 0;
    uint16_t video_frames = 0;
    //video_frames count is not valid if camera overflow happened. 
    //We still need expected vmode capture fps
    //we estimate it as video_frames + ovf_count
    uint16_t video_frames_expected = 0; 
    uint32_t sd_data = 0;
    uint32_t sd_drops = 0;
    
    uint32_t out_telemetry_data = 0;
    uint32_t in_telemetry_data = 0;

    uint16_t outPacketCounter = 0;
    uint16_t inPacketCounter = 0;
    uint16_t inRejectedPacketCounter = 0;

    uint8_t rssiDbm = 0;
    uint8_t noiseFloorDbm = 0;

    uint16_t camera_frame_size_min;
    uint16_t camera_frame_size_max;
};

extern Stats s_stats;
extern Stats s_last_stats;
extern uint8_t s_wlan_outgoing_queue_usage;



//=============================================================================================
//=============================================================================================
IRAM_ATTR void packet_received_cb(void* buf, wifi_promiscuous_pkt_type_t type)
{
    if (type == WIFI_PKT_MGMT)
    {
        //LOG("management packet\n");
        s_stats.inRejectedPacketCounter++;
        return;
    }
    else if (type == WIFI_PKT_DATA)
    {
        //LOG("data packet\n");
    }
    else if (type == WIFI_PKT_MISC)
    {
        //LOG("misc packet\n");
        s_stats.inRejectedPacketCounter++;
        return;
    }

    wifi_promiscuous_pkt_t *pkt = reinterpret_cast<wifi_promiscuous_pkt_t*>(buf);

    uint16_t len = pkt->rx_ctrl.sig_len;
    //s_stats.wlan_data_received += len;
    //s_stats.wlan_data_sent += 1;

    if (len <= WLAN_IEEE_HEADER_SIZE)    {
        LOG("WLAN receive header error");
        s_stats.wlan_error_count++;
        return;
    }

    //LOG("Recv %d bytes\n", len);
    //LOG("Channel: %d\n", (int)pkt->rx_ctrl.channel);

    //uint8_t broadcast_mac[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
    //LOG("MAC: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    uint8_t *data = pkt->payload;
    if (memcmp(data + 10, WLAN_IEEE_HEADER_GROUND2AIR + 10, 6) != 0)
    {
        s_stats.inRejectedPacketCounter++;
        return;
    }

    data += WLAN_IEEE_HEADER_SIZE;
    len -= WLAN_IEEE_HEADER_SIZE; //skip the 802.11 header

    len -= 4; //the received length has 4 more bytes at the end for some reason.

    s_stats.inPacketCounter++;
    s_stats.rssiDbm = -pkt->rx_ctrl.rssi;
    s_stats.noiseFloorDbm = -pkt->rx_ctrl.noise_floor;

    size_t size = std::min<size_t>(len, WLAN_MAX_PAYLOAD_SIZE);

    s_fec_decoder.lock();
    if (!s_fec_decoder.decode_data(data, size, false))
        s_stats.wlan_received_packets_dropped++;
    s_fec_decoder.unlock();

    s_stats.wlan_data_received += len;
}

//=============================================================================================
//=============================================================================================
//process settings not related to camera sensor setup
IRAM_ATTR static void handle_ground2air_config_packetEx1(Ground2Air_Config_Packet& src)
{
    s_recv_ground2air_packet = true;

    int64_t t = esp_timer_get_time();
    int64_t dt = t - s_last_seen_config_packet;
    s_last_seen_config_packet = t;

    Ground2Air_Config_Packet& dst = s_ground2air_config_packet;

    if (dst.sessionId != src.sessionId)
    {
        dst.sessionId = src.sessionId;
        dst.air_record_btn = src.air_record_btn;
        dst.profile1_btn = src.profile1_btn;
        dst.profile2_btn = src.profile2_btn;
    }

    if (dst.wifi_rate != src.wifi_rate)
    {
        LOG("Wifi rate changed from %d to %d\n", (int)dst.wifi_rate, (int)src.wifi_rate);
        ESP_ERROR_CHECK(set_wifi_fixed_rate(src.wifi_rate));
    }
    if (dst.wifi_power != src.wifi_power)
    {
        LOG("Wifi power changed from %d to %d\n", (int)dst.wifi_power, (int)src.wifi_power);
        ESP_ERROR_CHECK(set_wlan_power_dBm(src.wifi_power));
    }
    if (dst.fec_codec_n != src.fec_codec_n)
    {
        LOG("FEC codec changed from %d/%d/%d to %d/%d/%d\n", (int)dst.fec_codec_k, (int)dst.fec_codec_n, (int)dst.fec_codec_mtu, (int)src.fec_codec_k, (int)src.fec_codec_n, (int)src.fec_codec_mtu);
        {
            s_fec_encoder.switch_n( src.fec_codec_n );
        }
    }
    if (dst.wifi_channel != src.wifi_channel)
    {
        LOG("Wifi channel changed from %d to %d\n", (int)dst.wifi_channel, (int)src.wifi_channel);
        nvs_args_set("channel", src.wifi_channel);
        ESP_ERROR_CHECK(esp_wifi_set_channel((int)src.wifi_channel, WIFI_SECOND_CHAN_NONE));
    }

    if (dst.camera.fps_limit != src.camera.fps_limit)
    {
        if (src.camera.fps_limit == 0)
            s_video_target_frame_dt = 0;
        else
            s_video_target_frame_dt = 1000000 / src.camera.fps_limit;
        LOG("Target FPS changed from %d to %d\n", (int)dst.camera.fps_limit, (int)src.camera.fps_limit);
    }

    if ( s_restart_time == 0 )
    {
        if ( dst.air_record_btn != src.air_record_btn )
        {
            s_air_record = !s_air_record;
            dst.air_record_btn = src.air_record_btn;
        }

#if defined(ENABLE_PROFILER)
        if ( dst.profile1_btn != src.profile1_btn )
        {
            if ( s_profiler.isActive())
            {
                LOG("Profiler stopped!\n");
                s_profiler.stop();
                s_profiler.save();
                s_profiler.clear();
            }
            else
            {
                LOG("Profiler started!\n");
                s_profiler.start(500);
            }
            dst.profile1_btn = src.profile2_btn;
        }

        if ( dst.profile2_btn != src.profile2_btn )
        {
            if ( s_profiler.isActive())
            {
                LOG("Profiler stopped!\n");
                s_profiler.stop();
                s_profiler.save();
                s_profiler.clear();
            }
            else
            {
                LOG("Profiler started!\n");
                s_profiler.start(3000);
            }
            dst.profile2_btn = src.profile2_btn;
        }
#endif
    }

    dst = src;
}

//=============================================================================================
//=============================================================================================
//process settings related to camera sensor setup
IRAM_ATTR void handle_ground2air_config_packetEx2(bool forceCameraSettings)
{
    Ground2Air_Config_Packet& src = s_ground2air_config_packet;
    Ground2Air_Config_Packet& dst = s_ground2air_config_packet2;

#ifdef SENSOR_OV5640
    //on ov5640, aec2 is not aec dsp but "night vision" mode which decimate framerate dynamically
    src.camera.aec2 = false;
#endif

    if (forceCameraSettings || 
        (dst.camera.resolution != src.camera.resolution) || 
        (src.camera.ov2640HighFPS != dst.camera.ov2640HighFPS ) || 
        (src.camera.ov5640HighFPS != dst.camera.ov5640HighFPS )
    )
    {
        s_shouldRestartRecording =  esp_timer_get_time() + 1000000;
        LOG("Camera resolution changed from %d to %d\n", (int)dst.camera.resolution, (int)src.camera.resolution);
        sensor_t* s = esp_camera_sensor_get();

#ifdef SENSOR_OV5640
        s->set_colorbar(s, src.camera.ov5640HighFPS?1:0);
#else
        if ( src.camera.ov2640HighFPS && 
            ((src.camera.resolution == Resolution::VGA) ||
            (src.camera.resolution == Resolution::VGA16) ||
            (src.camera.resolution == Resolution::SVGA16)) 
            )
        {
            s->set_xclk( s, LEDC_TIMER_0, 16 );
        }
        else
        {
            s->set_xclk( s, LEDC_TIMER_0, 12 );
        }
#endif

        switch (src.camera.resolution)
        {
            case Resolution::QVGA: s->set_framesize(s, FRAMESIZE_QVGA); break;
            case Resolution::CIF: s->set_framesize(s, FRAMESIZE_CIF); break;
            case Resolution::HVGA: s->set_framesize(s, FRAMESIZE_HVGA); break;
            case Resolution::VGA: s->set_framesize(s, FRAMESIZE_VGA); break;

            case Resolution::VGA16:
#ifdef SENSOR_OV5640
                s->set_framesize(s, FRAMESIZE_P_3MP); //640x360
#else
                s->set_res_raw(s, 1/*OV2640_MODE_SVGA*/,0,0,0, 0, 72, 800, 600-144, 800,600-144,false,false);   //800x456x29.5? fps
                
#endif
            break;

            case Resolution::SVGA: s->set_framesize(s, FRAMESIZE_SVGA); break;

            case Resolution::SVGA16:
#ifdef SENSOR_OV5640
                //s->set_res_raw(s, 0, 0, 2623, 1951, 32, 16, 2844, 1968, 800, 600, true, true);  //attempt for 800x600
                //s->set_res_raw(s, 0, 240, 2623, 1711, 32, 16, 2844, 1488, 800, 450, true, true); //attempt for 800x450

                //s->set_pll(s, false, 26, 1, 1, false, 3, true, 4);  - root2x and pre_div are swapped due to incompatible signatures!
                //s->set_pll(s, false, 25, 1, false, 1, 3, true, 4); 

                //waning: LOGxxx should be commented out in ov5640.c otherwise there will be stack overflow in camtask
                s->set_framesize(s, FRAMESIZE_P_HD); //800x456
#else
                //s->set_framesize(s, FRAMESIZE_P_HD);  800x448 13 fps
                s->set_res_raw(s, 1/*OV2640_MODE_SVGA*/,0,0,0, 0, 72, 800, 600-144, 800,600-144,false,false);   //800x456 13 fps
#endif
            break;

            case Resolution::XGA: s->set_framesize(s, FRAMESIZE_XGA); break; //1024x768

            case Resolution::XGA16:  //1024x576
#ifdef SENSOR_OV5640
                s->set_framesize(s, FRAMESIZE_P_FHD);
#else
                s->set_res_raw(s, 0/*OV2640_MODE_UXGA*/,0,0,0, 0, 150, 1600, 1200-300, 1024, 576, false, false);   //1024x576 13 fps
                
#endif
            break;
            case Resolution::SXGA: s->set_framesize(s, FRAMESIZE_SXGA); break;
            case Resolution::HD: s->set_framesize(s, FRAMESIZE_HD); break;
            case Resolution::UXGA: s->set_framesize(s, FRAMESIZE_UXGA); break;

            case Resolution::COUNT:
            break;

        }
    }

#define APPLY(n1, n2, type) \
    if (forceCameraSettings || (dst.camera.n1 != src.camera.n1)) \
    { \
        LOG("Camera " #n1 " from %d to %d\n", (int)dst.camera.n1, (int)src.camera.n1); \
        sensor_t* s = esp_camera_sensor_get(); \
        s->set_##n2(s, (type)src.camera.n1); \
    }

    if ( src.camera.quality != 0 )
    {
        APPLY(quality, quality, int);
        s_quality = src.camera.quality;
    }
    else
    {
        if ( forceCameraSettings )
        {
            sensor_t* s = esp_camera_sensor_get(); 
            s->set_quality(s, 20); 
        }
    }

    if ( dst.camera.agc != src.camera.agc)
    {
        //reapply agc gain if agc changed
        forceCameraSettings = true;
    }

    if ( dst.camera.aec != src.camera.aec)
    {
        //reapply aec value if aec changed
        forceCameraSettings = true;
    }

    APPLY(brightness, brightness, int);
    APPLY(contrast, contrast, int);
    APPLY(saturation, saturation, int);
    if ( s_quality < 50 )
    {
        APPLY(sharpness, sharpness, int);
        s_sharpness = src.camera.sharpness;
    }
    APPLY(denoise, denoise, int);
#ifdef SENSOR_OV5640
    //gainceiling for ov5640 is range 0...3ff
    if (forceCameraSettings || (dst.camera.gainceiling != src.camera.gainceiling)) 
    { 
        LOG("Camera gainceiling from %d to %d\n", (int)dst.camera.gainceiling, (int)src.camera.gainceiling); 
        sensor_t* s = esp_camera_sensor_get(); 
        //s->set_gainceiling(s, (gainceiling_t)(2 << src.camera.gainceiling));
        //do not limit gainceiling on OV5640. Contrary to OV2640, it does good images with large gain ceiling, without enormous noise in dark scenes.
        s->set_gainceiling(s, (gainceiling_t)(0x3ff));
    }
#else
    APPLY(gainceiling, gainceiling, gainceiling_t);
#endif
    APPLY(awb, whitebal, int);
    APPLY(awb_gain, awb_gain, int);
    APPLY(wb_mode, wb_mode, int);
    APPLY(agc, gain_ctrl, int);
    APPLY(agc_gain, agc_gain, int);
    APPLY(aec, exposure_ctrl, int);
    APPLY(aec_value, aec_value, int);
    APPLY(aec2, aec2, int);
    APPLY(ae_level, ae_level, int);
    APPLY(hmirror, hmirror, int);
    APPLY(vflip, vflip, int);
    APPLY(special_effect, special_effect, int);
    APPLY(dcw, dcw, int);
    APPLY(bpc, bpc, int);
    APPLY(wpc, wpc, int);
    APPLY(raw_gma, raw_gma, int);
    APPLY(lenc, lenc, int);
#undef APPLY

    dst = src;
}

//===========================================================================================
//===========================================================================================
IRAM_ATTR static void handle_ground2air_config_packet(Ground2Air_Config_Packet& src)
{
    //handle settings not related to camera sensor setup.
    //camera sensor settings are processed in camera_data_available() callback
    handle_ground2air_config_packetEx1(src);
}

//===========================================================================================
//===========================================================================================
IRAM_ATTR static void handle_ground2air_data_packet(Ground2Air_Data_Packet& src)
{
#ifdef UART_MAVLINK
    xSemaphoreTake(s_serial_mux, portMAX_DELAY);

    int s = src.size - sizeof(Ground2Air_Header);
    s_stats.in_telemetry_data += s;        

    size_t freeSize = 0;
    ESP_ERROR_CHECK( uart_get_tx_buffer_free_size(UART_MAVLINK, &freeSize) );

    if ( freeSize >= s )
    {
        uart_write_bytes(UART_MAVLINK, ((uint8_t*)&src) + sizeof(Ground2Air_Header), s);
        //uart_write_bytes(UART_NUM_0, ((uint8_t*)&src) + sizeof(Ground2Air_Header), s);
    }

    xSemaphoreGive(s_serial_mux);
#endif
}



IRAM_ATTR void send_air2ground_osd_packet(){
    uint8_t* packet_data = s_fec_encoder.get_encode_packet_data(true);

    if(!packet_data){
        LOG("no data buf!\n");
        return ;
    }

    Air2Ground_OSD_Packet& packet = *(Air2Ground_OSD_Packet*)packet_data;
    packet.type = Air2Ground_Header::Type::OSD;
    packet.size = sizeof(Air2Ground_OSD_Packet);
    packet.pong = s_ground2air_config_packet.ping;
    packet.version = PACKET_VERSION;
    packet.crc = 0;

#ifdef DVR_SUPPORT
    packet.stats.SDDetected = s_sd_initialized ? 1: 0;
    packet.stats.SDSlow = s_last_stats.sd_drops ? 1: 0;
    packet.stats.SDError = SDError ? 1: 0;
#else
    packet.stats.SDDetected = 0;
    packet.stats.SDSlow = 0;
    packet.stats.SDError = 0;
#endif    
    packet.stats.curr_wifi_rate = (uint8_t)s_wlan_rate;

    packet.stats.wifi_queue_min = s_min_wlan_outgoing_queue_usage_seen;
    packet.stats.wifi_queue_max = s_max_wlan_outgoing_queue_usage;
    packet.stats.air_record_state = s_air_record ? 1 : 0;

    packet.stats.wifi_ovf = 0;
    if ( s_wifi_ovf_time > 0 )
    {
        int64_t t = esp_timer_get_time();
        t -= s_wifi_ovf_time;
        if ( t < 1000000 )
        {
            packet.stats.wifi_ovf = 1;
        }
        else
        {
            s_wifi_ovf_time = 0;
        }
    }
    
    packet.stats.SDFreeSpaceGB16 = SDFreeSpaceGB16;
    packet.stats.SDTotalSpaceGB16 = SDTotalSpaceGB16;
    packet.stats.curr_quality = s_quality;

#ifdef SENSOR_OV5640
    packet.stats.isOV5640 = 1;
#else
    packet.stats.isOV5640 = 0;
#endif    

    packet.stats.outPacketRate = s_last_stats.outPacketCounter;
    packet.stats.inPacketRate = s_last_stats.inPacketCounter;
    packet.stats.inRejectedPacketRate = s_last_stats.inRejectedPacketCounter;
    packet.stats.rssiDbm = s_last_stats.rssiDbm;
    packet.stats.noiseFloorDbm = s_last_stats.noiseFloorDbm;
    packet.stats.captureFPS = s_actual_capture_fps;
    packet.stats.cam_frame_size_min = s_last_stats.camera_frame_size_min;
    packet.stats.cam_frame_size_max = s_last_stats.camera_frame_size_max;
    packet.stats.cam_ovf_count = cam_ovf_count;
    packet.stats.inMavlinkRate = s_last_stats.in_telemetry_data;
    packet.stats.outMavlinkRate = s_last_stats.out_telemetry_data;

    memcpy( &packet.buffer, g_osd.getBuffer(), OSD_BUFFER_SIZE );
    
    packet.crc = crc8(0, &packet, sizeof(Air2Ground_OSD_Packet));

    if (!s_fec_encoder.flush_encode_packet(true))
    {
        LOG("Fec codec busy\n");
        s_stats.wlan_error_count++;
#ifdef PROFILE_CAMERA_DATA    
        s_profiler.toggle(PF_CAMERA_FEC_OVF);
#endif
    }
}

IRAM_ATTR void send_air2ground_video_packet(bool last){
    s_stats.video_data += s_video_frame_data_size;

    uint8_t* packet_data = s_fec_encoder.get_encode_packet_data(true);

    if(!packet_data){
        log_e("no data buf!\n");
    }

    Air2Ground_Video_Packet& packet = *(Air2Ground_Video_Packet*)packet_data;
    packet.type = Air2Ground_Header::Type::Video;
    packet.resolution = s_ground2air_config_packet.camera.resolution; /*NEED TO REMOVE*/
    packet.frame_index = s_video_frame_index; /*NEED TO CHANGE*/
    packet.part_index = s_video_part_index;
    packet.last_part = last ? 1 : 0;
    packet.size = s_video_frame_data_size + sizeof(Air2Ground_Video_Packet); /*NEED TO REMOVE*/
    packet.pong = s_ground2air_config_packet.ping; 
    packet.version = PACKET_VERSION; /*NEED TO REMOVE*/
    packet.crc = 0; /*NEED TO REMOVE*/
    packet.crc = crc8(0, &packet, sizeof(Air2Ground_Video_Packet)); /*NEED TO REMOVE*/
    if (!s_fec_encoder.flush_encode_packet(true)){
        log_i("Fec codec busy\n");
        s_stats.wlan_error_count++;
#ifdef PROFILE_CAMERA_DATA    
        s_profiler.toggle(PF_CAMERA_FEC_OVF);
#endif
    }
}



