#pragma once

//#include "structures.h"
//#include "wifi.h"
#include <esp_wifi_types.h>

#define DEFAULT_WIFI_CHANNEL 13
#define FW_VERSION "0.4.2"
#define PACKET_VERSION 1

#pragma pack(push, 1) // exact fit - no padding


// typedef struct{
//     uint16_t width;
//     uint16_t height;
//     uint8_t FPS2640;
//     uint8_t FPS5640;
//     uint8_t highFPS2640;
//     uint8_t highFPS5640;
// } TVMode;

// TVMode vmodes[] = {
//     {320,240,60,60,60,60},//QVGA,   //320x240
//     {400,296,60,60,60,60},//CIF,    //400x296
//     {480,320,30,30,30,30},//HVGA,   //480x320
//     {640,480,30,30,40,50},//VGA,    //640x480
//     {640,360,30,30,40,50},//VGA16,    //640x360
//     {800,600,30,30,30,30},//SVGA,   //800x600
//     {800,456,30,30,40,50},//SVGA16,  //800x456
//     {1024,768,12,30,12,30},//XGA,    //1024x768
//     {1024,576,12,30,12,30},//XGA16,    //1024x576
//     {1280,960,12,30,12,30},//SXGA,   //1280x960
//     {1280,720,12,30,12,30},//HD,   //1280x720
//     {1600,1200,10,10,10,10}//UXGA   //1600x1200
// };

//https://www.geeksforgeeks.org/ieee-802-11-mac-frame/
//https://en.wikipedia.org/wiki/802.11_Frame_Types
//each byte shifted from lower bits
//08 = 00 version, 01 frame type, 0000 subtype
uint8_t WLAN_IEEE_HEADER_AIR2GROUND[]={
  0x08, 0x00,//frame control
  0x00, 0x00,//2-3: Duration
  0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0xFF,// 4-9: Destination address (broadcast)
  0x11, 0x22, 0x33, 0x44, 0x55, 0x66,// 10-15: Source address
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00,// 16-21: BSSID
  0x10, 0x86// 22-23: Sequence / fragment number
};

constexpr uint8_t WLAN_IEEE_HEADER_GROUND2AIR[]={
  0x08, 0x01, 0x00, 0x00,
  0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
  0x66, 0x55, 0x44, 0x33, 0x22, 0x11,
  0x66, 0x55, 0x44, 0x33, 0x22, 0x11,
  0x10, 0x86
};

constexpr size_t WLAN_IEEE_HEADER_SIZE = sizeof(WLAN_IEEE_HEADER_AIR2GROUND);
constexpr size_t WLAN_MAX_PACKET_SIZE = 1500;
constexpr size_t WLAN_MAX_PAYLOAD_SIZE = WLAN_MAX_PACKET_SIZE - WLAN_IEEE_HEADER_SIZE;

static_assert(WLAN_IEEE_HEADER_SIZE == 24, "");

static constexpr size_t AIR2GROUND_MTU = WLAN_MAX_PAYLOAD_SIZE - 6; //6 is the fec header size



constexpr size_t GROUND2AIR_DATA_MAX_SIZE = 64;

struct Ground2Air_Header{
    enum class Type : uint8_t{
        Telemetry,
        Config,
    };

    Type type = Type::Telemetry; 
    uint32_t size = 0;
    uint8_t crc = 0;
    uint8_t packet_version = PACKET_VERSION;
};

constexpr size_t  GROUND2AIR_DATA_MAX_PAYLOAD_SIZE = GROUND2AIR_DATA_MAX_SIZE - sizeof(Ground2Air_Header);

struct Ground2Air_Data_Packet : Ground2Air_Header{
    uint8_t payload[GROUND2AIR_DATA_MAX_PAYLOAD_SIZE];
};
static_assert(sizeof(Ground2Air_Data_Packet) <= GROUND2AIR_DATA_MAX_SIZE, "");

enum class Resolution : uint8_t{
    QVGA,   //320x240
    CIF,    //400x296
    HVGA,   //480x320
    VGA,    //640x480
    VGA16,    //640x360
    SVGA,   //800x600
    SVGA16,  //800x456
    XGA,    //1024x768
    XGA16,    //1024x576
    SXGA,   //1280x960
    HD,   //1280x720
    UXGA,   //1600x1200
    COUNT
};


struct Ground2Air_Config_Packet: Ground2Air_Header{
    uint8_t ping = 0; //used for latency measurement
    int8_t wifi_power = 20;//dBm
    wifi_phy_rate_t wifi_rate;
    uint8_t wifi_channel = DEFAULT_WIFI_CHANNEL;
    uint8_t fec_codec_k;
    uint8_t fec_codec_n;
    uint16_t fec_codec_mtu = AIR2GROUND_MTU;
    uint8_t air_record_btn = 0; //incremented each time button is pressed on gs
    uint8_t profile1_btn = 0; //incremented each time button is pressed on gs
    uint8_t profile2_btn = 0; //incremented each time button is pressed on gs
    uint16_t sessionId;  //assigned random number on boot on each side. Used to recognise that either GS or Air unit has rebooted

    //Description of some settings:
    //https://heyrick.eu/blog/index.php?diary=20210418&keitai=0
    struct Camera_config{
        Resolution resolution = Resolution::SVGA;
        uint8_t fps_limit = 60;
        uint8_t quality = 0;//0 - 63  0-auto
        int8_t brightness = 0;//-2 - 2
        int8_t contrast = 0;//-2 - 2
        int8_t saturation = 1;//-2 - 2
        int8_t sharpness = 0;//-2 - 3
        uint8_t denoise = 0;  //0..8, ov5640 only
        uint8_t special_effect = 0;//0 - 6
        bool awb = true;
        bool awb_gain = true;
        uint8_t wb_mode = 0;//0 - 4
        bool aec = true; //automatic exposure control
        bool aec2 = true; //enable aec DSP (better processing?)
        int8_t ae_level = 1;//-2 - 2, for aec=true
        uint16_t aec_value = 204;//0 - 1200 ISO, for aec=false
        bool agc = true;  //automatic gain control
        uint8_t agc_gain = 0;//30 - 6, for agc=false
        uint8_t gainceiling = 0;//0 - 6, for agc=true. 0=2x, 1=4x, 2=8x,3=16x,4=32x,5=64x,6=128x
        bool bpc = true;
        bool wpc = true;
        bool raw_gma = true;
        bool lenc = true;
        bool hmirror = false;
        bool vflip = false;
        bool dcw = true;
        bool ov2640HighFPS = false;
        bool ov5640HighFPS = false;
        bool ov5640NightMode = false;
    } camera;
};
static_assert(sizeof(Ground2Air_Config_Packet) <= GROUND2AIR_DATA_MAX_SIZE, "");






struct Air2Ground_Header{
    enum class Type : uint8_t{
        Video,
        Telemetry,
        OSD
    };

    Type type = Type::Video; 
    uint32_t size = 0;
    uint8_t pong = 0; //used for latency measurement
    uint8_t version; //PACKET_VERSION
    uint8_t crc = 0;
};

struct Air2Ground_Video_Packet : Air2Ground_Header{
    Resolution resolution;
    uint8_t part_index : 7;
    uint8_t last_part : 1;
    uint32_t frame_index = 0;
    //data follows
};
static_assert(sizeof(Air2Ground_Video_Packet) == 14, "");

#define OSD_COLS 53
#define OSD_COLS_H 7 //56 bits
#define OSD_ROWS 20
#define OSD_BUFFER_SIZE (OSD_ROWS*OSD_COLS + OSD_ROWS*OSD_COLS_H)

struct OSDBuffer{
    uint8_t screenLow[OSD_ROWS][OSD_COLS];
    uint8_t screenHigh[OSD_ROWS][OSD_COLS_H];
};

struct AirStats{
    uint8_t SDDetected : 1;
    uint8_t SDSlow : 1;
    uint8_t SDError : 1;
    uint8_t curr_wifi_rate :5; //WIFI_Rate

    uint8_t wifi_queue_min : 7;
    uint8_t air_record_state : 1;

    uint8_t wifi_queue_max;

    uint32_t SDFreeSpaceGB16 : 12;
    uint32_t SDTotalSpaceGB16 : 12;
    uint32_t curr_quality : 6;
    uint32_t wifi_ovf : 1;
    uint32_t isOV5640 : 1;

    uint16_t outPacketRate;
    uint16_t inPacketRate;
    uint16_t inRejectedPacketRate;
    uint8_t rssiDbm;
    uint8_t noiseFloorDbm;
    uint8_t captureFPS;
    uint8_t cam_ovf_count;
    uint16_t cam_frame_size_min; //bytes
    uint16_t cam_frame_size_max; //bytes
    uint16_t inMavlinkRate; //b/s
    uint16_t outMavlinkRate; //b/s
};

struct Air2Ground_OSD_Packet : Air2Ground_Header{
    AirStats stats;
    OSDBuffer buffer;
};

static_assert(sizeof(Air2Ground_OSD_Packet) <= AIR2GROUND_MTU, "");
