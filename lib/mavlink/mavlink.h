

//constexpr size_t MAX_TELEMETRY_PAYLOAD_SIZE = AIR2GROUND_MTU - sizeof(Air2Ground_Data_Packet);
//constexpr size_t MAX_TELEMETRY_PAYLOAD_SIZE = 512;
constexpr size_t MAX_TELEMETRY_PAYLOAD_SIZE = 128;

static uint8_t s_mavlink_out_buffer[MAX_TELEMETRY_PAYLOAD_SIZE];
static int s_mavlinkOutBufferCount = 0;

#ifdef UART_MAVLINK
//=============================================================================================
//=============================================================================================
//this currently called every frame: 50...11 fps
//30 fps: 30 * 128 = 3840 bytes/sec or 38400 baud
//11 fps: 11 * 128 - 1408 = 14080 baud
//todo: increase max mavlink payload size to 1K. Packets are 1.4K anyway. With 128 bytes we can not push 115200 mavlink stream currently.
//also UART RX ring buffer of 512 can not handle 115200 at 11 fps
IRAM_ATTR void send_air2ground_data_packet()
{
    int avail = MAX_TELEMETRY_PAYLOAD_SIZE - s_mavlinkOutBufferCount;
    if ( avail > 0 )
    {
        size_t rs = 0;
        ESP_ERROR_CHECK( uart_get_buffered_data_len(UART_MAVLINK, &rs) );
        if ( rs > avail ) rs = avail;

        if ( rs > 0 )
        {
            int len = uart_read_bytes(UART_MAVLINK, &(s_mavlink_out_buffer[s_mavlinkOutBufferCount]),rs, 0);
            if ( len < 0 )
            {
                LOG("MAVLNK COM error\n");
            }
            else
            {
                s_mavlinkOutBufferCount += len;
            }
        }
    }

    if ( s_mavlinkOutBufferCount < MAX_TELEMETRY_PAYLOAD_SIZE ) return; //todo: or agregationtime

    uint8_t* packet_data = s_fec_encoder.get_encode_packet_data(true);
    if(!packet_data)
    {
        LOG("no data buf!\n");
        return;
    }

    Air2Ground_Data_Packet& packet = *(Air2Ground_Data_Packet*)packet_data;
    packet.type = Air2Ground_Header::Type::Telemetry;
    packet.size = s_mavlinkOutBufferCount + sizeof(Air2Ground_Data_Packet);
    packet.pong = s_ground2air_config_packet.ping;
    packet.version = PACKET_VERSION;
    packet.crc = 0;
    packet.crc = crc8(0, &packet, sizeof(Air2Ground_Data_Packet));

    memcpy( packet_data + sizeof(Air2Ground_Data_Packet), s_mavlink_out_buffer, s_mavlinkOutBufferCount );

    if (!s_fec_encoder.flush_encode_packet(true))
    {
        LOG("Fec codec busy\n");
        s_stats.wlan_error_count++;
#ifdef PROFILE_CAMERA_DATA
        s_profiler.toggle(PF_CAMERA_FEC_OVF);
#endif
    }
    else
    {
        s_stats.out_telemetry_data += s_mavlinkOutBufferCount;
        s_mavlinkOutBufferCount = 0;
    }
}
#endif
