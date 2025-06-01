#pragma once
static int64_t s_video_last_sent_tp = esp_timer_get_time();
static int64_t s_video_target_frame_dt = 0;
static uint8_t s_max_wlan_outgoing_queue_usage = 0;
static uint8_t s_min_wlan_outgoing_queue_usage_seen = 0;

static int64_t s_last_seen_config_packet = esp_timer_get_time();;

static int64_t s_restart_time = 0;
static int64_t s_wifi_ovf_time = 0;

extern WIFI_Rate s_wlan_rate;

static bool SDError = false;
static uint16_t SDTotalSpaceGB16 = 0;
static uint16_t SDFreeSpaceGB16 = 0;
static uint8_t cam_ovf_count = 0;

int32_t s_dbg;
uint16_t s_framesCounter = 0;

static bool s_initialized = false;

sdmmc_card_t* card = nullptr;

static bool s_air_record = false;

static uint64_t s_shouldRestartRecording;



#ifdef DVR_SUPPORT

SemaphoreHandle_t s_sd_fast_buffer_mux = xSemaphoreCreateBinary();
SemaphoreHandle_t s_sd_slow_buffer_mux = xSemaphoreCreateBinary();

////////////////////////////////////////////////////////////////////////////////////

static TaskHandle_t s_sd_write_task = nullptr;
static TaskHandle_t s_sd_enqueue_task = nullptr;
static bool s_sd_initialized = false;
static size_t s_sd_file_size = 0;
static uint32_t s_sd_next_session_id = 0;
static uint32_t s_sd_next_segment_id = 0;


//the fast buffer is RAM and used to transfer data quickly from the camera callback to the slow, SPIRAM buffer. 
//Writing directly to the SPIRAM buffer is too slow in the camera callback and causes lost frames, so I use this RAM buffer and a separate task (sd_enqueue_task) for that.
static constexpr size_t SD_FAST_BUFFER_SIZE = 8192;
Circular_Buffer s_sd_fast_buffer((uint8_t*)heap_caps_malloc(SD_FAST_BUFFER_SIZE, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL), SD_FAST_BUFFER_SIZE);

//this slow buffer is used to buffer data that is about to be written to SD. The reason it's this big is because SD card write speed fluctuated a lot and 
// sometimes it pauses for a few hundred ms. So to avoid lost data, I have to buffer it into a big enough buffer.
//The data is written to the sd card by the sd_write_task, in chunks of SD_WRITE_BLOCK_SIZE.
static constexpr size_t SD_SLOW_BUFFER_SIZE_PSRAM = 3 * 1024 * 1024;
//removeme: buffer is in RAM for esp32-vtx board
static constexpr size_t SD_SLOW_BUFFER_SIZE_RAM = 32768;  
Circular_Buffer* s_sd_slow_buffer = NULL;

//Cannot write to SD directly from the slow, SPIRAM buffer as that causes the write speed to plummet. So instead I read from the slow buffer into
// this RAM block and write from it directly. This results in several MB/s write speed performance which is good enough.
//ESP32S3: 2048  - ~1.2 MB/s
//ESP32S3: 6*512 - ~1.5 MB/s
//ESP32S3: 4096  - ~1.4...1.8 MB/s 
//ESP32S3: 10*512- ~1.9 MB/s
//ESP32S3: 8192  - 1.9...2.5 MB/s
//ESP32S3: 20*512- ~2.5 MB/s
//ESP32S3: 4096 SPI RAM - 0.38 MB/s

//ESP32 2096: 0.9 MB/s
//ESP32 6*512:1 MB/s
//ESP32 4096: 1.6 MB/s 
//ESP32 12*512:1.5 MB/s  ???
//ESP32 8192: 1.9 MB/s
static constexpr size_t SD_WRITE_BLOCK_SIZE = 8192;

//for fast SD writes, buffer has to be in DMA enabled memory
static uint8_t* sd_write_block = (uint8_t*)heap_caps_malloc(SD_WRITE_BLOCK_SIZE, MALLOC_CAP_DMA);


/*
static void shutdown_sd()
{
    if (!s_sd_initialized)
        return;
    LOG("close sd card!\n");
    
    esp_vfs_fat_sdcard_unmount("/sdcard",card);

    s_sd_initialized = false;

    //to turn the LED off
#ifdef BOARD_ESP32CAM    
    gpio_set_pull_mode((gpio_num_t)4, GPIO_PULLDOWN_ONLY);    // D1, needed in 4-line mode only
#endif    
}
*/
void updateSDInfo()
{
    if ( !s_sd_initialized) return;

    /* Get volume information and free clusters of sdcard */
    FATFS *fs;
    DWORD free_clust ;
    auto res = f_getfree("/sdcard/", &free_clust, &fs);
    if (res == 0 ) 
    {
        DWORD free_sect = free_clust * fs->csize;
        DWORD tot_sect = fs->n_fatent * fs->csize;

        SDFreeSpaceGB16 = free_sect / 2 / 1024 / (1024/16);
        SDTotalSpaceGB16 = tot_sect / 2 / 1024 / (1024/16);
	}
}

static bool init_sd()
{
    if (s_sd_initialized)
        return true;

    SDTotalSpaceGB16 = 0;
    SDFreeSpaceGB16 = 0;

#ifdef BOARD_ESP32CAM
    esp_vfs_fat_sdmmc_mount_config_t mount_config;
#ifdef CAMERA_MODEL_ESP_VTX
    mount_config.format_if_mount_failed = true;
#else
    mount_config.format_if_mount_failed = false;
#endif
    mount_config.max_files = 2;
    mount_config.allocation_unit_size = 0;

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;    
    //host.max_freq_khz = SDMMC_FREQ_PROBING;
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    host.flags = SDMMC_HOST_FLAG_1BIT;

    gpio_set_pull_mode((gpio_num_t)14, GPIO_PULLUP_ONLY);   // CLK, needed in 4-line mode only
    gpio_set_pull_mode((gpio_num_t)15, GPIO_PULLUP_ONLY);   // CMD
    gpio_set_pull_mode((gpio_num_t)2, GPIO_PULLUP_ONLY);    // D0
    //gpio_set_pull_mode((gpio_num_t)4, GPIO_PULLDOWN_ONLY);  // D1, needed in 4-line mode only
    //gpio_set_pull_mode((gpio_num_t)12, GPIO_PULLUP_ONLY);   // D2, needed in 4-line mode only
    //gpio_set_pull_mode((gpio_num_t)13, GPIO_PULLUP_ONLY);   // D3, needed in 4-line mode only

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;

    LOG("Mounting SD card...\n");
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK)
    {
        LOG("Failed to mount SD card VFAT filesystem. Error: %s\n", esp_err_to_name(ret));
        //to turn the LED off
        gpio_set_pull_mode((gpio_num_t)4, GPIO_PULLDOWN_ONLY);
        return false;
    }
#endif
#ifdef BOARD_XIAOS3SENSE
/*
    esp_vfs_fat_sdmmc_mount_config_t mount_config;
    mount_config.format_if_mount_failed = false;
    mount_config.max_files = 2;
    mount_config.allocation_unit_size = 0;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    //host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    //host.max_freq_khz = SDMMC_FREQ_PROBING;
    //host.max_freq_khz = SDMMC_FREQ_DEFAULT;
    //host.max_freq_khz = 26000;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = 9,
        .miso_io_num = 8,
        .sclk_io_num = 7,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4092
    };
    esp_err_t ret = spi_bus_initialize((spi_host_device_t)host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) 
    {
        LOG("Failed to initialize SD SPI bus.");
        return false;
    }
    //host.set_card_clk(host.slot, 10000);

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = GPIO_NUM_21;  //shared with USER LED pin
    slot_config.host_id = (spi_host_device_t)host.slot;

    LOG("Mounting SD card...\n");
    ret = esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK)
    {
        LOG("Failed to mount SD card VFAT filesystem. Error: %s\n", esp_err_to_name(ret));
        return false;
    }
*/ 
    esp_vfs_fat_sdmmc_mount_config_t mount_config;
    mount_config.format_if_mount_failed = false;
    mount_config.max_files = 2;
    mount_config.allocation_unit_size = 0;

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;    
    //host.max_freq_khz = SDMMC_FREQ_PROBING;
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
    host.flags = SDMMC_HOST_FLAG_1BIT;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 1;
    slot_config.clk = GPIO_NUM_7;
    slot_config.cmd = GPIO_NUM_9;
    slot_config.d0 = GPIO_NUM_8;

    // Enable internal pullups on enabled pins. The internal pullups
    // are insufficient however, please make sure 10k external pullups are
    // connected on the bus. This is for debug / example purpose only.
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    
    LOG("Mounting SD card...\n");
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);
    if (ret != ESP_OK)
    {
        LOG("Failed to mount SD card VFAT filesystem. Error: %s\n", esp_err_to_name(ret));
        return false;
    }
#endif

    LOG("sd card inited!\n");
    s_sd_initialized = true;

#ifdef DVR_SUPPORT
    s_air_record = true;
#endif

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card);

    updateSDInfo();

    //find the latest file number
    char buffer[64];
    for (uint32_t i = 0; i < 100000; i++)
    {
#ifdef WRITE_RAW_MJPEG_STREAM 
        sprintf(buffer, "/sdcard/v%03lu_000.mpg", (long unsigned int)i);
#else        
        sprintf(buffer, "/sdcard/v%03lu_000.avi", (long unsigned int)i);
#endif        
        FILE* f = fopen(buffer, "rb");
        if (f)
        {
            fclose(f);
            continue;
        }
        s_sd_next_session_id = i;
        s_sd_next_segment_id = 0;
        break;
    }
    return true;
}

static int open_sd_file()
{
    char buffer[64];
#ifdef WRITE_RAW_MJPEG_STREAM
    sprintf(buffer, "/sdcard/v%03lu_%03lu.mpg", (long unsigned int)s_sd_next_session_id, (long unsigned int)s_sd_next_segment_id);
#else
    sprintf(buffer, "/sdcard/v%03lu_%03lu.avi", (long unsigned int)s_sd_next_session_id, (long unsigned int)s_sd_next_segment_id);
#endif

    int fd = open(buffer, O_WRONLY | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR);

    if (fd == -1)
    {
        LOG("error to open sdcard session %s!\n",buffer);
        return-1;
    }

    LOG("Opening session file '%s'\n", buffer);
    s_sd_file_size = 0;
    s_sd_next_segment_id++;

    return fd;
}


#ifdef WRITE_RAW_MJPEG_STREAM
//=============================================================================================
//=============================================================================================
//this will write data from the slow queue to raw MJPEG file
static void sd_write_proc(void*)
{
    while (true)
    {
        if (!s_air_record || (s_shouldRestartRecording > esp_timer_get_time()))
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            continue;
        }
        
        s_shouldRestartRecording = 0;

        int fd = open_sd_file();
        if ( fd == -1)
        {
            s_air_record = false;
            SDError = true;
            vTaskDelay(1000 / portTICK_PERIOD_MS); 
            continue;
        }

        xSemaphoreTake(s_sd_slow_buffer_mux, portMAX_DELAY);
        s_sd_slow_buffer->clear();

#ifdef PROFILE_CAMERA_DATA
        s_profiler.set(PF_CAMERA_SD_SLOW_BUF, 0 );
#endif

        xSemaphoreGive(s_sd_slow_buffer_mux);

        bool error = false; 
        bool done = false;
        bool syncFrameStart = true;
        int offset = 0;

        while (!done)
        {
            ulTaskNotifyTake(pdTRUE, 1000 / portTICK_PERIOD_MS); //wait for notification

            while (true) //consume all the buffer
            {
                if (!s_air_record)
                {
                    LOG("Done recording, closing file\n", s_sd_file_size);
                    done = true;
                    break;
                }

                if ( s_shouldRestartRecording != 0) 
                {
                    done = true;
                    break;
                }

                int len = SD_WRITE_BLOCK_SIZE - offset;

                xSemaphoreTake(s_sd_slow_buffer_mux, portMAX_DELAY);
                bool read = s_sd_slow_buffer->read(&(sd_write_block[offset]), len);
#ifdef PROFILE_CAMERA_DATA    
                s_profiler.set(PF_CAMERA_SD_SLOW_BUF, s_sd_slow_buffer->size() / (SD_SLOW_BUFFER_SIZE_PSRAM / 100 ));
#endif
                xSemaphoreGive(s_sd_slow_buffer_mux);
                if (!read)
                {
                    break; //not enough data, wait
                }

                if ( syncFrameStart )
                {
                    while ( (len >= 3) && ((sd_write_block[offset] != 0xff) || (sd_write_block[offset+1] != 0xd8) || (sd_write_block[offset+2] != 0xff))) 
                    {
                        offset++;
                        len--;
                    }

                    if ( len < 3 ) 
                    {
                        offset = 0;
                        continue;
                    }
                    else
                    {
                        //LOG("Sync = %d %d 0x%02x 0x%02x\n", offset, len, (int)sd_write_block[offset], (int)sd_write_block[offset+1]);
                        syncFrameStart = false;
                        if ( offset > 0 )
                        {
                            memcpy(sd_write_block, sd_write_block+offset, len);
                            offset = len;
                            continue;
                        }
                    }
                }

                offset = 0;
                
                if (write(fd, sd_write_block, SD_WRITE_BLOCK_SIZE) < 0 )
                {
                    LOG("Error while writing! Stopping session\n");
                    done = true;
                    error = true;
                    SDError = true;
                    break;
                }
                s_stats.sd_data += SD_WRITE_BLOCK_SIZE;
                s_sd_file_size += SD_WRITE_BLOCK_SIZE;
                if (s_sd_file_size > 50 * 1024 * 1024)
                {
                    LOG("Max file size reached: %d. Restarting session\n", s_sd_file_size);
                    done = true;
                    break;
                }
            }  //while true -consume all buffer
        }  //while !done

        if (!error)
        {
            fsync(fd);
            close(fd);

            updateSDInfo();
        }
        else
        {
            s_air_record = false;
            //shutdown_sd();
        }
    }
}
#else


//=============================================================================================
//=============================================================================================
bool findFrameStart(Circular_Buffer* buffer, uint32_t &offset, size_t data_avail) 
{
    if ( data_avail <= MJPEG_PATTERN_SIZE*2 ) return false;

    uint32_t maxOffset = data_avail - MJPEG_PATTERN_SIZE*2;

    bool foundZero = false;
    while ( offset < maxOffset )
    {
        if ( buffer->peek(offset) == 0 )
        {
            foundZero = true;
            break;
        }
        else
        {
            offset += MJPEG_PATTERN_SIZE / 2;
        }
    }

    if ( !foundZero )
    {
        return false;
    }

    uint32_t offset1 = offset-1;

    //walk front until buffer contains zeros
    uint32_t count = 0;
    while ( (buffer->peek(offset) == 0) )
    {
        offset++;
        count++;

        if ( offset > (data_avail - 4) )
        {
            //something wrong
            //whole buffer is filled with zeros ?
            return false;
        }
    }

    //count zeros back
    while ( ( count < MJPEG_PATTERN_SIZE / 2 ) && (offset1 > 0) && (buffer->peek(offset1) == 0) )
    {
        offset1--;
        count++;
    }

    return ( 
        (count >= MJPEG_PATTERN_SIZE / 2) && 
        (buffer->peek(offset) == 0xFF ) &&
        (buffer->peek(offset+1) == 0xD8 ) &&
        (buffer->peek(offset+2) == 0xFF ) 
    );
}

//=============================================================================================
//=============================================================================================
void storeBuffer( int fd, size_t count, Circular_Buffer* buffer, SemaphoreHandle_t mux, uint8_t* sd_write_block, size_t& blockSize, size_t& fileSize, bool& done, bool& error)
{
    while (count > 0 )
    {
        size_t len = std::min<size_t>(count, SD_WRITE_BLOCK_SIZE - blockSize);
        
        if (mux != NULL ) xSemaphoreTake(mux, portMAX_DELAY);
        buffer->read( sd_write_block + blockSize, len);
        if (mux != NULL ) xSemaphoreGive(mux);
        count -= len;
        blockSize += len;

        if (blockSize == SD_WRITE_BLOCK_SIZE)
        {
            if (write(fd, sd_write_block, SD_WRITE_BLOCK_SIZE) < 0 )
            {
                LOG("Error while writing! Stopping session\n");
                done = true;
                error = true;
                SDError = true;
                break;
            }
            fileSize += SD_WRITE_BLOCK_SIZE;
            blockSize = 0;
            s_stats.sd_data += SD_WRITE_BLOCK_SIZE;
        }
    }
}

//=============================================================================================
//=============================================================================================
//this will write data from the slow queue to AVI file
static void sd_write_proc(void*)
{
    //frames are separated by MJPEG_PATTERN_SIZE zeros
    //frame start search: at least MJPEG_PATTERN_SIZE/2 zero bytes, then FF D8 FF
    //MJPEG_PATTERN_SIZE is 512 bytes, so we can check for zero every 256th byte instead of every byte to find pattern

    uint16_t lastFrameCounter =0;

    while (true)
    {
        if (!s_air_record || (s_shouldRestartRecording > esp_timer_get_time()))
        {
            vTaskDelay(1000 / portTICK_PERIOD_MS);

            //flush frames with possibly different resolution
            xSemaphoreTake(s_sd_slow_buffer_mux, portMAX_DELAY);
            size_t dataAvail = s_sd_slow_buffer->size();
            s_sd_slow_buffer->skip(dataAvail);
            xSemaphoreGive(s_sd_slow_buffer_mux);

            continue;
        }

        s_shouldRestartRecording = 0;

        int fd = open_sd_file();
        if ( fd == -1)
        {
            s_air_record = false;
            SDError = true;
            vTaskDelay(1000 / portTICK_PERIOD_MS); 
            continue;
        }
        
        const TVMode* v = &vmodes[clamp((int)s_ground2air_config_packet2.camera.resolution, 0, (int)(Resolution::COUNT)-1)];
#ifdef SENSOR_OV5640
        uint8_t fps = s_ground2air_config_packet2.camera.ov5640HighFPS ? v->highFPS5640 : v->FPS5640;
#else
        uint8_t fps = s_ground2air_config_packet2.camera.ov2640HighFPS ? v->highFPS2640 : v->FPS2640;
#endif        
        uint16_t frameWidth = v->width;
        uint16_t frameHeight = v->height;

        LOG("%dx%d %dfps\n", frameWidth, frameHeight, fps);

        prepAviIndex();

        bool error = false; 
        bool done = false;
        size_t blockSize = AVI_HEADER_LEN; //start with a header
        size_t aviFileSize = AVI_HEADER_LEN;
        bool lookForFrameStart = true;
        uint32_t offset = 0;
        uint32_t frameStartOffset = 0;
        uint32_t frameCnt = 0;
        while (!done)
        {
            ulTaskNotifyTake(pdTRUE, 1000 / portTICK_PERIOD_MS); //wait for notification

            if ( s_shouldRestartRecording != 0) 
            {
                done = true;
                break;
            }

            if (!s_air_record)
            {
                LOG("Done recording, closing file\n", s_sd_file_size);
                done = true;
                continue;
            }

            xSemaphoreTake(s_sd_slow_buffer_mux, portMAX_DELAY);
            size_t dataAvail = s_sd_slow_buffer->size();
            xSemaphoreGive(s_sd_slow_buffer_mux);

            //consume all data
            while ( true )
            {
                if ( lookForFrameStart )
                {
                    if ( findFrameStart( s_sd_slow_buffer, offset, dataAvail ))
                    {
                        lookForFrameStart = false;
                        //offset points to FF D8 FF
                        frameStartOffset = offset;
                    }
                    else
                    {
                        if ( offset > 150*1024)
                        {
                            //something wrong
                            xSemaphoreTake(s_sd_slow_buffer_mux, portMAX_DELAY);
                            s_sd_slow_buffer->skip(offset);
                            xSemaphoreGive(s_sd_slow_buffer_mux);
                            offset = 0;
                            s_stats.sd_drops++;
                            LOG("Unable to find frame start\n");
                        }
                        break;
                    }
                }

                //next frame start
                if ( findFrameStart( s_sd_slow_buffer, offset, dataAvail ))
                {
#ifdef TEST_AVI_FRAMES                    
                    uint16_t fc = s_sd_slow_buffer->peek(offset + 15);
                    fc <<= 8;
                    fc |=  s_sd_slow_buffer->peek(offset + 14);

                    static uint16_t prevFrameCounter;
                    if (prevFrameCounter+1 !=fc )
                    {
                        LOG("Frame: %d+1 != %d\n", prevFrameCounter, fc);
                    } 
                    prevFrameCounter = fc;
#endif
                    lookForFrameStart = true;
                    //offset points to FF D8 FF
                    //exclude zero patern
                    offset-= MJPEG_PATTERN_SIZE;
                    frameCnt++;

                    //save data from frameStartOffset to offset
                    if ( frameStartOffset > 0 )
                    {
                        xSemaphoreTake(s_sd_slow_buffer_mux, portMAX_DELAY);
                        s_sd_slow_buffer->skip(frameStartOffset);
                        xSemaphoreGive(s_sd_slow_buffer_mux);
                    }

                    size_t jpegSize = offset - frameStartOffset; 

                    uint16_t filler = (4 - (jpegSize & 0x3)) & 0x3; 
                    size_t jpegSize1 = jpegSize + filler;

                    // add avi frame header
                    uint8_t buf[8];
                    memcpy(buf, dcBuf, 4); 
                    memcpy(&buf[4], &jpegSize1, 4);
                    Circular_Buffer temp(buf, 8, 8);
                    storeBuffer( fd, 8, &temp, NULL, sd_write_block, blockSize, aviFileSize, done, error);
                    if (done) break;

                    //store jpeg
                    storeBuffer( fd, jpegSize, s_sd_slow_buffer, s_sd_slow_buffer_mux, sd_write_block, blockSize, aviFileSize, done, error);
                    dataAvail -= offset;
                    offset = 0;
                    if (done) break;

                    //store filler
                    memset(buf, 0, 4); 
                    Circular_Buffer temp1(buf, 4, 4);
                    storeBuffer( fd, filler, &temp1, NULL, sd_write_block, blockSize, aviFileSize, done, error);

                    buildAviIdx(jpegSize1); // save avi index for frame

                    if (frameCnt == (DVR_MAX_FRAMES-1))
                    {
                        break;
                    }
                }
                else
                {
                    if ( (offset - frameStartOffset) > 150*1024)
                    {
                        //something wrong
                        xSemaphoreTake(s_sd_slow_buffer_mux, portMAX_DELAY);
                        s_sd_slow_buffer->skip(offset);
                        xSemaphoreGive(s_sd_slow_buffer_mux);
                        offset = 0;
                        lookForFrameStart = true;
                        s_stats.sd_drops++;
                        LOG("Unable to find frame end\n");
                    }
                    break;
                }
            }

            if (frameCnt == (DVR_MAX_FRAMES-1))
            {
                LOG("Max frames count reached: %d. Restarting session\n", frameCnt);
                break;
            }

            if (aviFileSize > 50 * 1024 * 1024)
            {
                LOG("Max file size reached: %d. Restarting session\n", aviFileSize);
                break;
            }

            if (done) break;
        }
        
        if (!error)
        {
            // save avi index
            finalizeAviIndex(frameCnt);

            while(true)
            {
                size_t sz = writeAviIndex(sd_write_block + blockSize, SD_WRITE_BLOCK_SIZE - blockSize);
                blockSize += sz;
                if ( (blockSize == SD_WRITE_BLOCK_SIZE) || (sz == 0)) //flush block or write leftover
                {
                    if (write(fd, sd_write_block, blockSize) < 0 )
                    {
                        LOG("Error while writing! Stopping session\n");
                        done = true;
                        error = true;
                        SDError = true;
                        break;
                    }
                    blockSize = 0;
                }
                
                if ( sz == 0)
                {
                    break;
                }
            }

            // save avi header at start of file
            buildAviHdr( fps, frameWidth, frameHeight, frameCnt );

            lseek(fd, 0, SEEK_SET); // start of file
            write(fd, aviHeader, AVI_HEADER_LEN); 
            fsync(fd);
            close(fd);

            updateSDInfo();
        }
        else
        {
            s_air_record = false;
            //shutdown_sd();
        }
    }
}


#endif


//=============================================================================================
//=============================================================================================
//this will move data from the fast queue to the slow queue
static void sd_enqueue_proc(void*)
{
    while (true)
    {
        if (!s_air_record)
        {
            //vTaskDelay(1000 / portTICK_PERIOD_MS);
            ulTaskNotifyTake(pdTRUE, 1000 / portTICK_PERIOD_MS); //wait for notification
            continue;
        }

        xSemaphoreTake(s_sd_fast_buffer_mux, portMAX_DELAY);
        s_sd_fast_buffer.clear();

#ifdef PROFILE_CAMERA_DATA    
        s_profiler.set(PF_CAMERA_SD_FAST_BUF, 0);
#endif
        xSemaphoreGive(s_sd_fast_buffer_mux);

        while (true)
        {
            ulTaskNotifyTake(pdTRUE, 1000 / portTICK_PERIOD_MS); //wait for notification

            xSemaphoreTake(s_sd_fast_buffer_mux, portMAX_DELAY);
            size_t size = s_sd_fast_buffer.size();
            if (size == 0)
            {
                xSemaphoreGive(s_sd_fast_buffer_mux);
                continue; //no data? wait some more
            }

            const void* buffer = s_sd_fast_buffer.start_reading(size);
            xSemaphoreGive(s_sd_fast_buffer_mux);

            xSemaphoreTake(s_sd_slow_buffer_mux, portMAX_DELAY);
            if ( !s_sd_slow_buffer->write(buffer, size) )
            {
                s_stats.sd_drops += size;
#ifdef PROFILE_CAMERA_DATA    
                s_profiler.toggle(PF_CAMERA_SD_OVF );
#endif
            }

#ifdef PROFILE_CAMERA_DATA    
            s_profiler.set(PF_CAMERA_SD_SLOW_BUF, s_sd_slow_buffer->size() / (SD_SLOW_BUFFER_SIZE_PSRAM / 100));
#endif
            xSemaphoreGive(s_sd_slow_buffer_mux);

            xSemaphoreTake(s_sd_fast_buffer_mux, portMAX_DELAY);
            s_sd_fast_buffer.end_reading(size);

#ifdef PROFILE_CAMERA_DATA    
            s_profiler.set(PF_CAMERA_SD_FAST_BUF, s_sd_fast_buffer.size() / (SD_FAST_BUFFER_SIZE / 100));
#endif

            xSemaphoreGive(s_sd_fast_buffer_mux);

            if (s_sd_write_task)
                xTaskNotifyGive(s_sd_write_task); //notify task

            if (!s_air_record)
                break;
        }
    }
}

IRAM_ATTR static void add_to_sd_fast_buffer(const void* data, size_t size, bool addFrameStartPattern)
{
    xSemaphoreTake(s_sd_fast_buffer_mux, portMAX_DELAY);
    bool ok = s_sd_fast_buffer.write(data, size);

#ifdef WRITE_RAW_MJPEG_STREAM 
#else
    if ( ok && addFrameStartPattern ) 
    {
        ok = s_sd_fast_buffer.writeBytes(0, MJPEG_PATTERN_SIZE);
    }
#endif

#ifdef PROFILE_CAMERA_DATA
    s_profiler.set(PF_CAMERA_SD_FAST_BUF, s_sd_fast_buffer.size() / (SD_FAST_BUFFER_SIZE / 100) );
#endif

    xSemaphoreGive(s_sd_fast_buffer_mux);
    if (ok)
    {
        if (s_sd_enqueue_task)
            xTaskNotifyGive(s_sd_enqueue_task); //notify task
    }
    else
    {
        s_stats.sd_drops += size;
#ifdef PROFILE_CAMERA_DATA
        s_profiler.toggle(PF_CAMERA_SD_OVF);
#endif
    }
}

#endif