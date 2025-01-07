void initialize_status_led(){
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1ULL << STATUS_LED_PIN;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(STATUS_LED_PIN, STATUS_LED_OFF);
}

void initialize_flash_led(){
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1ULL << FLASH_LED_PIN;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
}

void set_status_led(bool enabled){
#ifdef STATUS_LED_PIN
    gpio_set_level(STATUS_LED_PIN, enabled ? STATUS_LED_ON : STATUS_LED_OFF);
#endif    

#ifdef FLASH_LED_PIN
    if ( enabled) 
    {
        gpio_set_pull_mode(FLASH_LED_PIN, GPIO_PULLUP_ONLY);    //SD D1, needed in 4-line mode only
    }
    else
    {
        gpio_set_pull_mode(FLASH_LED_PIN, GPIO_PULLDOWN_ONLY);    //SD D1, needed in 4-line mode only
    }
#endif    
}


void update_status_led(){
 /*
  if ( cameraInitError )
  {
    bool b = (millis() & 0x7f) > 0x40;
    b &= (millis() & 0x7ff) > 0x400;
    digitalWrite( LED_PIN, b ? LOW : HIGH);
    digitalWrite( 4, b ? HIGH : LOW);
    return;
  }

  if ( initError )
  {
    bool b = (millis() & 0x7f) > 0x40;
    digitalWrite( LED_PIN, b ? LOW : HIGH);
    digitalWrite( 4, b ? HIGH : LOW);
    return;
  }
*/
  if (s_air_record)
  {
    bool b = (millis() & 0x7ff) > 0x400;
    set_status_led(b);
  }
  else
  {
#ifdef DVR_SUPPORT    
    set_status_led(true);
#else
    bool b = (millis() & 0x7ff) > 0x400;
    set_status_led(b);
#endif
  }
}

void update_status_led_file_server(){
    bool b = (millis() & 0x2ff) > 0x180;
    set_status_led(b);
}

