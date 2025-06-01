#pragma once
void initialize_rec_button(){
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = 1ULL << REC_BUTTON_PIN;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
}


bool getButtonState(){
    return gpio_get_level((gpio_num_t)REC_BUTTON_PIN) == 0;
}

void checkButton(){
  static uint32_t debounceTime = millis() + 100;
  static bool lastButtonState = false;

  if ( debounceTime > millis() ) return;
  bool buttonState = getButtonState();
  if ( buttonState != lastButtonState )
  {
    debounceTime = millis() + 100;
    lastButtonState = buttonState;

    if ( buttonState )
    {
        if ( s_restart_time == 0 )
        {
#if defined(ENABLE_PROFILER) && defined (START_PROFILER_WITH_BUTTON)
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
                s_profiler.start(1000);
            }
#else
            s_air_record = !s_air_record;
#endif            
        }
        LOG("Button pressed!\n");
    }
    else
    {
        LOG("Button unpressed!\n");
    }
  }
}

