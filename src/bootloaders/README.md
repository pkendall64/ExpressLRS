These bootloaders have been built from https://github.com/pkendall64/ExpressLRS-ESP32-Bootloader

They are based on the esp-idf 5.4 bootloader, and have been built with a `bootloader_before_init` hook that resets the RMT, LEDC & MCPWM devices so PWM receivers don't continue pumping out stale PWM signals after a reboot/brown-out.

They also set the log-level set to ERROR so we don't get garbage printed on the UART pin. 
