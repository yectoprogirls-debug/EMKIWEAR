idf_component_register(SRCS "main.c"
                            "bmi270_esp32.c"
                            "test_bmi270.c"
                            "signal_proc.c"
                            "tremor.c"
                            "ble_stream.c"
                            "hourly_summary.c"
                            "display.c"
                            "display_gfx.c"
                            "buttons.c"
                            "charge.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_i2c esp_driver_spi esp_lcd esp_lcd_gc9a01
                                bmi270_api bt nvs_flash esp_timer esp_driver_gpio
                                esp_driver_rmt esp_hw_support)
