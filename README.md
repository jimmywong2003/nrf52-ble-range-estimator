# nrf52-ble-rssi-with-packet_error_rate

Example on how to show the RSSI with Packet Error Rate with difference link (1Mbps, Codec)

## Method
* Press the button 1 : Switch between 1Mbps and Coded PHY (125kbps)
* Press the button 2 : Change the TX Power on 0dBm, 4dBm, 8dBm
* Press the button 3 : Start / Stop advertising (Peripheral / Central)

![image](https://github.com/jimmywong2003/nrf52-ble-rssi-with-packet_error_rate_range_test/blob/master/picture/nRF52840_LONG_RANGE_DEMO.jpg)

## Output
* Show the RSSI value during the connection
* Show the Packet Success Rate (PSR) in %


## Requirement

### SDK 15.3
* ble_central_long_range_demo_kit_sdk15.3
* ble_peripheral_long_range_demo_kit_sdk15.3
* NRF52840 DK x 2
* Softdevice S140v6.1.1
* Adafruit 1947 (320x240) display (https://www.adafruit.com/product/1947) x 2
* SDK 15.3
* Segger Embedded Studio

### SDK 17.0
* ble_central_long_range_sdk17.0
* ble_peripheral_long_range_sdk17.0
* NRF52840 DK x 2 or NRF52833 DK x 2
* Softdevice S140v7.0.1
* Adafruit 1947 (320x240) display (https://www.adafruit.com/product/1947) x 2
* Segger Embedded Studio

### With / Without LCD by configuration of WITH_LCD_ILI9341 (default is off)
```
#ifdef WITH_LCD_ILI9341
        app_display_init(&m_application_state);
#endif
```

All the details can be found at [URL](https://jimmywongiot.com/2019/10/22/ble-range-estimator-on-the-nordic-nrf52840/).
