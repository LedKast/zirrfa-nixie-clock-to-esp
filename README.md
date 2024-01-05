# ESP8266 mod for zirrfa v5.1 IN14 nixie clock board 

Cheap boards with name 'zirrfa' for nixie clock present on aliexpress: https://aliexpress.com/item/1005001889131974.html. This project allows to expand functionality and fix problems of this board.

![Alt text](img/zirrfa-stock.png)

Features:
*   Automatic time update (via NTP server)
*   Smooth fade in/out for digits
*   ~~Ability to change digits brightness~~ (not implemented)

## Upgrade guide

### Firmware
*   Create file `include/secret.h` with code:
    ```
    const char *WIFI_SSID     = "your-wifi-ap-ssid";
    const char *WIFI_PASSWORD = "your-wifi-ap-password";
    ```
*   Build project: you can use VScode + PlatformIO to build and upload firmware to esp board

### Hardware
*   Used esp board - WeMos D1 mini

*TODO hardware guide image*

## Scheme
![Alt text](img/scheme.JPG)

## PCB
![Alt text](img/PCB.png)
