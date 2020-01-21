# Lorro_BQ25703A

A library for interfacing with Texas Instruments BQ25703A battery charger chip.
This chip has a comprehensive set of capabilities and many settings required
to use it properly, so reading through the data sheet is a must.
You will need to refer to the TI data sheet to get a description of all the different
registers http://www.ti.com/lit/ds/symlink/bq25703a.pdf
This library enables easy reading and writing of the registers. The data is
organised into a data structure similar to that outlined in the TRM and for each
entry includes the value, address and data type.


<!-- START COMPATIBILITY TABLE -->

## Compatibility

MCU                | Tested Works | Doesn't Work | Not Tested  | Notes
------------------ | :----------: | :----------: | :---------: | -----
Atmega328 @ 16MHz  |              |             |     X      |
Atmega328 @ 12MHz  |              |             |     X      |
Atmega32u4 @ 16MHz |              |             |     X      |
Atmega32u4 @ 8MHz  |              |             |     X      |
ESP8266            |              |             |     X      |
ESP32              |              |             |     X      |
Atmega2560 @ 16MHz |              |             |     X      |
ATSAM3X8E          |              |             |     X      |
ATSAM21D           |              |             |     X      |
ATtiny85 @ 16MHz   |              |             |     X      |
ATtiny85 @ 8MHz    |              |             |     X      |
Intel Curie @ 32MHz|              |             |     X      |
STM32F2            |              |             |     X      |
Teensy 3.2         |      X       |             |            |



<!-- END COMPATIBILITY TABLE -->
