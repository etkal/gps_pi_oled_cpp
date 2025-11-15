# GPS_PI_OLED_cpp
GPS implementation in C++ using Raspberry Pi with SSD1306 OLED display

![Raspberry Pi 4B showing GPS module and SSD1306 display](images/Pi_GPS_OLED.jpg)

- Background

  This project is a port of my Raspberry Pi Pico RP2040 application.
  
- Hardware

  Raspberry Pi serial and I2C ports, along with a simple 3.3 volt GPS module (in this implementation it's an AdaFruit GPS Breakout board), and a 128x64 SSD1306 OLED display. The serial port can be the UART or USB serial device.
  
- Software Requirements

  Raspberry Pi, conan install using conancenter repo.

- Operation

  In main.cpp the required abstraction objects are created, and the program reads NMEA 0183 sentences from the GPS serial port.

  The data is correlated and displayed in textual and graphical form on the display.  For the SSD1306 it displays the latitude, longitude, altitude, GMT time and an indication of the number of satellites and fix type.  A graphical representation of the satellite positions is displayed as well.

- Enjoy!!
