To run this install Adafruit BMP280 Library and put it into your arduino library folder
https://github.com/adafruit/Adafruit_BMP280_Library

Flash and run apogee.ino for the test and then eepromread.ino is for post analysis  once the test is complete.
Each reading from the test will take up 2 bytes thats why the addresses are indexed by 2.
The micro holds up to 1024 address readings.
EEPROM only stores int or long data types, so no sig figs after the decimal will be recorded.
