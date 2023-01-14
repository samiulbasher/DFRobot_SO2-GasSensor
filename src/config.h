#ifndef config_h
 #define config_h

 #define BAUDRATE 115200

 #define LCD_WIDTH  16
 #define LCD_HEIGHT 2

 #define LCD_RS 5
 #define LCD_EN 6
 #define LCD_D4 7
 #define LCD_D5 8
 #define LCD_D6 9
 #define LCD_D7 10
 
    /*!
    ### pin connection

    # SO2 sensor (use UART mode)
    * @ DFROBOT Gravity board dip switch "SEL" should be 1 selection
    * @ + pin   = VCC (5v)
    * @ - pin   = GND
    * @ C/R pin = D3 (softwareSerial Tx) 
    * @ D/T pin = D2 (softwareSerial Rx)

    # SO2 sensor (use I2C mode)
    * @ DFROBOT Gravity board dip switch "SEL" should be 0 selection
    * @ GND = GND
    * @ VCC = VCC (5v)
    * @ C/R pin = SDA
    * @ D/T pin = SCL
    */


 #define DBG_LED  13
 #define BUZZ_PIN A3
 #define ESW_PIN  A2
 #define TH_PIN   A6

 #define LONG_BEEP  0
 #define SHORT_BEEP 1

 void beeper(uint8_t numOfBeep, uint8_t beepPattern);

#endif
