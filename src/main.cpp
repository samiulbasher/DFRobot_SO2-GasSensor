#include <Arduino.h>
#include <LiquidCrystal.h>

#include "config.h"
#include "DFRobot_MultiGasSensor.h"

/*!
  * https://github.com/DFRobot/DFRobot_MultiGasSensor
  * @brief The sensor actively reports all data
  * @n Experimental method: Connect the sensor communication pin to the main control, then burn     codes into it. 
  * @n Communication mode selection, dial switch SEL:0: IIC, 1: UART
 @n I2C address selection, the default I2C address is 0x74, A1 and A0 are combined into 4 types of IIC addresses
                | A1 | A0 |
                | 0  | 0  |    0x74
                | 0  | 1  |    0x75
                | 1  | 0  |    0x76
                | 1  | 1  |    0x77   default i2c address
  * @n Experimental phenomenon: Print all data via serial port
*/


LiquidCrystal lcd(LCD_RS, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

//Enabled by default, use IIC communication at this time. Use UART communication when disabled
#define I2C_COMMUNICATION

#ifdef  I2C_COMMUNICATION
  #define I2C_ADDRESS    0x77
  DFRobot_GAS_I2C gas(&Wire ,I2C_ADDRESS);
#else
  #if (!defined ARDUINO_ESP32_DEV) && (!defined __SAMD21G18A__)
    /**
      UNO:pin_2-----RX
          pin_3-----TX
    */
    SoftwareSerial mySerial(2,3);
    DFRobot_GAS_SoftWareUart gas(&mySerial);
  #else
    /**
      ESP32:IO16-----RX
            IO17-----TX
    */
    DFRobot_GAS_HardWareUart gas(&Serial2); //ESP32HardwareSerial
  #endif
#endif


unsigned long previous_time = 0;
unsigned long wait_update_time = 1000;    //we will check the sensor 1 times a second

String gasType; 
float readGas = 0;
float readTemp = 0;

uint8_t clean_LCD = false;

void setup() {

  Serial.begin(BAUDRATE);
  lcd.begin(LCD_WIDTH, LCD_HEIGHT);

  pinMode(DBG_LED, OUTPUT);
  pinMode(BUZZ_PIN, OUTPUT);
  pinMode(TH_PIN, INPUT);

  //Sensor init, used to init serial port or I2C, depending on the communication mode currently used
  while(!gas.begin())
  {
    Serial.println("NO Deivces !");
    delay(1000);
  }
  Serial.println("The device is connected successfully!");


  /*
  * @n  INITIATIVE The sensor proactively reports data
  * @n  PASSIVITY The main controller needs to request data from sensor
  */
  gas.changeAcquireMode(gas.PASSIVITY);
  delay(1000);

  /**
  * Turn on temperature compensation: gas.ON  :turn on
  *                                   gas.OFF :turn off
  */
  gas.setTempCompensation(gas.ON);

  Serial.println(">start");
}

void loop() {

  if((millis() - previous_time) > wait_update_time) //check every 1 times in a secound
  {
    previous_time = millis();

    digitalWrite(DBG_LED,HIGH);
    gasType  = gas.queryGasType();
    readGas  = gas.readGasConcentrationPPM();
    readTemp = gas.readTempC();
  }

  if(gasType != NULL) // read sensor data without problems
  {
    /*print serial port*/
    Serial.print("Ambient ");
    Serial.print(gasType);
    Serial.print(" concentration is: ");
    Serial.print(readGas);
    Serial.println(" ppm");

    Serial.print("The board temperature is: ");
    Serial.print(readTemp);
    Serial.println(" ℃");
    Serial.println();



    /*print display*/
    if(clean_LCD == true)
    {
      clean_LCD = false;
      lcd.clear();
    }

    //lcd.setCursor(0, 0);
    String line1 = " " + gasType + ": " + String(readGas) +  "ppm ";
    lcd.setCursor((LCD_WIDTH - line1.length()) / 2, 0); //center the lcd text
    lcd.print(line1);

    //lcd.setCursor(2, 1);
    String line2 = " B.Temp:" + String(readTemp) + (char)223 +"C ";
    lcd.setCursor((LCD_WIDTH - line2.length()) / 2, 1); //center the lcd text
    lcd.print(line2);

    digitalWrite(DBG_LED,LOW);
  }

  else
  {
    clean_LCD = true;
    Serial.println(">Data reading problem please reset.");

    lcd.setCursor(0, 0);
    lcd.print(">Reading problem");
    lcd.setCursor(0, 1);
    lcd.print(" please reset.  ");
  }
}

void beeper(uint8_t numOfBeep, uint8_t beepPattern)
{
  uint16_t beepDelay;
  
  if(beepPattern == SHORT_BEEP)  beepDelay = 10;
  if(beepPattern == LONG_BEEP )  beepDelay = 300;

  for (int i = 0; i < numOfBeep; i++)
  {
    digitalWrite(BUZZ_PIN, HIGH);
    delay(beepDelay);
    digitalWrite(BUZZ_PIN, LOW);
    delay(beepDelay);
  }
}