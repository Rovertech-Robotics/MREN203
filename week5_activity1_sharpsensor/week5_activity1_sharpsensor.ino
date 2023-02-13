/**
 * @file sharp-range.ino
 * @author Naser Al-Obeidat 
 * @brief Arduino program to read proximity data from a Sharp GP2Y0A21YK.
 * @version 2.1
 * @date 2022-12-21
 *
 * @copyright Copyright (c) 2022
 *
 */

 

// Arduino analog input pin to which the Sharp sensor is connected
const byte SHARP_PIN = A0;
const byte SHARP_PIN2 = A1;
const byte SHARP_PIN3 = A2;  

 

// Variables to store the proximity measurement
int sharp_val = 0; // integer read from analog pin
int sharp_val2 = 0; 
int sharp_val3 = 0; 
float sharp_range; // range measurement [cm]

 

void setup()
{
    // Open the serial port at 115200 bps
    Serial.begin(250000);
    pinMode(sharp_val, INPUT);

 

}

 

void loop()
{
    // Read the sensor output (0-1023, which is 10 bits and fits inside an Arduino int-type)
    sharp_val = analogRead(SHARP_PIN);
    sharp_val2 = analogRead(SHARP_PIN2);
    sharp_val3 = analogRead(SHARP_PIN3);
    // Print all values
    Serial.print ("Left Sharp Sensor:");
    Serial.print(sharp_val);
    Serial.print("\n");
    Serial.print ("Right Sharp Sensor:");
    Serial.print(sharp_val2);
    Serial.print("\n");
    Serial.print ("Front Sharp Sensor:");
    Serial.print(sharp_val3);
    Serial.print("\n"); 

 

    
    // Delay for a bit before reading the sensor again
    delay(100);
}