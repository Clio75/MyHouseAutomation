/**
 * The MySensors Arduino library handles the wireless radio link and protocol
 * between your home built sensors/actuators and HA controller of choice.
 * The sensors forms a self healing radio network with optional repeaters. Each
 * repeater and gateway builds a routing tables in EEPROM which keeps track of the
 * network topology allowing messages to be routed to nodes.
 *
 * Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
 * Copyright (C) 2013-2015 Sensnology AB
 * Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
 *
 * Documentation: http://www.mysensors.org
 * Support Forum: http://forum.mysensors.org
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 *******************************
 *
 * REVISION HISTORY
 * Version 1.0 - Henrik Ekblad
 * 
 * DESCRIPTION
 * Motion Sensor example using HC-SR501 
 * http://www.mysensors.org/build/motion
 *
 */

// Enable debug prints
 #define MY_DEBUG

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

#include <SPI.h>
#include <MySensors.h>

int BATTERY_SENSE_PIN = A0;  // select the input pin for the battery sense point
int oldBatteryPcnt = 0;
int batteryPcnt = 0;

unsigned long SLEEP_TIME = 3600*1000; // Sleep time between reports (in milliseconds)
#define Alarm_PIN 3   // The digital input you attached your motion sensor.  (Only 2 and 3 generates interrupt!)
#define Alarm_ID 1   // Id of the child Tripped alarm
#define BATT_ID  2   // ID of the Child Voltage of Battery

// Initialize motion message
MyMessage msg(Alarm_ID, V_TRIPPED);

void setup()  
{  
  // use the 1.1 V internal reference
#if defined(__AVR_ATmega2560__)
   analogReference(INTERNAL1V1);
#else
   analogReference(INTERNAL);
#endif

pinMode(Alarm_PIN, INPUT);      // sets the motion sensor digital pin as input
} // End Setup

void presentation()  {
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("Smoke Alarm monitroing", "1.0");

  // Register all sensors to gw (they will be created as child devices)
  present(Alarm_ID, S_SMOKE);
}

void loop()     
{     
  Alarm();
  ReadBatt();
      // Sleep until interrupt comes in on motion sensor. Send update every Sleep_Time.
  sleep(digitalPinToInterrupt(Alarm_PIN), CHANGE, SLEEP_TIME);
}

void Alarm()
{ 
        // Read AlarmPIN
  boolean tripped = digitalRead(Alarm_PIN) == HIGH; 
        
  #ifdef MY_DEBUG
      if (tripped) {
        Serial.println("Smoke detektor is tripped ");
      }else{
        Serial.println("Smoke detektor is Armed ");}
      
  #endif
    send(msg.set(tripped?"1":"0"));  // Send tripped value to gw 
      //considder to add a delay between alerts. 
};

void ReadBatt()
{
      // get the battery Voltage
   int sensorValue = analogRead(BATTERY_SENSE_PIN);
      // 1023 is the number of seteps the microcontroller can detect. And max voltage in the A0 is 1V1
      // Since I have 10 volt max and 1 volt on the microcontroller I need to divide the voltage down with 
      // two resistors. R1=approx 4M and R2 approx to 444K(I will use a potensiometer to final adjust the correct votage.

      // Here we need to detect battery voltage. And max voltage must be litle higher than 9 volt.
      // So I have defined that I need approx 10 Volt. 10V / 1023 = 0.009775 Close to 0.01
      // Volts per bit = 0.01 With this I can measure 10.23 volts. 
   float batteryV  = sensorValue * 0.00949;
       // I have diffined that 7 Volt is my 0. And the battery needst to be change. 
      // The orginal voltage of my battery is 9 volt. So I take the 9-sensed_value to find out how mutc is used. 
      // Then divide this on the known difference alowed. and make a % number of this. 
   int batteryPcntD = (1-(9-batteryV)/(2))*100;
         
   if (batteryPcntD < 0) {// this is to avoid negative %Value, witch not give any meaning
      int batteryPcnt = 0;
      }else {batteryPcnt = batteryPcntD;} // don't need negative value

   #ifdef MY_DEBUG
     Serial.println(sensorValue);
     Serial.print("Battery Voltage: ");
     Serial.print(batteryV);
     Serial.println(" V");
  
     Serial.print("Battery percent: ");
     Serial.print(batteryPcnt);
     Serial.print(" % Debug : ");
     Serial.println(batteryPcntD);
   #endif
        // I sending the battery % everythime. This was the main thing to monitor with this Arduino.
     
     sendBatteryLevel(batteryPcnt);
 };
