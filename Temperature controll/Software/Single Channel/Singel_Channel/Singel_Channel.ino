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
  0.1 First Rev
  0.2 -Change PID variables From consKp=3, consKi=1, consKd=3; TO: consKp=0.4, consKi=0, consKd=0.7;
      -Using NODE_ID to spread the heaters one the hour 0=0 1=6 2=12 3=18 4=24 .. .. .. 
      -Longer DUBUG time. from 10 sek to 15 sec.
  0,3 consKp=10, consKi=2.5, consKd=1;

  
 */

// Enable debug prints to serial monitor
#define MY_DEBUG 

// Enable and select radio type attached
#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

// Enabled repeater feature for this node
//#define MY_REPEATER_FEATURE

#include <SPI.h>
#include <MySensors.h>
#include <PID_v1.h>

// Varibles 
#define RELAY_ON 1     // GPIO value to write to turn on attached relay
#define RELAY_OFF 0    // GPIO value to write to turn off attached relay
#define CHILD_ID 1     // Id of the sensor child
#define RELAY_PIN 3     // 

float ReadVar3 = 0;    // VAR3 
float ReadVar4 = 0;    // VAR4
unsigned long debug = millis();
unsigned long PIDTime = millis();
boolean SensorReceived = false;
boolean SetTempReceived = false; 
boolean Var3Received = false;
boolean Var4Received = false;
String State = "";
double old_var2 = 0;

unsigned long  WindowSize = 3600000; //One hour in milli_Secounds
//unsigned long  WindowSize = 60000; //One min in milli_Secounds for debug 
unsigned long windowStartTime;
unsigned long SaveEprom;
// PID valiables. 
double consKp=20, consKi=0.5, consKd=0.1;

double Setpoint=23, Input = Setpoint;
double Output = WindowSize / 5000 ; //Output is in Secounds but WindowSize is in milliseounds.
unsigned long OutputMS= Output * 1000; 

PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, P_ON_M, DIRECT);

// Eprom Settings
#define EEPROM_OUTPUT_HIGH_ADDRESS 0
#define EEPROM_OUTPUT_LOW_ADDRESS 1
#define EEPROM_SETPOINT_ADDRESS 2
   
MyMessage mSGvAR(CHILD_ID+1,V_VAR1);
MyMessage mSvAR2(CHILD_ID+1,V_VAR2);

void setup() {
    int NODEID = getNodeId();
       //Serial.print("Node ID : ");Serial.println(NODEID);
    // setup output, and relay to default
    pinMode(RELAY_PIN, OUTPUT);  
    digitalWrite(RELAY_PIN,RELAY_OFF);
      
    // Start timers
    // find node last digit. 
    int TEMP = NODEID;
    while (TEMP>10){
      //Like my heaters to start in diffrent times. So I find the last digit(0-9) 
      //and have 6 minuts difference on these. 
      TEMP = TEMP -10;
      }
    // take the now time and adjust windows stat time Back in time. 
    windowStartTime = millis() - (TEMP*360000);

    //tell the PID to range between 0 and the full window size
      myPID.SetOutputLimits(0, (WindowSize / 2000)); // input in secounds
    //turn the PID on
      myPID.SetMode(AUTOMATIC);
      
    //Recall last Setpoint from E-prom
      Setpoint = loadState(EEPROM_SETPOINT_ADDRESS);
      if (Setpoint >40) {
        Serial.println("Error on Eprom or First time run. Setpoint = 20 ");
        Setpoint = 20 ;
      }else{
        Serial.print("Setpoint from EEPROM : ");Serial.println(Setpoint);
      }
    //Recall Output from E-prom
      Output = word(loadState(EEPROM_OUTPUT_HIGH_ADDRESS), loadState(EEPROM_OUTPUT_LOW_ADDRESS));
      Serial.print(" Output from EEprom : ");Serial.println(Output); 
      wait(1000);
      Serial.print("EEprom Value of the Output : "); Serial.println(Output);
      if (Output > WindowSize / 1000) {
        Output = WindowSize / 4000 ; // Failsafe if eeprom is not stored. 
        SaveOutput(true);
        Serial.println("Using Default output = Windowssize /4");Serial.print("New Output is : "); Serial.print(Output);
      }else {
        Serial.println(" Output size retrived from EEPROM:  ");
        Serial.print("Output : "); Serial.println(Output);Serial.print("WindowsSize : "); Serial.println(WindowSize);
      }
} // setup

void presentation()  
{  
  //Send the sensor node sketch version information to the gateway
  sendSketchInfo("Temp Coltroller", "1.1");
  //present(CHILD_ID,S_LIGHT); // not in USE. 
  present(CHILD_ID+1,S_HEATER);
// Ver 1.1 Added that opening is not lower than 5 min. But Time between opening will increese from 1 hour up to 3,5 hours. 
// ver 1.2 Will adding 1.25 hour for each minute under 5 minutte heating. PID change from 3-1-3 to 5-1-2.5 
}

void loop() 
{  
  /****************************************************
   * Check If time to calculate PID again. 
   * Every 5 secound or if the Temp sensor is received.
   ****************************************************/
    if ((SensorReceived) || (millis() - PIDTime > 3000)) {
      myPID.Compute();
      PIDTime = millis();
      OutputMS = Output * 1000; // sec -> MilliSec
      if        (OutputMS < 120000)   {WindowSize =  7200000; //If under 2 min 5 min opening every 2.0 hour
      }else if  (OutputMS < 180000)   {WindowSize =  6300000;  //If under 3 min 5 min opening every 1.75 hour
      }else if  (OutputMS < 240000)   {WindowSize =  5400000;  //If under 4 min 5 min opening every 1.5 hour
      }else if  (OutputMS < 300000)   {WindowSize =  4500000;  //If under 5 min 5 min opening every 1.25 hour
      }else {WindowSize = 3600000;}                        // Windowszise is 1 hour.
      
      Serial.print("Sensor Recieved is : ");
      if (SensorReceived) {
        SensorReceived = false;
        Serial.println("TRUE, Changed to False");
      }else {Serial.println("false"); }
      
    }// IF PID
  /****************************************************
   * Check if windows Size is overdue and must be reset
   ****************************************************/
    if (millis() - windowStartTime > WindowSize)
    { windowStartTime += WindowSize;
      #ifdef MY_DEBUG
        Serial.println("Windows Timer restated "); 
      #endif
    }
  /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/  
   if (OutputMS < 30000){// if under 5 min we always open for 5 min.
      //this state is to make sure that the floor is no to cold even if the temperature in the room is OK. 
      if (30000 < millis() - windowStartTime) {
        digitalWrite(RELAY_PIN, RELAY_OFF);
        State = "Relay OFF";
      }else {
        digitalWrite(RELAY_PIN, RELAY_ON);
        State = "Relay ON";
      }// Window Restart}
    }else{
       if (OutputMS < millis() - windowStartTime) {
        digitalWrite(RELAY_PIN, RELAY_OFF);
        State = "Relay OFF";
      }else {
        digitalWrite(RELAY_PIN, RELAY_ON);
        State = "Relay ON";
      }// Window Restart}
    }      
  /************************************************
   * writeing Debug to user on serial port. 
   ************************************************/ 
   SaveOutput(false);
   WriteDebug(); 
}// END LOOP

void receive(const MyMessage &message) {
  if (message.type==V_VAR1) {  
    Input = message.getFloat();
    Serial.print("Received V_VAR1(From Senor) from VERA :");Serial.println(message.getFloat());
    SensorReceived = true;
  }// V_VAR1

  else if (message.type==V_VAR2) {  
    float ReadVar2 = message.getFloat();
    Serial.print("Received Read V_VAR2 from VERA :");Serial.println(message.getFloat());
  }// V_VAR2

    else if (message.type==V_VAR3) {  
    ReadVar3 = message.getFloat();
    Serial.print("Received Read V_VAR3 from VERA :");Serial.println(message.getFloat());
    Var3Received = true;
  }// V_VAR3
  
    else if (message.type==V_VAR4) {  
    ReadVar4 = message.getFloat();
    Serial.print("Received Read V_VAR4 from VERA :");Serial.println(message.getFloat());
    Var4Received = true;
  }// V_VAR4
  
  else if(message.type==V_HVAC_SETPOINT_HEAT) {
     // Get Setpoint
      Setpoint = message.getFloat();
      SetTempReceived = true;
    //Save to EEprom Saved as a INT
      saveState(EEPROM_SETPOINT_ADDRESS, message.getInt()); 
    // Write some debug info
      Serial.print("Incoming change for Setpoint : ");Serial.println(message.getFloat());
  }// V_Setpoint_heat

  else if (message.type==V_HVAC_FLOW_STATE) {
     // Get Setpoint
      float FloatState = message.getFloat();
      bool FolwReceived = true;
    // Write some debug info
      Serial.print("Incoming change for Flow State : ");Serial.println(message.getString());
  }// V_HVAC_FLOW_STATE
  
  else if (message.type==V_TEMP) {
     // Get V_TEMP
      float VtempState = message.getFloat();
      bool FolwReceived = true;
    // Write some debug info
      Serial.print("Incoming change for V_TEMP  : ");Serial.println(message.getFloat());
  }// V_TEMP
  
} // receive

void WriteDebug(){
#ifdef MY_DEBUG
  if (millis() - debug > 15000) {
    debug = millis();
    Serial.print("Input : ");Serial.println(Input);
    Serial.print("setpoit : ");Serial.println(Setpoint);
    Serial.print("Output : ");Serial.println(Output);
    Serial.print("New Output : ");Serial.println(Output);
    Serial.print("Start time : "); Serial.println(windowStartTime);
    Serial.print("Hvor langt : "); Serial.println((millis()-windowStartTime)/1000 );
    Serial.print("Output time is : "); Serial.println(Output);
    Serial.print("Output i Minutter : ");Serial.println(float(OutputMS/60000));
    Serial.print("Windows time is : "); Serial.println(WindowSize/1000);
    Serial.println(State);  
    // Sending the output time to VERA 
    double new_var2 = Output/60;
    if (old_var2 != new_var2){
        old_var2 = new_var2;
        send(mSvAR2.set(new_var2, 2));
      }//VAR2 
    }
#endif

};// WriteDebug

void SaveOutput(bool save) {
   /************************************************
   * Saving output to Eprom Every Hour.
   ************************************************/ 
  if ((millis() - SaveEprom > (WindowSize-1000) ) || (save == true)){
    // saving output to Eprom once every Windows Size
    SaveEprom = millis();
    unsigned int TEMP = int(Output);
    saveState(EEPROM_OUTPUT_HIGH_ADDRESS, highByte(TEMP)); 
    saveState(EEPROM_OUTPUT_LOW_ADDRESS, lowByte(TEMP));
#ifdef MY_DEBUG 
    Serial.print("Save to EPROM - High Byte : "); Serial.println(highByte(TEMP));
    Serial.print("Save to EPROM - Low Byte : "); Serial.println(lowByte(TEMP));
#endif
  };// IF Savetime
}//END SaveOutput


