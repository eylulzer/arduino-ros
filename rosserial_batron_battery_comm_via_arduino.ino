#define USE_USBCON
#include "ros.h"
#include "battery_from_arduino_msgs/BatteryStatusFromArduino.h"
#include <Wire.h> 

int BMSAddress = 11;   
int Temp,okundu;
float Temp1;
int Volt,SOC;
signed int Current;
int Cycle;
int x=0;
int deneoku=0;
int end_cycle,end_temp,end_volt,end_cur;  


ros::NodeHandle nh;

battery_from_arduino_msgs::BatteryStatusFromArduino battery_state_from_arduino; // Doğru türü kullan
ros::Publisher pub("battery_state_from_arduino", &battery_state_from_arduino);

void setup() 
{

  Serial.begin(9600); //BaudRate 9600  
  Wire.begin();
  Wire.setClock(100000);

  nh.initNode();     // Ros publisher 
  nh.advertise(pub);

}

void loop() 
{
 i2cRead_Temp();
 delay(1000);
 i2cRead_Volt();
 delay(1000);
 i2cRead_Current();
 delay(1000);
 i2cRead_SOC();
 delay(1000);

  battery_state_from_arduino.voltage = Volt;
  battery_state_from_arduino.current = Current;
  battery_state_from_arduino.temperature = Temp1;
  battery_state_from_arduino.capacity = 40;

  pub.publish(&battery_state_from_arduino);

  nh.spinOnce();
  delay(500);
}



//-----------------------------------
void i2cRead_Temp()
{
  okundu=1;
  deneoku=0;
  while(okundu==1)
  {
    Wire.beginTransmission(BMSAddress);
    Wire.write(0x08);
    end_temp=Wire.endTransmission();
    okundu=1;

    deneoku=deneoku+1;
    if(deneoku>=100)
    {
      okundu=0;
      deneoku=0;
    }
    if(end_temp==0)
    {
       Wire.requestFrom(BMSAddress,2);
       if(Wire.available()) 
       {
        Temp = (Wire.read() | Wire.read()<<8);
        Temp1=((float)Temp/10.0-273.15);
        Serial.print("Temp:");
        Serial.println(Temp1);
        okundu=0;
       }
    }
  }
  end_temp=5;
  delay(5);
}

//-----------------------------------
void i2cRead_Volt()
{
  okundu=1;
  deneoku=0;
  while(okundu==1)
  {
    Wire.beginTransmission(BMSAddress);
    Wire.write(0x09);
    end_volt=Wire.endTransmission();

    deneoku=deneoku+1;
    if(deneoku>=100)
    {
      okundu=0;
      deneoku=0;
    }

    if(end_volt==0)
    {
      Wire.requestFrom(BMSAddress,2);
      if(Wire.available()) 
      {
        Volt = (Wire.read() | Wire.read()<<8); 
        Serial.print("Volt:");
         Serial.println(Volt);
         okundu=0;
      }
    }
  }
  end_volt=5;
    delay(5);
}

//-----------------------------------
void i2cRead_Current()
{
    okundu=1;
    deneoku=0;
  while(okundu==1)
  {
    Wire.beginTransmission(BMSAddress);
    Wire.write(0x0A);
    end_cur=Wire.endTransmission();

    deneoku=deneoku+1;
    if(deneoku>=100)
    {
      okundu=0;
      deneoku=0;
    }

    if(end_cur==0)
    {
      Wire.requestFrom(BMSAddress,2);
      if(Wire.available()) 
      {
        Current = (Wire.read() | Wire.read()<<8); 
        int sign= (Current>>15); 
        //Serial.print("sign");
        //Serial.println(sign);
        if(sign==1)
        {
          Current=Current-65535;
        }
         Serial.print("Current:");
        Serial.println(Current);
        okundu=0;
      }
    }
  }
  end_cur=5;
  delay(5);
}

//-----------------------------------
void i2cRead_SOC()
{
  okundu=1;
  deneoku=0;
  while(okundu==1)
  {
    Wire.beginTransmission(BMSAddress);
    Wire.write(0x0D);
    int end_soc=Wire.endTransmission();

    deneoku=deneoku+1;
    if(deneoku>=100)
    {
      okundu=0;
      deneoku=0;
    }

    if(end_soc==0)
    {
      Wire.requestFrom(BMSAddress,2);
      if(Wire.available()) 
      {
        SOC = (Wire.read() | Wire.read()<<8); 
         Serial.print("SOC:");
         Serial.println(SOC);
         okundu=0;
      }
    }
    end_soc=5;
  }
  
  delay(5);
}
