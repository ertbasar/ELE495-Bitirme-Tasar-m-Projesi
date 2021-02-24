#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <IRremote.h>
IRsend irsend;
/* This driver reads raw data from the BNO055

   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   History
   =======
   2015/MAR/03  - First release (KTOWN)
*/
//irsend.sendRaw(rawkart,68,38);
//1 e basma kodu
unsigned int raw1[68] = {4400,4500,550,1700,500,1700,500,1700,550,550,550,600,450,650,500,600,500,600,500,1700,550,1700,450,1750,500,600,550,550,500,650,500,600,500,600,500,600,500,600,500,1750,500,600,500,600,500,600,500,600,500,600,550,1700,500,1700,500,650,450,1750,500,1700,500,1700,550,1700,500,1700,500,};
//ac kapa kodu 
//unsigned int raw[68] = {4450,4450,550,1700,500,1700,550,1700,500,600,550,550,500,600,550,550,550,550,550,1700,550,1650,550,1650,550,600,550,550,550,550,600,500,600,500,600,500,550,1700,550,550,600,500,650,450,550,550,550,600,550,550,550,1650,600,500,600,1650,550,1650,600,1600,550,1700,550,1650,600,1600,650,};
// 2 butonu
unsigned int raw2[68] = {4400,4500,500,1750,500,1700,500,1750,500,600,500,600,500,600,500,650,450,650,500,1700,500,1750,500,1700,500,600,500,650,450,650,500,600,500,600,500,1750,500,600,500,1700,500,650,450,650,500,600,500,600,500,650,450,650,500,1700,500,600,500,1750,500,1700,500,1750,500,1700,500,1750,450,};
/* Set the delay between fresh samples */
// 3 butonu
unsigned int raw3[68] = {4400,4500,550,1700,450,1750,500,1700,500,650,450,650,500,600,550,550,500,600,500,1750,500,1700,500,1700,550,550,550,600,500,600,500,600,550,550,600,500,500,1750,450,1750,550,550,500,600,500,650,500,600,550,550,500,1700,550,550,600,550,500,1700,550,1650,500,1750,450,1750,500,1700,650,};
// 4 butonu
unsigned int raw4[68] = {4400,4550,500,1700,500,1700,500,1750,450,650,550,550,500,600,550,550,500,600,550,1700,550,1650,500,1700,550,600,500,600,500,600,550,550,550,550,500,600,550,600,500,600,550,1650,500,600,550,550,550,600,550,550,500,1700,500,1700,550,1700,550,550,500,1700,500,1750,450,1750,500,1700,500,};
// 5 butonu
unsigned int raw5[68] = {4450,4500,450,1750,600,1600,500,1750,500,600,500,600,550,550,600,500,500,600,550,1700,550,1650,550,1650,550,600,550,550,550,550,550,550,550,550,600,1650,550,550,600,500,550,1650,600,550,500,600,600,500,600,500,600,500,600,1650,600,1600,600,500,600,1650,500,1700,550,1650,600,1650,500,};
// 6 butonu
unsigned int raw6[68] = {4450,4450,500,1700,600,1650,500,1700,500,600,500,600,500,600,500,650,500,600,500,1700,500,1700,500,1750,450,650,500,600,500,600,500,600,500,600,550,600,550,1650,500,600,500,1700,550,600,450,650,500,600,500,600,500,1700,500,650,500,1700,550,550,550,1650,500,1750,500,1700,500,1700,550,};
// kanal arttır
unsigned int rawkanart[68] = {4400,4500,500,1750,450,1750,500,1700,500,650,450,650,450,650,500,600,500,600,500,1750,450,1750,500,1700,500,600,500,650,450,650,450,650,500,600,500,600,500,1750,450,650,450,650,500,1700,500,600,500,650,450,650,500,1700,500,600,500,1750,450,1750,500,600,500,1700,500,1750,500,1700,500,};
// kanal azalt
unsigned int rawkanalaz[68] = {4450,4500,500,1700,500,1750,450,1750,500,600,500,600,500,650,450,650,450,650,500,1700,500,1750,450,1750,500,600,500,600,500,600,500,650,450,650,450,650,500,600,500,600,500,600,500,1750,450,650,500,600,500,600,500,1750,450,1750,500,1700,500,1750,450,650,500,1700,500,1700,500,1750,500,};
// ses arttır
unsigned int rawsesart[68] = {4450,4450,600,1650,550,1650,550,1650,550,550,600,550,500,600,550,550,550,550,600,1600,600,1650,600,1600,600,500,600,500,600,550,550,550,550,550,550,1650,600,1650,550,1650,550,550,600,500,600,500,600,550,600,500,550,550,550,550,600,500,600,1650,600,1600,550,1650,600,1650,600,1600,550,};
// ses azalt
unsigned int rawsesaz[68] = {4550,4350,550,1700,550,1650,550,1650,600,550,500,600,550,500,600,550,550,550,600,1650,600,1600,500,1700,600,500,600,550,600,500,600,500,550,550,550,1650,550,1700,600,500,550,1650,650,450,600,550,550,550,600,500,550,550,600,500,600,1650,550,550,550,1650,550,1650,650,1600,550,1650,600,};

#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);
bool basildi=false;

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  Serial.println(F("Orientation Sensor Raw Data Test")); Serial.println(F(""));

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print(F("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print(F("Current Temperature: "));
  Serial.print(temp);
  Serial.println(F(" C"));
  Serial.println(F(""));

  bno.setExtCrystalUse(true);

  Serial.println(F("Calibration status values: 0=uncalibrated, 3=fully calibrated"));
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
//  Serial.print("X: ");
//  Serial.print(euler.x());
//  Serial.print(" Y: ");
//  Serial.print(euler.y());
//  Serial.print(" Z: ");
//  Serial.print(euler.z());
//  Serial.print("\t\t");

  /*
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  Serial.print("qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);
  Serial.print("\t\t");
  */

  /* Display calibration status for each sensor. */
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  while(!(system==3)) {
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print(F("CALIBRATION: Sys="));
  Serial.print(system, DEC);
  Serial.print(F(" Gyro="));
  Serial.print(gyro, DEC);
  Serial.print(F(" Accel="));
  Serial.print(accel, DEC);
  Serial.print(F(" Mag="));
  Serial.println(mag, DEC);
  }
  Serial.print(F("Fully Calibrated\n"));
 mod_secimi:
 Serial.print(F("Mod Secimi\n"));
 delay(1000);
 imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print(F("X: "));
  Serial.print(euler.x());
  Serial.print(F(" Y: "));
  Serial.print(euler.y());
  Serial.print(F(" Z: "));
  Serial.print(euler.z());
  Serial.print(F("\t\t"));
if((euler.x()>=280&&euler.x()<350)&&(euler.y()>=-90&&euler.y()<-75)&&(euler.z()>=170||euler.z()<10)){
  sag_mod:
  delay(1000);
  
 imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print(F("sag_mod\n"));
  Serial.print(F("X: "));
  Serial.print(euler.x());
  Serial.print(F(" Y: "));
  Serial.print(euler.y());
  Serial.print(F(" Z: "));
  Serial.print(euler.z());
  Serial.print(F("\t\t"));
  if((!basildi)&&(euler.x()>=280&&euler.x()<350)&&(euler.y()>=-90&&euler.y()<-75)&&(euler.z()>=170||euler.z()<10)){
  basildi=true;
  Serial.print(F("\nkanalart"));
  irsend.sendRaw(rawkanart,68,38);
}
if((!basildi)&&(euler.x()>=280&&euler.x()<350)&&(euler.y()>=75&&euler.y()<90)&&(euler.z()>=170||euler.z()<10)){
  basildi=true;
  Serial.print(F("\nkanalazal"));
  irsend.sendRaw(rawkanalaz,68,38);
}
if((!basildi)&&(euler.x()>=280&&euler.x()<350)&&(euler.y()>=-10&&euler.y()<10)&&(euler.z()>=-110&&euler.z()<-70)){
  basildi=true;
  Serial.print(F("\nsesart"));
  irsend.sendRaw(rawsesart,68,38);
}
if((!basildi)&&(euler.x()>=280&&euler.x()<350)&&(euler.y()>=-10&&euler.y()<10)&&(euler.z()>=70&&euler.z()<110)){
  basildi=true;
  Serial.print(F("\nsesazalt"));
  irsend.sendRaw(rawsesaz,68,38);
}
if((!basildi)&&(euler.x()>=330||euler.x()<30)&&(euler.y()>=-10&&euler.y()<10)&&(euler.z()>=-10&&euler.z()<10)){
  basildi=true;
  //exit
  goto mod_secimi;
}
if((euler.x()>=275&&euler.x()<315)&&(euler.y()>=-10&&euler.y()<10)&&(euler.z()>=-10&&euler.z()<10)){
  //0
  Serial.print(F("\nresetlendi"));
  basildi=false;
}
goto sag_mod;
}
else if((euler.x()>=280&&euler.x()<350)&&(euler.y()>=75&&euler.y()<90)&&(euler.z()>=170||euler.z()<10)){
  sol_mod:
  //delay(1000);
  //Serial.print(F("sol_mod\n"));
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  if((!basildi)&&(euler.x()>=280&&euler.x()<350)&&(euler.y()>=-90&&euler.y()<-75)&&(euler.z()>=170||euler.z()<10)){
  basildi=true;
  Serial.print(F("\nkana1"));
  irsend.sendRaw(raw1,68,38);
}
if((!basildi)&&(euler.x()>=280&&euler.x()<350)&&(euler.y()>=75&&euler.y()<90)&&(euler.z()>=170||euler.z()<10)){
  basildi=true;
  Serial.print(F("\nkanal2"));
  irsend.sendRaw(raw2,68,38);
}
if((!basildi)&&(euler.x()>=280&&euler.x()<350)&&(euler.y()>=-10&&euler.y()<-10)&&(euler.z()>=-110&&euler.z()<-70)){
  basildi=true;
  Serial.print(F("\nkanal3"));
  irsend.sendRaw(raw3,68,38);
}
if((!basildi)&&(euler.x()>=280&&euler.x()<350)&&(euler.y()>=-10&&euler.y()<10)&&(euler.z()>=70&&euler.z()<110)){
  basildi=true;
  Serial.print(F("\nkanal4"));
  irsend.sendRaw(raw4,68,38);
}
if((!basildi)&&(euler.x()>=330||euler.x()<30)&&(euler.y()>=-10&&euler.y()<10)&&(euler.z()>=-10&&euler.z()<10)){
  basildi=true;
  Serial.print(F("\nkanal5"));
  irsend.sendRaw(raw5,68,38);
}
if((!basildi)&&(euler.x()>=150&&euler.x()<210)&&(euler.y()>=-10&&euler.y()<10)&&(euler.z()>=-10&&euler.z()<10)){
  basildi=true;
  Serial.print(F("\nkanal6"));
  //irsend.sendRaw(raw6,68,38);
}

if((euler.x()>=275&&euler.x()<315)&&(euler.y()>=-10&&euler.y()<10)&&(euler.z()>=-10&&euler.z()<10)){
  //0
  Serial.print(F("\nresetlendi"));
  Serial.print(F("\nsol_mod"));
  basildi=false;
}
goto sol_mod;
}
//if((euler.x()>=230&&euler.x()<310)){
//  basildi1=false;
//  basildi3=false;
//}
//
//if((!basildi3)&&(euler.x()>=150&&euler.x()<210)){
//  basildi3=true;
//  Serial.print("\nbasildi3 aktif");
//  irsend.sendRaw(raw2,68,38);
//}
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
