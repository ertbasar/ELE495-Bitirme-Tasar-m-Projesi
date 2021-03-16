#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <IRremote.h>
#include <TimerOne.h>
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
int a=0;
int kes=0;
int girdi=0;
int k=0;
unsigned long eski_zaman;
/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/

void setup(void)
{
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  
//  cli();
//  sei();
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
//  cli();
//  /* Ayarlamaların yapılabilmesi için öncelikle kesmeler durduruldu */
//
//  /* Timer1 kesmesi saniyede bir çalışacak şekilde ayarlanacaktır (1 Hz)*/
//  TCCR1A = 0;
//  TCCR1B = 0;
//  TCNT1  = 0;
//  OCR1A = 46872;
//  //OCR1A = 15534;
//  /* Bir saniye aralıklar için zaman sayıcısı ayarlandı */
//  TCCR1B |= (1 << WGM12);
//  /* Adımlar arasında geçen süre kristal hızının 1024'e bölümü olarak ayarlandı */
//  TCCR1B |= (1 << CS12) | (1 << CS10);
//  TIMSK1 |= (1 << OCIE1A);
//  /* Timer1 kesmesi aktif hale getirildi */
//
//  sei();
  /* Timer1 kesmesinin çalışabilmesi için tüm kesmeler aktif hale getirildi */
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
//ISR(TIMER4_COMPA_vect){
//  kes=1;
//   Serial.println(F("\nnmdfkjjfwkjfk\n"));
//   TIMSK4 &= ~(1 << OCIE4A);
//}

//void inter(void){
////Timer1.start();
//if(k!=1){
//kes=1;
//Timer1.stop();
//}
//Serial.println(k);
//Serial.println(F("\nnmdfkjjfwkjfk\n"));
////goto mod_secimi;
//k=0;
//}
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
  while(a==0&&!(system==3)) {
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
  a=1;
 mod_secimi:
 digitalWrite(5,HIGH);
 digitalWrite(6,HIGH);
 eski_zaman=0;
 kes=0;
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
 digitalWrite(5,HIGH);
 digitalWrite(6,LOW);
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
  digitalWrite(5,LOW);
 digitalWrite(6,LOW);
}
if((!basildi)&&(euler.x()>=280&&euler.x()<350)&&(euler.y()>=75&&euler.y()<90)&&(euler.z()>=170||euler.z()<10)){
  basildi=true;
  Serial.print(F("\nkanalazal"));
  irsend.sendRaw(rawkanalaz,68,38);
  digitalWrite(5,LOW);
 digitalWrite(6,LOW);
}
if((!basildi)&&(euler.x()>=280&&euler.x()<350)&&(euler.y()>=-10&&euler.y()<10)&&(euler.z()>=-110&&euler.z()<-70)){
  basildi=true;
  Serial.print(F("\nsesart"));
  irsend.sendRaw(rawsesart,68,38);
    digitalWrite(5,LOW);
 digitalWrite(6,LOW);
}
if((!basildi)&&(euler.x()>=280&&euler.x()<350)&&(euler.y()>=-10&&euler.y()<10)&&(euler.z()>=70&&euler.z()<110)){
  basildi=true;
  Serial.print(F("\nsesazalt"));
  irsend.sendRaw(rawsesaz,68,38);
    digitalWrite(5,LOW);
 digitalWrite(6,LOW);
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
digitalWrite(5,HIGH);
 digitalWrite(6,LOW);
}
goto sag_mod;
}
else if((euler.x()>=280&&euler.x()<350)&&(euler.y()>=75&&euler.y()<90)&&(euler.z()>=170||euler.z()<10)){
  digitalWrite(5,LOW);
 digitalWrite(6,HIGH);
  delay(2000);
  girdi=0;
  kes=0;
  k=0;
  sol_mod:
  //delay(250);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
//  Serial.print(F("X: "));
//  Serial.print(euler.x());
//  Serial.print(F(" Y: "));
//  Serial.print(euler.y());
//  Serial.print(F(" Z: "));
//  Serial.print(euler.z());
//  Serial.print(F("\t\t"));
//  //delay(250);
//  Serial.print(F("sol_mod\n"));
  //imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  if((girdi<4)&&(!basildi)&&(euler.x()>=280&&euler.x()<350)&&(euler.y()>=-90&&euler.y()<-75)&&(euler.z()>=170||euler.z()<10)){
  basildi=true;
  Serial.print(F("\nkana1"));
  irsend.sendRaw(raw1,68,38);
  eski_zaman=millis();
  digitalWrite(5,LOW);
 digitalWrite(6,LOW);
  //Serial.print(eski_zaman);
//  k=1;
//  Timer1.initialize(3000000);
//  Timer1.attachInterrupt(inter);
//  cli();
//  /* Ayarlamaların yapılabilmesi için öncelikle kesmeler durduruldu */
//
//  /* Timer1 kesmesi saniyede bir çalışacak şekilde ayarlanacaktır (1 Hz)*/
//  TCCR1A = 0;
//  TCCR1B = 0;
//  TCNT1  = 0;
//  OCR1A = 46872;
//  //OCR1A = 15534;
//  /* Bir saniye aralıklar için zaman sayıcısı ayarlandı */
//  TCCR1B |= (1 << WGM12);
//  /* Adımlar arasında geçen süre kristal hızının 1024'e bölümü olarak ayarlandı */
//  TCCR1B |= (1 << CS12) | (1 << CS10);
//  TIMSK1 |= (1 << OCIE1A);
//  /* Timer1 kesmesi aktif hale getirildi */
//
//  sei();
////  OCR1A = 46872;
////  sei();
  girdi=girdi+1;
}
if((girdi<4)&&(!basildi)&&(euler.x()>=280&&euler.x()<350)&&(euler.y()>=75&&euler.y()<90)&&(euler.z()>=170||euler.z()<10)){
  basildi=true;
  Serial.print(F("\nkanal2"));
  irsend.sendRaw(raw2,68,38);
  eski_zaman=millis();
  digitalWrite(5,LOW);
 digitalWrite(6,LOW);
//cli();
//  /* Ayarlamaların yapılabilmesi için öncelikle kesmeler durduruldu */
//
//  /* Timer1 kesmesi saniyede bir çalışacak şekilde ayarlanacaktır (1 Hz)*/
//  TCCR1A = 0;
//  TCCR1B = 0;
//  TCNT1  = 0;
//  OCR1A = 46872;
//  //OCR1A = 15534;
//  /* Bir saniye aralıklar için zaman sayıcısı ayarlandı */
//  TCCR1B |= (1 << WGM12);
//  /* Adımlar arasında geçen süre kristal hızının 1024'e bölümü olarak ayarlandı */
//  TCCR1B |= (1 << CS12) | (1 << CS10);
//  TIMSK1 |= (1 << OCIE1A);
//  /* Timer1 kesmesi aktif hale getirildi */
//
//  sei();
//  OCR1A = 46872;
//  sei();
  //girdi=girdi+1;
}
if((girdi<4)&&(!basildi)&&(euler.x()>=280&&euler.x()<350)&&(euler.y()>=-10&&euler.y()<10)&&(euler.z()>=-110&&euler.z()<-70)){
  basildi=true;
  Serial.print(F("\nkanal3"));
  irsend.sendRaw(raw3,68,38);
  eski_zaman=millis();
  digitalWrite(5,LOW);
 digitalWrite(6,LOW);
//cli();
  /* Ayarlamaların yapılabilmesi için öncelikle kesmeler durduruldu */

  /* Timer1 kesmesi saniyede bir çalışacak şekilde ayarlanacaktır (1 Hz)*/
//  TCCR1A = 0;
//  TCCR1B = 0;
//  TCNT1  = 0;
//  OCR1A = 46872;
//  //OCR1A = 15534;
//  /* Bir saniye aralıklar için zaman sayıcısı ayarlandı */
//  TCCR1B |= (1 << WGM12);
//  /* Adımlar arasında geçen süre kristal hızının 1024'e bölümü olarak ayarlandı */
//  TCCR1B |= (1 << CS12) | (1 << CS10);
//  TIMSK1 |= (1 << OCIE1A);
//  /* Timer1 kesmesi aktif hale getirildi */
//
//  sei();
////  OCR1A = 46872;
////  sei();
 girdi=girdi+1;
}
if((girdi<4)&&(!basildi)&&(euler.x()>=280&&euler.x()<350)&&(euler.y()>=-10&&euler.y()<10)&&(euler.z()>=70&&euler.z()<110)){
  basildi=true;
  Serial.print(F("\nkanal4"));
  irsend.sendRaw(raw4,68,38);
  eski_zaman=millis();
  digitalWrite(5,LOW);
 digitalWrite(6,LOW);
//noInterrupts();
//cli();
//  /* Ayarlamaların yapılabilmesi için öncelikle kesmeler durduruldu */
//
//  /* Timer1 kesmesi saniyede bir çalışacak şekilde ayarlanacaktır (1 Hz)*/
//  TCCR4A = 0;
//  TCCR4B = 0;
//  TCNT4  = 0;
//  OCR4A = 46872;
//  //OCR4A = 15534;
//  /* Bir saniye aralıklar için zaman sayıcısı ayarlandı */
//  TCCR4B |= (1 << WGM12);
//  /* Adımlar arasında geçen süre kristal hızının 1024'e bölümü olarak ayarlandı */
//  TCCR4B |= (1 << CS12) | (1 << CS10);
//  TIMSK4 |= (1 << OCIE4A);
//  /* Timer1 kesmesi aktif hale getirildi */
//
//  sei();
  //interrupts();
//  OCR1A = 46872;
//  sei();
  girdi=girdi+1;
}
if((girdi<4)&&(!basildi)&&(euler.x()>=330||euler.x()<30)&&(euler.y()>=-10&&euler.y()<10)&&(euler.z()>=-10&&euler.z()<10)){
  basildi=true;
  Serial.print(F("\nkanal5"));
 irsend.sendRaw(raw5,68,38);
 eski_zaman=millis();
 digitalWrite(5,LOW);
 digitalWrite(6,LOW);
//cli();
//  /* Ayarlamaların yapılabilmesi için öncelikle kesmeler durduruldu */
//
//  /* Timer1 kesmesi saniyede bir çalışacak şekilde ayarlanacaktır (1 Hz)*/
//  TCCR1A = 0;
//  TCCR1B = 0;
//  TCNT1  = 0;
//  OCR1A = 46872;
//  //OCR1A = 15534;
//  /* Bir saniye aralıklar için zaman sayıcısı ayarlandı */
//  TCCR1B |= (1 << WGM12);
//  /* Adımlar arasında geçen süre kristal hızının 1024'e bölümü olarak ayarlandı */
//  TCCR1B |= (1 << CS12) | (1 << CS10);
//  TIMSK1 |= (1 << OCIE1A);
//  /* Timer1 kesmesi aktif hale getirildi */
//
//  sei();
////  OCR1A = 46872;
////  sei();
  girdi=girdi+1;
}
if((girdi<4)&&(!basildi)&&(euler.x()>=150&&euler.x()<210)&&(euler.y()>=-10&&euler.y()<10)&&(euler.z()>=-10&&euler.z()<10)){
  basildi=true;
  Serial.print(F("\nkanal6"));
  irsend.sendRaw(raw6,68,38);
  eski_zaman=millis();
  digitalWrite(5,LOW);
 digitalWrite(6,LOW);
//cli();
//  /* Ayarlamaların yapılabilmesi için öncelikle kesmeler durduruldu */
//
//  /* Timer1 kesmesi saniyede bir çalışacak şekilde ayarlanacaktır (1 Hz)*/
//  TCCR1A = 0;
//  TCCR1B = 0;
//  TCNT1  = 0;
//  OCR1A = 46872;
//  //OCR1A = 15534;
//  /* Bir saniye aralıklar için zaman sayıcısı ayarlandı */
//  TCCR1B |= (1 << WGM12);
//  /* Adımlar arasında geçen süre kristal hızının 1024'e bölümü olarak ayarlandı */
//  TCCR1B |= (1 << CS12) | (1 << CS10);
//  TIMSK1 |= (1 << OCIE1A);
//  /* Timer1 kesmesi aktif hale getirildi */
//
//  sei();
////  OCR1A = 46872;
////  sei();
  girdi=girdi+1;
}

if((euler.x()>=275&&euler.x()<315)&&(euler.y()>=-10&&euler.y()<10)&&(euler.z()>=-10&&euler.z()<10)){
  //0
//  Serial.print(F("\nresetlendi"));
//  Serial.print(girdi);
  //Serial.print(F("\nsol_mod"));
  basildi=false;
 digitalWrite(5,LOW);
 digitalWrite(6,HIGH);
}
if(girdi==3){
// cli();
  kes=0;
  Serial.print(F("\ncikis"));
goto mod_secimi;
}
if(((millis()-eski_zaman)>=3000)&&(eski_zaman!=0)){
 Serial.print(F("\nucsaniye")); 
goto mod_secimi;  
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
