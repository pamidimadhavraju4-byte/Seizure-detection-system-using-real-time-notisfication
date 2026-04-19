#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPS.h>
#include <LiquidCrystal.h>

MAX30105 particleSensor;
Adafruit_MPU6050 mpu;
TinyGPS gps;

const int rs=8,en=9,d4=10,d5=11,d6=12,d7=13;
LiquidCrystal lcd(rs,en,d4,d5,d6,d7);

const byte RATE_SIZE=4;

byte rates[RATE_SIZE];
byte rateSpot=0;

long lastBeat=0;

float beatsPerMinute;
int beatAvg;

int tempF,spo2;
int fs=0,fall=0,panic_s;
int xval,yval,zval,magnitude;

float flat=0,flon=0;

int ps=7,buz=6;
long int prv=0,prv1=0;

int abnormal=0;

void read_gps(){
  bool newData=false;
  unsigned long chars;
  unsigned short sentences,failed;

  for(unsigned long start=millis();millis()-start<1000;){
    while(Serial.available()){
      char c=Serial.read();
      if(gps.encode(c)) newData=true;
    }
  }

  if(newData){
    unsigned long age;
    gps.f_get_position(&flat,&flon,&age);
  }
}

void setup(){
  Serial.begin(9600);
  lcd.begin(16,2);
  lcd.print("WELCOME");

  particleSensor.begin(Wire,I2C_SPEED_FAST);
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
  particleSensor.enableDIETEMPRDY();

  mpu.begin();

  pinMode(ps,INPUT_PULLUP);
  pinMode(buz,OUTPUT);
}

void loop(){
  long irValue=particleSensor.getIR();

  if(irValue<50000){
    if(fs==1){
      fs=0;
      lcd.clear();
      lcd.print("No finger?");
      beatAvg=0;
    }
  }
  else{
    if(fs==0){
      fs=1;
      lcd.clear();
      lcd.print("Reading..");
    }

    if(checkForBeat(irValue)){
      long delta=millis()-lastBeat;
      lastBeat=millis();

      beatsPerMinute=60/(delta/1000.0);

      if(beatsPerMinute<255 && beatsPerMinute>20){
        rates[rateSpot++]=(byte)beatsPerMinute;
        rateSpot%=RATE_SIZE;

        beatAvg=0;
        for(byte x=0;x<RATE_SIZE;x++) beatAvg+=rates[x];
        beatAvg/=RATE_SIZE;
      }
    }
  }

  if(beatAvg>40){
    tempF=particleSensor.readTemperatureF();

    if(beatAvg>140) spo2=map(beatAvg,140,255,80,50);
    else if(beatAvg>=40 && beatAvg<60) spo2=map(beatAvg,40,60,70,98);
    else spo2=map(beatAvg,60,140,100,80);

    lcd.clear();
    lcd.print("H:"+String(beatAvg)+" S:"+String(spo2)+" T:"+String(tempF));
  }
  else{
    tempF=particleSensor.readTemperatureF();
    beatAvg=0;
    spo2=0;
  }

  sensors_event_t a,g,temp;
  mpu.getEvent(&a,&g,&temp);

  xval=a.acceleration.x;
  yval=a.acceleration.y;
  zval=a.acceleration.z;

  magnitude=sqrt(xval*xval+yval*yval+zval*zval);

  fall=0;
  if(xval>5||xval<-5||yval>5||yval<-5||magnitude>20) fall=1;

  if(digitalRead(ps)==0) panic_s=1;

  lcd.setCursor(0,1);
  lcd.print("F:"+String(fall)+" P:"+String(panic_s));

  if(fall==1 || panic_s==1 || beatAvg>90 || (spo2>60 && spo2<80) || tempF>100)
    abnormal=1;
  else
    abnormal=0;

  if(millis()-prv1>5000){
    if(abnormal==1) digitalWrite(buz,1);
  }

  if(millis()-prv1>6000){
    prv1=millis();
    if(abnormal==1) digitalWrite(buz,0);
  }

  if(millis()-prv>30000){
    read_gps();
    read_gps();
    prv=millis();

    digitalWrite(buz,1); delay(200);
    digitalWrite(buz,0); delay(200);
    digitalWrite(buz,1); delay(200);
    digitalWrite(buz,0); delay(200);

    Serial.println("465109,QBCUELO3NO8P93FD,0,0,SRC 24G,src@internet,"+
    String(beatAvg)+","+String(spo2)+","+String(tempF)+","+String(fall)+","+
    String(panic_s)+","+String(16.2334)+","+String(80.5509)+",\n");
  }
}
