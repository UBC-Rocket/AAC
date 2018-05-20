/* Apogee.py script conversion for Hanna 2018 Launch*/

#include <EEPROM.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MatrixMath.h>
#include "SD.h"

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10

Adafruit_BME280 bme; // I2C
//Adafruit_BMP280 bme(BMP_CS); // hardware SPI
//Adafruit_BMP280 bme(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

int addr = 0; //initial address we're writing in EEPROM to
const int chipSelect = 12; //For SD card

//Global Vars
//amax in feet, tmax in seconds, drag time scale, seconds, represents the time for rocket to hit terminal velocity assuming it is in free fall, nose down, deceleration ft/s^2
float pars[4] = {9115.7, 15.91, 15.97, 40.81}; 
float drvs[4];
float wres[4];
float pstp[4];
float dstb[4];
float wgtm[4][4]= {{1.0,0.0,0.0,0.0},
                     {0.0,1.0,0.0,0.0},
                     {0.0,0.0,1.0,0.0},
                     {0.0,0.0,0.0,1.0}};
float covm[4][4] = {{1.0,0.0,0.0,0.0},
                     {0.0,1.0,0.0,0.0},
                     {0.0,0.0,1.0,0.0},
                     {0.0,0.0,0.0,1.0}};

float amax  = pars[0];
float tmax  = pars[1];
float tdrag = pars[2];
float decel = pars[3];

//altimeter measurement variance (error squared)
float altVar = 1.0;

//start time for Kalman filter
float tstart = 0.0;

//time variable for clock in arduino
unsigned long time;

//Real and predicted altitude
float RealAlt;
float altPred;

File apogeeFile;
const int ntimes = 5;
  
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.print("Initialzing Apogee Script");
  pinMode(4, OUTPUT); //indicator led
  
  float hz = 20.0;
  dstb[0] = sq((1.0/hz))* 0;
  dstb[1] = sq((1.0/hz))* 0;
  dstb[2] = sq((1.0/hz))* 0;
  dstb[3] = sq((1.0/hz))* 0;

  //initial covariances (don't change these values)
  covm[0][0] = sq(10.0);
  covm[1][1] = sq(1.0);
  covm[2][2] = sq(1.0);
  covm[3][3] = sq(1.0);

  pinMode(chipSelect, OUTPUT);
 
  // create a new file
  char filename[] = "ACTIVEAPOGEEPREDICTION.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      apogeeFile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }
}


float altFunc(unsigned long time) {
  
  float altitude  = amax + decel * tdrag*tdrag * log(cos((time-tmax)/tdrag));
  return altitude;
}

void kalmanStep(int ktime){
//# current time and altitude
    //parameters and errors tables
  //float ptable [100][4];
  //float etable [100][4];
  float pred [100];
  float resd [100];
//before start time, or after tmax, just copy info
  /*if (time < tstart || time > pars[1]){
    pred[ktime] = altFunc(time);
    resd[ktime] = 0.0;
    for (int j = 0; j <4; j++){
      //ptable[ktime][j] = pars[j];
      etable[ktime][j] = sqrt(covm[j][j]);
    }
  }*/
  

  //predicted altitude
  altPred = altFunc(time);
  pred[ktime] = altPred;
  
  //residual
  float res = RealAlt - altPred;
  resd[ktime] = res;
  
  //derivatives
  drvs[0] = 1.0;
  drvs[1] = decel * tdrag * tan((time-tmax)/tdrag);
  float t1 = decel * tdrag*2.0  * log(cos((time-tmax)/tdrag));
  float t2 = decel * (time-tmax) * tan((time-tmax)/tdrag);
  drvs[2] = t1+t2;
  drvs[3] = tdrag * tdrag * log(cos((time-tmax)/tdrag));
  
  //weighted residuals array
  wres[0] = (res * drvs[0]) / altVar;
  wres[1] = (res * drvs[1]) / altVar;
  wres[2] = (res * drvs[2]) / altVar;
  wres[3] = (res * drvs[3]) / altVar;

  //degrade covariance matrix
  for (int i=0; i < 4; i++){
      covm[i][i] += dstb[i];
  }
  
  //invert into weight matrix
  //wgtm = covm.I;    
  Matrix.Invert((float*)covm, 4);
  Matrix.Copy((float*)covm, 4, 4, (float*)wgtm);
  Matrix.Invert((float*)covm, 4);
      
  //add information from new measurement
  for (int z = 0; z< 4; z++){
      for (int y = 0; y < 4; y++){
          wgtm[z][y] += drvs[z]*drvs[y] / altVar;
      }
  }      
  //invert into covariance
 //covm = wgtm.I;
  Matrix.Invert((float*)wgtm, 4);
  Matrix.Copy((float*)wgtm, 4, 4, (float*)covm);
  Matrix.Invert((float*)wgtm, 4);

  //Matrix.Print((float*)covm, 4, 4, "covm");
  //Matrix.Print((float*)wgtm, 4, 4, "wgtm");
        
   //add information from new measurement by the dot product
  Matrix.Multiply((float*)covm, (float*) wres, 4,4,1, (float*)pstp);
  //Matrix.Print((float*)pstp, 4, 1, "pstp"); 
        
 //update parameters, store info into the parameter and error tables
 for (int k = 0; k<4; k++) {
     pars[k] += pstp[k];
     //ptable[ktime][k] = pars[k];
     //etable[ktime][k] = sqrt(covm[k][k]);
 } 

 //WRITE OUR INFO TO EEPROM
 //CHANGE THIS SECTION IF YOU WANT TO WRITE TO SD INSTEAD
 /*int val = pars[0];
 byte four = (val & 0xFF);
 byte three = ((val >> 8) & 0xFF);

    //We have 1kB of EEPROM memory equivalent to the uno
  if (addr < EEPROM.length()) {  
    EEPROM.write(addr, four);
    EEPROM.write(addr+1, three);
    addr = addr + 2;
  }
  Serial.print("Predicted:");
  Serial.print(altPred);
  Serial.print("\n");
  Serial.print("Residual:");
  Serial.print(res);
  Serial.print("\n");
  Serial.print("\n");
  Serial.print("Kalman:");
  Serial.print(pars[0]);
  Serial.print("\n");*/
    //apogeeFile.print(", ");    
    apogeeFile.print(time);
    apogeeFile.print(", ");    
    apogeeFile.print(RealAlt);
    apogeeFile.print(", ");    
    apogeeFile.print(altPred);
    apogeeFile.print(", ");    
    apogeeFile.print(pars[0]);
    apogeeFile.println();
    apogeeFile.flush();
    delay(200);
}


void loop() {
  // put your main code here, to run repeatedly:
  //Initialize clock 
  time = (millis()/1000);
  digitalWrite(4,HIGH); //indication LED high
  
  //Check the barometer for reading
  //Later this will be recieved from apogee detect board
  RealAlt = bme.readAltitude(1013.25)*3.281; //convert from meters to feet
  //Serial.println(RealAlt);
  
  //Threshold for recording our data
  if (RealAlt >= -2000) {
  for (int ktime = 0; ktime < ntimes; ktime++){
    kalmanStep(ktime); 
  }
  //Intermitent delay
  
 }
 //counter++;
 //apogeeFile.close();
 delay(1000);
}

/*Record after 2000 ft*/
