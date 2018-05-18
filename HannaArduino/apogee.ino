/* Apogee.py script conversion for Hanna 2018 Launch*/

#include <EEPROM.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <MatrixMath.h>
#include "SD.h"
#include <Canbus.h>
#include <iostream>
#include <fstream>

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11 
#define BMP_CS 10

Adafruit_BMP280 bme; // I2C
//Adafruit_BMP280 bme(BMP_CS); // hardware SPI
//Adafruit_BMP280 bme(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);

int addr = 0; //initial address we're writing in EEPROM to
const int chipSelect = 10; //For SD card
File apogeeFile;

//Global Vars
//amax in feet, tmax in seconds, drag time scale, seconds, represents the time for rocket to hit terminal velocity assuming it is in free fall, nose down, deceleration ft/s^2
float pars[4] = {9115.7, 15.91, 15.97, 40.81}; 
float drvs[4];
float wres[4];
float pstp[4];
float dstb[4];
float wgtm[4][4] =  {{1.0,0.0,0.0,0.0},
                     {0.0,1.0,0.0,0.0},
                     {0.0,0.0,1.0,0.0},
                     {0.0,0.0,0.0,1.0}};
float covm[4][4] =  {{1.0,0.0,0.0,0.0},
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
// unsigned long time;

//initialization
unsigned long time = (millis()/1000);
digitalWrite(4,HIGH); //indication LED high
//Check the barometer for reading
//Later this will be recieved from apogee detect board
float RealAlt = bme.readAltitude(1013.25)*3.281; //convert from meters to feet


//Real and predicted altitude
float RealAlt;
float altPred;

const int ntimes = 5;
  
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.print("Initialzing Apogee Script");
  pinMode(4, OUTPUT); //indicator led
  Serial.println(F("BMP280 test"));
  
  if (!bme.begin()) {  
    Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    while (1);
  }
  
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
 
 // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  
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
  
  if (! apogeeFile) {
    error("couldnt create file");
  }
  
  Serial.print("Logging to: ");
  Serial.println(filename);
}

void error(char *str)
{
  Serial.print("error: ");
  Serial.println(str);
  
  // red LED indicates error
  //digitalWrite(redLEDpin, HIGH);
  
  while(1);
}

float altFunc(unsigned long time) {
  
  float altitude  = amax + decel * tdrag*tdrag * log(cos((time-tmax)/tdrag));
  return altitude;
}

float kalmanStep(int ktime){
//# current time and altitude
    //parameters and errors tables
  float ptable [100][4];
  float etable [100][4];
  float pred [100];
  float resd [100];
//before start time, or after tmax, just copy info
  if (time < tstart || time > pars[1]){
    pred[ktime] = altFunc(time);
    resd[ktime] = 0.0;
    for (int j = 0; j <4; j++){
      ptable[ktime][j] = pars[j];
      etable[ktime][j] = sqrt(covm[j][j]);
    }
  }
  

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
     ptable[ktime][k] = pars[k];
     etable[ktime][k] = sqrt(covm[k][k]);
 } 

 //WRITE OUR INFO TO EEPROM
 //CHANGE THIS SECTION IF YOU WANT TO WRITE TO SD INSTEAD
 int val = pars[0];
 byte four = (val & 0xFF);
 byte three = ((val >> 8) & 0xFF);

    //We have 1kB of EEPROM memory equivalent to the uno
  if (addr < EEPROM.length()) {  
    EEPROM.write(addr, four);
    EEPROM.write(addr+1, three);
    addr = addr + 2;
  }
  apogeeFile.print(", ");    
  apogeeFile.print(time);
  apogeeFile.print(", ");    
  apogeeFile.print(pars[0]);

  //Return max altitude
  return(pars[0]);
}

//Function that looks up altitude diff for each time run
float lookUpAB(time){
  //set a constant actuation time and lookup table file "inLUT"
  const int actuateTime = 3;
  //simulation has 383 lines, starting from t=0
  int table = [1149];
  ifstream inLUT("formatted_data2.txt");
  if(inLUT.is_open()){
    //load all values into "table" var
    for(int i=0; i<1149; i++){
      i >> table[i];
    } 
  }

  return(table[time]);

  //lookup logic here
  continue;
}


void loop() {
  float maxAlt;
  //state mc
  switch (curr_state){
    case STANDBY:
      //Threshold for recording our data
      if (RealAlt >= 2000) {
        next_state = PREDICT_APOGEE;
        //Intermitent delay
        delay(2000);
      }

    case PREDICT_APOGEE:
      for (int ktime = 0; ktime < ntimes; ktime++){
        maxAlt = kalmanStep(ktime);
        //Deployment conditional (replace rs comparison with val of actuation effect via lookup table (fn of time))
        if(abs(maxAlt - 10000) >= lookUpAB(time)){
          //Print to console/write to memory "actuate"
          next_state = RETRACT_BRAKES;
          break;
        }
      }

    case DEPLOY_BRAKES:
      //Some logic to write to memory
      next_state = PREDICT_APOGEE;
      
    case RETRACT_BRAKES:
      //Some logic to write to memory
      next_state = PREDICT_APOGEE;
      
    case POST_APOGEE:
      // once apogee is detected, air brakes become useless and nothing we can do will help -- stop trying
      end_signal = 1;
  }

  curr_state = next_state;
}
