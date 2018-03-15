/*
 * UBC ROCKET - Active Apogee Control State Machine 2018
 */
 
#include libs

// State Definitions
#define STANDBY           0 
#define PRE_BURNOUT       1 
#define PREDICT_APOGEE    2 
#define DEPLOY_BRAKES     3 
#define RETRACT_BRAKES    4 
#define POST_APOGEE       5 

// Apogee prediction parameters
#define NUM_STEPS         5 // number of readings averaged for kalman filter
// add other parameters from python prediction code

// Motor control pins - control mechanism TBD
const int MOTOR_CW = 7;  // turns motor clockwise to open brakes (or some equivalent)
const int MOTOR_CCW = 8; // turns counterclockwise to close brakes (or some equivalent)

// Data from avionics 
float altitude, angle;
// what data are we getting from avionics?

void setup() {

  Serial.begin(9600);
  Serial.println("Initializing");

  // Initialize pins for motor circuit as outputs
  pinMode(MOTOR_CW, OUTPUT);
  pinMode(MOTOR_CCW, OUTPUT);
  
  // Initialize any sensors/comm with avionics  
  
}

void loop(){

  switch (state){
      
    case STANDBY:
      // do nothing until launch detected (method TBD - likely signaled by avionics or using avionics data)

    case PRE_BURNOUT:
      // apogee prediction is unstable during powered ascent -- do nothing until burnout is detected
      
    case PREDICT_APOGEE:
      // use apogee prediction functions and avionics data to determine next state - DEPLOY/RETRACT_BRAKES
      
    case DEPLOY_BRAKES:
      // deploy airbrakes to lower predicted apogee -- figure out deployment control method
      // return to predict apogee state
            
    case RETRACT_BRAKES:
      // retract airbrakes to lower drag -- figure out control
      // return to predict apogee state
      
    case POST_APOGEE:
      // once apogee is detected, air brakes become useless and nothing we can do will help -- stop trying
          
}
