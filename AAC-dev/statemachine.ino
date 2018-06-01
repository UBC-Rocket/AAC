#include <Stepper.h>

// State Definitions
#define STANDBY           0 
#define PREDICT_APOGEE    1 
#define DEPLOY_BRAKES     2 
#define RETRACT_BRAKES    3 
#define POST_APOGEE       4 

// Apogee prediction parameters
#define NUM_STEPS         20 // number of readings averaged for kalman filter
// add other parameters from python prediction code

#define MIN_DEPLOY_TIME             3500 // min time in seconds before moving to predict apogee state (sim data when we get to mach 0.5)
#define MAX_BRAKE_TIME              3 // defined threshold to retract air brakes - we probably need this as a failsafe
#define MIN_BRAKE_TIME              1 // defined threshold below which air brake deployment has no effect on flight
#define LOW_DRAG_DEPLOY_APOGEE      9900 // defined threshold to retract in low drag situation with uncertainty
#define HIGH_DRAG_DEPLOY_APOGEE     10200 // defined threshold to deploy in high drag situation with uncertainty
#define MAX_TILT                    30 // max angle above which brake functionality is no longer reliable

// Motor control pins 6,7,8,9 on the Arduino 2560 (MEGA)
const int in1Pin = 6;
const int in2Pin = 7;
const int in3Pin = 8;
const int in4Pin = 9;
const int stepsPerRevolution = 200; //set by properties of stepper

Stepper airbrakes (stepsPerRevolution, in1Pin, in2Pin, in3Pin, in4Pin);

int stepCount = 0;

// Time variables
unsigned long curr_time, setup_time, burnout_time, deploy_brakes_time, retract_brakes_time;

// Data from avionics 
float curr_alt, curr_accel, curr_tilt;
bool post_burnout, post_apogee;

// State control variables
int curr_state, next_state;
bool deploy_signal, retract_signal, end_signal;
float pred_ld_apogee, pred_hd_apogee; // low drag vs. high drag predictions
bool curr_drag; // 0 for low drag, 1 for high drag

//Other Digital Pins
int burnoutPin = 3;

void setup() {

  Serial.begin(9600);
  Serial.println("Initializing");

  // Initialize pins for motor circuit as outputs
  airbrakes.setSpeed(300); //set to 300 rpm which I was told to set as
  
  // Initialize any sensors/comm with avionics  

  setup_time = millis();
}

void loop(){
  //Read value from the burnout gpio pin
  post_burnout = digitalRead(burnoutPin);
  //curr_alt =  ;
  //curr_accel = ;
  
  switch (curr_state){
      
    case STANDBY:
      // do nothing until burnout is detected (signal from avionics)GPIO  pin 3
      // apogee prediction is unstable during powered ascent
      if(post_burnout) {
        burnout_time = millis();
        next_state = PREDICT_APOGEE;
      }
      else if(!post_burnout){
        next_state = STANDBY;
      }

    case PREDICT_APOGEE:
      // use apogee prediction functions and avionics data to determine next state - DEPLOY/RETRACT_BRAKES
      // do we want to predict both low/high drag or only curr drag apogee?
      pred_ld_apogee = predict_apogee(curr_alt, curr_accel, low_drag_parameters);    
      pred_hd_apogee = predict_apogee(curr_alt, curr_accel, high_drag_parameters);
      
      // this logic is very hacky and probably needs to be fixed
      if(pred_ld_apogee > LOW_DRAG_DEPLOY_APOGEE) {
        retract_signal = 1;
        deploy_signal = 0;
      }
      else if(pred_hd_apogee > HIGH_DRAG_DEPLOY_APOGEE) {
        retract_signal = 0;
        deploy_signal = 1;
      }

      // is there any scenario in our predict apogee function where both deploy and retract signals could be true?
      if((deploy_signal && !retract_signal) && (curr_time > MIN_DEPLOY_TIME) && (curr_tilt < MAX_TILT))
        next_state = DEPLOY_BRAKES;

      if((deploy_brakes_time != 0 && curr_time - deploy_brakes_time > MAX_DEPLOY_TIME) || pred_hd_apogee < HIGH_DRAG_DEPLOY_APOGEE)
        next_state = RETRACT_BRAKES;

    case DEPLOY_BRAKES:
      // deploy airbrakes to lower predicted apogee -- figure out deployment control method
      // return to predict apogee state
      deploy_signal = 1;
      //retract_signal = 0;
      //end_signal = 0;
      deploy_brakes_time = millis();
      curr_drag = 1;
      airbrakes.step(stepsPerRevolution);
      next_state = PREDICT_APOGEE;
      
    case RETRACT_BRAKES:
      // retract airbrakes to lower drag -- figure out control
      // return to predict apogee state
      //deploy_signal = 0;
      //end_signal = 0;
      retract_signal = 1;
      retract_brakes_time = millis(); //Try and figure out what shes trying to do with this
      curr_drag = 0;
      airbrakes.step(-stepsPerRevolution);
      next_state = PREDICT_APOGEE;
      
    case POST_APOGEE:
      // once apogee is detected, air brakes become useless and nothing we can do will help -- stop trying
      end_signal = 1;
      airbrakes.step(0);
      next_state = POST_APOGEE;
  }

  curr_state = next_state;
}

