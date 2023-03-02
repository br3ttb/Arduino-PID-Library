/********************************************************
   PID Basic simulated heater Example
   Reading analog input 0 to control analog PWM output 3
 ********************************************************/
//  This simulates a 20W heater block driven by the PID
//  Vary the setpoint with the Pot, and watch the heater drive the temperature up
//
//  Simulation at https://wokwi.com/projects/358122536159671297
//
//  Based on
//  Wokwi https://wokwi.com/projects/357374218559137793
//  Wokwi https://wokwi.com/projects/356437164264235009

#include <PID_v1.h> // https://github.com/br3ttb/Arduino-PID-Library

//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp = 17, Ki = 0.3, Kd = 2; // works reasonably with sim heater block 
//double Kp = 255, Ki = .0, Kd = 0; // works reasonably with sim heater block 
//double Kp = 2, Ki = 5, Kd = 1; // commonly used defaults
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_E, DIRECT);

const int PWM_PIN = 3;  // UNO PWM pin
const int INPUT_PIN = -1; // Analog pin for Input (set <0 for simulation)
const int SETPOINT_PIN = A1;   // Analog pin for Setpoint Potentiometer
const int SETPOINT_INDICATOR = 6; // PWM pin for indicating setpoint
const int INPUT_INDICATOR = 5; // PWM pin for indicating Input

void setup()
{
  Serial.begin(115200);
  Serial.println(__FILE__);
  myPID.SetOutputLimits(-4, 255);
  if (SETPOINT_INDICATOR >= 0) pinMode(SETPOINT_INDICATOR, OUTPUT);
  if (INPUT_INDICATOR >= 0) pinMode(INPUT_INDICATOR, OUTPUT);
  Setpoint = 0;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  if(INPUT_PIN>0){
    Input = analogRead(INPUT_PIN);
  }else{
    Input = simPlant(0.0,1.0); // simulate heating
  }
  Serial.println("Setpoint Input Output Watts");
}

void loop()
{
  // gather Input from INPUT_PIN or simulated block
  float heaterWatts = (int)Output * 20.0 / 255; // 20W heater
  if (INPUT_PIN > 0 ) {
    Input = analogRead(INPUT_PIN);
  } else {
    float blockTemp = simPlant(heaterWatts,Output>0?1.0:1-Output); // simulate heating
    Input = blockTemp;   // read input from simulated heater block
  }

  if (myPID.Compute())
  {
    analogWrite(PWM_PIN, (int)Output);

    Setpoint = analogRead(SETPOINT_PIN) / 4; // Read setpoint from potentiometer
    if (INPUT_INDICATOR >= 0) analogWrite(INPUT_INDICATOR, Input);
    if (SETPOINT_INDICATOR >= 0) analogWrite(SETPOINT_INDICATOR, Setpoint);
  }
  report();
}

void report(void)
{
  static uint32_t last = 0;
  const int interval = 1000;
  if (millis() - last > interval) {
    last += interval;
    //    Serial.print(millis()/1000.0);
    Serial.print(Setpoint);
    Serial.print(' ');
    Serial.print(Input);
    Serial.print(' ');
    Serial.print(Output);
    Serial.print(' ');
    Serial.print(myPID.GetIntegral());
    Serial.print(' ');
    Serial.println();
  }
}

float simPlant(float Q,float hfactor) { // heat input in W (or J/s)
  // simulate a 1x1x2cm aluminum block with a heater and passive ambient cooling
 // float C = 237; // W/mK thermal conduction coefficient for Al
  float h = 5 *hfactor ; // W/m2K thermal convection coefficient for Al passive
  float Cps = 0.89; // J/g°C
  float area = 1e-4; // m2 area for convection
  float mass = 10 ; // g
  float Tamb = 25; // °C
  static float T = Tamb; // °C
  static uint32_t last = 0;
  uint32_t interval = 100; // ms

  if (millis() - last >= interval) {
    last += interval;
    // 0-dimensional heat transfer
    T = T + Q * interval / 1000 / mass / Cps - (T - Tamb) * area * h;
  }
  return T;
}
