/****************************************************************
 * Example1_Basics.ino
 * ICM 20948 Arduino Library Demo
 * Use the default configuration to stream 9-axis IMU data
 * Owen Lyke @ SparkFun Electronics
 * Original Creation Date: April 17 2019
 *
 * Please see License.md for the license information.
 *
 * Distributed as-is; no warranty is given.
 ***************************************************************/
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include "MegunoLink.h" //for filter
#include "Filter.h" // for filter
#include <SD.h> // for interacting with the sd card
#include <Servo.h> // for servos
 
//*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+
//      |  Master Rates/Gains  |
#define Scale .5
#define P_rate 2
       //Larger value is larger impact for the whole system
#define I_rate 0.2       //larger value means a larger impact
#define D_rate 0        //Larger value is larger impact

#define Roll_Offset 0
#define Pitch_Offset 5

float integral_phi = 0;
float integral_theta = 0;

float prev_phi_error = 0;
float prev_theta_error = 0;

//     filename = Data_[scale]_[P-rate]_[I-rate]_[D-rate]_[Roll-offset]_[Pitch-offset]
//String filename = "Data_" + String(Scale) + "_" + String(P_rate) + "_" + String(I_rate) + "_" + String(D_rate) + "_" + String(Roll_Offset) + "_" + String(Pitch_Offset) + ".txt";
String filename = "Data1.txt";
//*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+*+


#define AileronRC_pin  2 //define aileron and elevator input pins
#define ElevatorRC_pin 3
#define Active_pin 4 // define the pin that turns the PID on and off

volatile long A_StartTime = 0;
volatile long A_CurrentTime = 0;
volatile long A_Pulses = 0;
float roll_in;
float pitch_in;

volatile long E_StartTime = 0;
volatile long E_CurrentTime = 0;
volatile long E_Pulses = 0;


File myFile;
Servo ail_servo1; // define all three servos
Servo ail_servo2;
Servo elev_servo;

#define ail_servo1_pin 5 // define servo
#define ail_servo2_pin 6
#define elev_servo_pin 9


#define gyr_scale_factor 1310

#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.   
#define IMU_CS_PIN 8     // Which pin you connect CS (chip select) to.
#define SD_CS_PIN 10


#define filter_weight_acc 90
ExponentialFilter<long> Filter1(filter_weight_acc, 0);
ExponentialFilter<long> Filter2(filter_weight_acc, 0);
ExponentialFilter<long> Filter3(filter_weight_acc, 0);

#define filter_weight_end 100
ExponentialFilter<long> Filter4(filter_weight_end, 0);
ExponentialFilter<long> Filter5(filter_weight_end, 0);

float acc_filtered_x;
float acc_filtered_y;
float acc_filtered_z;

float theta_acc; //  *|  Pitch  |*
float phi_acc;//     *|  Roll   |*
float pitch_gyr;
float roll_gyr;

float running_phi;
float running_theta;    // for gyrometer meaurments, running as in running sum for integral

bool pidPrevActive = false;

// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1
#define g 9.80665

ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object


float prev_time;
float delta_time; //in milli-seconds, ms

float acc_weight = 0.1;
float gyr_weight = 1 - acc_weight;

float complementary_phi;
float complementary_theta;

void setup()
{
  pinMode(AileronRC_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(AileronRC_pin), A_PulseTimer, CHANGE);
  pinMode(ElevatorRC_pin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ElevatorRC_pin), E_PulseTimer, CHANGE);
  
  pinMode(Active_pin, INPUT);

  ail_servo1.attach(ail_servo1_pin);
  ail_servo2.attach(ail_servo2_pin);
  elev_servo.attach(elev_servo_pin);

  SERIAL_PORT.begin(115200);
  while (!SERIAL_PORT) { }; // only continue if serial port is running

  SPI_PORT.begin();

  Serial.print("Initializing SD card...");
  pinMode(SD_CS_PIN, OUTPUT);
  
  
  /*if (!SD.begin(SD_CS_PIN)) {
    Serial.println("Initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  //Serial.println(filename);

  //myFile = SD.open("data.txt", FILE_WRITE); //open file for writing
  //myFile.close();
// title the file the rates used for the test
// timestamp, PID_on, roll_SP, roll_PV, roll_error, pitch_SP, pitch_PV, pitch_error, roll_P, roll_I, roll_D, pitch_P, pitch_I, pitch_D
  /*if (myFile){
    myFile.print("timestamp,PID_on,roll_SP,roll_PV,roll_error,pitch_SP,pitch_PV,pitch_error,roll_P,roll_I,roll_D,pitch_P,pitch_I,pitch_D");
    myFile.close();
  }*/
  

  bool initialized = false;
  while (!initialized)
  {
    myICM.begin(IMU_CS_PIN, SPI_PORT);

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    }
    else
    {
      initialized = true;
    }
    
  }
  prev_time = millis();
}

void loop()
{
  roll_in = map(A_Pulses, 1000, 2000, 90, -90)+ Roll_Offset; // in degrees, what the RC input is for roll
  pitch_in = map(E_Pulses, 1000, 2000, -90, 90) + Pitch_Offset;
  if (myICM.dataReady())
  {
    
    myFile = SD.open("data.txt", FILE_WRITE); //open file for writing

    myICM.getAGMT();

    

    filter_acc(myICM.agmt); // filter data with exponential filter

    theta_acc = get_theta_acc();
    phi_acc = get_phi_acc();

    get_theta_gyr(myICM.agmt);
    get_phi_gyr(myICM.agmt);


    //Filter4.Filter(complementary_phi);
    //Filter5.Filter(complementary_theta);
    /*Serial.print(running_theta);
    Serial.print(", ");
    Serial.print(running_phi);
    Serial.print(", ");
    Serial.print(theta_acc);
    Serial.print(", ");
    Serial.print(phi_acc);
    Serial.print(", ");
    
    
    Serial.print(complementary_theta);
    Serial.print(", ");
    Serial.print(complementary_phi);
    Serial.print(", ");
    */

    // timestamp, PID_on, roll_SP, roll_PV, roll_error, pitch_SP, pitch_PV, pitch_error, roll_P, roll_I, roll_D, pitch_P, pitch_I, pitch_D
    bool pidActive = (digitalRead(Active_pin) == 1);

    // detect OFF edge: was previously active, now not active
    if (!pidActive && pidPrevActive) {
      resetPID();
    }

    // update previous state for next loop
    pidPrevActive = pidActive;

    // Use pidActive for the rest of your logic
    if (pidActive) {
      /*
      if (myFile){

        myFile.print(millis()); // timestamp
        myFile.print(", ");

        myFile.print(Active_pin); // PID-on
        myFile.print(", ");

        myFile.print(roll_in); // roll-SP
        myFile.print(", ");
        myFile.print(complementary_phi); // roll-PV
        myFile.print(", ");
        myFile.print(roll_in + complementary_phi); // roll-error
        myFile.print(", ");
      
        myFile.print(pitch_in); // pitch-SP
        myFile.print(", ");
        myFile.print(complementary_theta); // pitch-PV
        myFile.print(", ");
        myFile.print(pitch_in + complementary_theta); // pitch-error

        myFile.close();
      }*/
      float roll_out = PID_phi();
      float pitch_out = PID_theta();

      //Serial.print(pitch_out);
      //Serial.print(", ");
      //Serial.println(roll_out);

      servos_out(elev_servo, ail_servo1, ail_servo2, pitch_out, roll_out);
    }
    else {
      float roll_out = roll_in;
      float pitch_out = pitch_in;

      servos_out(elev_servo, ail_servo1, ail_servo2, pitch_out, roll_out);

      //Serial.print(pitch_out);
      //Serial.print(", ");
      //Serial.println(roll_out);
    }
    
    // timestamp, PID_on, roll_SP, roll_PV, roll_error, pitch_SP, pitch_PV, pitch_error
    
    
    
    delay(30);
  }
  else
  {
    float roll_out = roll_in;
    float pitch_out = pitch_in;

    servos_out(elev_servo, ail_servo1, ail_servo2, pitch_out, roll_out);

    Serial.print(pitch_out);
    Serial.print(", ");
    Serial.println(roll_out);

    SERIAL_PORT.println("Waiting for data");
    delay(30);
  }


  delta_time = millis() - prev_time; //check how much time has passed for integration
  prev_time = millis(); // update time at the very end
  //Serial.println(delta_time);
}

void resetPID() {
  // Clear integrators
  integral_phi = 0.0;
  integral_theta = 0.0;

  // Clear previous-error memory used for derivative
  prev_phi_error = 0.0;
  prev_theta_error = 0.0;

  // Clear integrated gyro-running values (optional but reduces surprises)
  running_phi = 0.0;
  running_theta = 0.0;

  // Reinitialize complementary filter to current accel-only angles so we don't get a jump
  // Note: get_phi_acc/get_theta_acc use filtered acc values, so call reset after filter update
  complementary_phi = get_phi_acc();
  complementary_theta = get_theta_acc();

  // Reset timing so delta_time won't be large on next loop
  prev_time = millis();
}


float get_phi_acc(){
  return (180/M_PI) * atan(acc_filtered_y / acc_filtered_z);
}

float get_theta_acc(){
  return (180/M_PI) * atan(acc_filtered_x / acc_filtered_z);
}

void get_phi_gyr(ICM_20948_AGMT_t agmt){
  //phi is on x axis so is roll
  float x = agmt.gyr.axes.x;
  float x_scaled = x / gyr_scale_factor;
  float x_add = x_scaled * (delta_time/100);

  running_phi = running_phi + x_add;
  complementary_phi = (complementary_phi + x_add) * gyr_weight + phi_acc * acc_weight;
}

void get_theta_gyr(ICM_20948_AGMT_t agmt){
  //theta is on y axis
  float y = agmt.gyr.axes.y;
  float y_scaled = y / gyr_scale_factor;
  float y_add = y_scaled * (delta_time/100);

  running_theta = running_theta - (y_add);
  complementary_theta = (complementary_theta - (y_add)) * gyr_weight + theta_acc * acc_weight;
}

void filter_acc(ICM_20948_AGMT_t agmt){
  float x = agmt.acc.axes.x;
  float y = agmt.acc.axes.y;
  float z = agmt.acc.axes.z;

  Filter1.Filter(x);
  Filter2.Filter(y);
  Filter3.Filter(z);  
  
  // !*!*!*!*!*!*! ROTATES THE IMU IN SPACE IN THE PLANE !*!*!*!*!*!*!
  acc_filtered_x = -Filter1.Current(); //body_X = -x
  acc_filtered_y = Filter3.Current(); //body_y = z
  acc_filtered_z = Filter2.Current(); //body_z = y
}

void A_PulseTimer(){ // For Aileron input
  A_CurrentTime = micros();
  if (A_CurrentTime > A_StartTime){
    if ((A_CurrentTime - A_StartTime) < 2000){
      A_Pulses = A_CurrentTime - A_StartTime;
    }
    A_StartTime = A_CurrentTime;
  }
}

void E_PulseTimer(){  // For Elevator input
  E_CurrentTime = micros();
  if (E_CurrentTime > E_StartTime){
    if ((E_CurrentTime - E_StartTime) < 2000 && (E_CurrentTime - E_StartTime) > 1000){
      E_Pulses = E_CurrentTime - E_StartTime;
    }
    E_StartTime = E_CurrentTime;
  }
}

int PID_phi(){ // roll or ailerons
  float roll_SP = roll_in; // get roll setpoint from the controller/RC
  float error = roll_SP + complementary_phi;
  
  
  float integral_term = take_integral_phi(error);
  float derivative_term = take_derivative_phi(error);

  float out = Scale * ((P_rate * error) + integral_term - derivative_term);
/*
  if (myFile){
    myFile.print(P_rate * error);
    myFile.print(",");
    myFile.print(integral_term);
    myFile.print(",");
    myFile.print(derivative_term);
    
    myFile.close();
  }
  */

  if (out > 90.0){ return 90.0;}
  if (out < -90.0){ return -90.0;}
  else{ return out;}
}

float take_integral_phi(float error){ // this is where I will implement trapezoid or reimann sums for approximation. currently right sum
  integral_phi = integral_phi + (error * (delta_time/100));
  return integral_phi * (I_rate);
}

float take_derivative_phi(float error){
  float delta_error = error - prev_phi_error;
  float derivative_phi = (delta_error) / (delta_time);
  return derivative_phi * (D_rate);
}


int PID_theta(){ // pitch or elevators
  float pitch_SP = pitch_in;
  float error = complementary_theta + pitch_SP;

  float integral_term = take_integral_theta(error);
  float derivative_term = take_derivative_theta(error);

  float out = Scale * ((P_rate * error) + integral_term - derivative_term);
/*
  if (myFile){
    myFile.print(P_rate * error);
    myFile.print(",");
    myFile.print(integral_term);
    myFile.print(",");
    myFile.println(derivative_term);
    
    myFile.close();
  }
*/
  if (out > 90.0){ return 90.0;}
  if (out < -90.0){ return -90.0;}
  else{ return out;}


}

float take_integral_theta(float error){ // this is where I will implement trapezoid or reimann sums for approximation
  integral_theta = integral_theta + (error * delta_time/100);
  return integral_theta * (I_rate);
}

float take_derivative_theta(float error){ //  ****************** DO SOMETHING TO COMBAT DERIVATIVE KICK    thats when a change in the setpoint makes the derivative extra big 
  float delta_error = error - prev_theta_error;
  float derivative_theta = (delta_error) / (delta_time);
  return derivative_theta * (D_rate);
}

void servos_out(Servo Elevator, Servo Aileron_1, Servo Aileron_2, float elev_out, float ail_out){
  Elevator.write(map(elev_out, -90, 90, 0, 180));
  Aileron_1.write(map(ail_out, -90, 90, 0, 180));
  Aileron_2.write(map(ail_out, -90, 90, 0, 180));
}
