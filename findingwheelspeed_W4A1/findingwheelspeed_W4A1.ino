/**
 * @file findingwheelspeed_W4A2.ino
 * @author Daniel Tcherkezian (20dt18@queensu.ca)
 * @brief Arduino program to get the robot to drive straight using P only control 
 * @version 1.0
 * @date 2023-01-31
 *
 * @copyright Copyright (c) 2023
 *
 */

// Encoder digital pins

//for the right side the A channel is 12 and the B channel is 13 
//For the left motor it's 3 and 4. 
const byte SIGNAL_A = 3;
const byte SIGNAL_B = 4; 
const byte SIGNAL_C = 12; 
const byte SIGNAL_D = 13; 


int EA = 5; // Wheel PWM pin (must be a PWM pin)
int EB = 6; 
int I1 = 8; // Wheel direction digital pin 1
int I2 = 10; // Wheel direction digital pin 2
int I3 = 7; 
int I4 = 2; 


// Motor driver PWM pin
const byte E1 = 6;
const byte E2 = 5; 
// Motor driver direction pin
const byte M1 = 10;
const byte M2 = 8; 
const byte M3 = 7;
const byte M4 = 2; 

// Motor PWM command variable [0-255]
byte u_L = 0;
byte u_R = 0;
// Integral error
double e_intL = 0;
double e_intR = 0;
// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

// Wheel radius [m]
const double RHO = 0.0625;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticks = 0;
volatile long encoder_ticks2= 0; 

// Variable to store estimated angular rate of left wheel [rad/s]
double omega_L = 0.0;
double omega_R = 0.0; 
// Sampling interval for measurements in milliseconds
const int T = 1000;
//ELL is the track of the robot
const double ELL = 0.2775;
//speed of the left and right wheels 
double v_L = 0;
double v_R =0; 
//left and right desired speeds 
double des_v_L; 
double des_v_R; 
//Overall desired vehicle speed 
double V = 0.25; 
// Desired vehicle turning rate
double omega = 0; 
//Set gains for the feedback controller, k_P, k_I
const double k_P = 200; 
const double k_I = 100; 
// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;




// Counter to keep track of encoder ticks [integer]
// Counter to keep track of the last number of ticks [integer]
long encoder_ticks_left = 0;
long encode_ticks_right = 0; 

// Compute vehicle speed [m/s]
double compute_vehicle_speed(double v_L, double v_R) 
{
    double v; 
    v = 0.5*(v_L+v_R);
    return v;
}
// Compute vehicle turning rate [rad/s]
double compute_vehicle_rate(double v_L, double v_R)
{
    double omega;
    omega = 1.0 / ELL*(v_R-v_L); 
    return omega;
}
//Find desired left wheel speed
double leftwheel_speed(double omega, double V)
{
  des_v_L = V-omega*ELL/2; 
  return des_v_L; 
}
double rightwheel_speed(double omegam, double B)
{
  des_v_R = omega*ELL/2+V; 
  return des_v_R; 
}
  
// This function is called when SIGNAL_A goes HIGH
void decodeEncoderTicks()
{
    if (digitalRead(SIGNAL_B) == LOW)
    {
        // SIGNAL_A leads SIGNAL_B, so count one way
        encoder_ticks--;
    }
    else
    {
        // SIGNAL_B leads SIGNAL_A, so count the other way
        encoder_ticks++;
    }
}

void decodeEncoderTicks2()
{
    if (digitalRead(SIGNAL_D) == LOW)
    {
        // SIGNAL_D leads SIGNAL_C, so count one way
        encoder_ticks2--;
    }
    else
    {
        // SIGNAL_C leads SIGNAL_D, so count the other way
        encoder_ticks2++;
    }
}

//Compute errors and ensure that the input, u, stays between 0-255
short PI_controller (double e_now, double k_p, double e_int, double k_i)
{
  short u;
  u = short(k_p * e_now + k_i * e_int);
  if (u > 255)
  {
    u = 255;
  }
  else if  (u < -255)
  {
    u = -255;
  }
  return u;
}

void setup()
{
    // Open the serial port at 9600 bps
    Serial.begin(9600);
    

    // Set the pin modes for the motor driver
    pinMode(M1, OUTPUT);
    pinMode(M2, OUTPUT); 

    // Set the pin modes for the encoders
    pinMode(SIGNAL_A, INPUT);
    pinMode(SIGNAL_B, INPUT);
    pinMode(SIGNAL_C, INPUT);
    pinMode(SIGNAL_D, INPUT);
    
    pinMode(EA, OUTPUT);
    pinMode(I1, OUTPUT); 
    pinMode(I2, OUTPUT);
    pinMode(EB, OUTPUT);
    pinMode(I3, OUTPUT); 
    pinMode(I4, OUTPUT);

    // Every time SIGNAL_A goes HIGH, this is a pulse
    attachInterrupt(digitalPinToInterrupt(SIGNAL_A), decodeEncoderTicks, RISING);
    attachInterrupt(digitalPinToInterrupt(SIGNAL_C), decodeEncoderTicks2, RISING);
    
// Get desired left and right wheel speeds
des_v_L = leftwheel_speed(omega, V);
des_v_R = rightwheel_speed(omega, V);

Serial.print(des_v_L);
Serial.print("\n");
Serial.print(des_v_R);
Serial.print("\n");

// Print a message
Serial.print("Program initialized.");
Serial.print("\n");

}

void loop() {
// Get the elapsed time [ms]
    t_now = millis();
    //if the timestep is greater than or equal to a second, 
    if (t_now - t_last >= T)
    {     
        // Estimate the rotational speed [rad/s]
        omega_L = 2.0 * PI * ((double)encoder_ticks2 / (double)TPR) * 1000.0 / (double)(t_now - t_last);
        omega_R =  2.0 * PI * ((double)encoder_ticks / (double)TPR) * 1000.0 / (double)(t_now - t_last);
        v_L = omega_L*RHO; 
        v_R = omega_R*RHO; 

        // Record the current time [ms]
        t_last = t_now;

        // Reset the encoder ticks counter
        encoder_ticks = 0;
        encoder_ticks2 = 0; 

    }


    // Set the wheel motor PWM command [0-255]
    u_L = PI_controller (des_v_L - v_L, 100, e_intL, 50);
    u_R = PI_controller (des_v_R - v_R, 100, e_intR, 50);
    //Safety delay
    delay(2000);

    if (abs(u_L) < 255){
        e_intL += (des_v_L - v_L);
      }
      if (abs(u_R) < 255){
        e_intR += (des_v_R - v_R);
      }
// Select a direction
    if (u_L > 0)
    {
      digitalWrite(I3, HIGH);
      digitalWrite(I4, LOW);
    }
    else
    {
      digitalWrite(I3, LOW);
      digitalWrite(I4, HIGH);
    }

    if (u_R < 0)
    {
      digitalWrite(I1, HIGH);
      digitalWrite(I2, LOW);
    }
    else
    {
      digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);
    }
    
    

    // PWM command to the motor driver
    analogWrite(EA, abs(u_R));
    analogWrite(EB, abs(u_L));

}


