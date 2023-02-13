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
//Set gains for the feedback controller, k_P, k_I
const double k_P = 200; 
const double k_I = 100; 
// Counters for milliseconds during interval
long t_now = 0;
long t_last = 0;




// Counter to keep track of encoder ticks [integer]
// Counter to keep track of the last number of ticks [integer]
long encoder_ticks_last = 0;
  
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
//Caclulate errors 
double compute_errorL(u_L, v_L)
{
  e_nowL = u_L - v_L; 
}
double compute_errorR(u_R, v_R)
{
  e_nowR = u_R - v_R; 
}
//Compute error integrals
double lefterror_integral()
{

}
double righterror_integral()
{

}

//Compute errors and ensure that the input, u, stays between 0-255
short PI_controller_left(double e_nowL, double e_int, double k_P, double k_I)
{
    short u_L; 
    u_L = (short)(k_P*e_nowL+k_I*e_intL); 
    if (u_L>255)
    {
      u_L = 255;
    }
    else if (u_L<-255)
    {
      u_L = -255;

    }
    return u_L; 
}
short PI_controller_right(double e_nowR, double e_int, double k_P, double k_I)
{
    short u_R; 
    u_R = (short)(k_P*e_nowR+k_I*e_intR); 
    if (u_R>255)
    {
      u_R = 255;
    }
    else if (u_R<-255)
    {
      u_R = -255;
      
    }
    return u_R; 
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
    u_R = 200;
    u_L = 200;
    //Safety delay
    delay(2000);

    // Write to the output pins
    digitalWrite(M1, LOW); // Drive forward (left wheels)
    digitalWrite(M2, HIGH);
    analogWrite(E1, u_L);    // Write left motors command
    digitalWrite(M3, LOW); // Drive forward (right wheels)
    digitalWrite(M4, HIGH);
    analogWrite(E2, u_R);    // Write rightmotors command

//calculate speed and turn rate
compute_vehicle_speed(v_L, v_R);
compute_vehicle_rate(v_L, v_R);

//calculate speed error
compute_errorLeft(u_L, v_L);
compute_errorRight(u_R, v_R); 

//calculate error integral 

//implement PI control  
PI_controller_left(e_nowL, e_intL, k_P, k_L); 
PI_controller_right(e_nowR, e_intR, k_P,k_L); 
}


