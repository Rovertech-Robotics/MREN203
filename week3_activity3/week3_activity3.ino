
/**
 * @file read-wheel-encoder.ino
 * @author Joshua Marshall (joshua.marshall@queensu.ca)
 * @brief Arduino program to read from a digital quadrature encoder.
 * @version 2.0
 * @date 2022-12-09
 *
 * @copyright Copyright (c) 2022
 *
 */
//testing
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

// Motor PWM command variable [0-255]
byte u = 0;

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

double v_L = 0;
double v_R =0; 

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
void setup()
{
    // Open the serial port at 9600 bps
    Serial.begin(9600);
    

    // Set the pin modes for the motor driver
    pinMode(M1, OUTPUT);

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

    // Print a message
    Serial.print("Program initialized.");
    Serial.print("\n");
}

void loop()
{
    // Get the elapsed time [ms]
    t_now = millis();
    

    if (t_now - t_last >= T)
    {
    
          
        // Estimate the rotational speed [rad/s]
        omega_L = 2.0 * PI * ((double)encoder_ticks2 / (double)TPR) * 1000.0 / (double)(t_now - t_last);
        omega_R =  2.0 * PI * ((double)encoder_ticks / (double)TPR) * 1000.0 / (double)(t_now - t_last);
        v_L = omega_L*RHO; 
        v_R = omega_R*RHO; 
       
        // Print some stuff to the serial monitor
        Serial.print("Right Wheel Encoder ticks: ");
        Serial.print(encoder_ticks);
        Serial.print("\t");

        Serial.print("Left Wheel Encoder ticks: ");
        Serial.print(encoder_ticks2);
        Serial.print("\t");

        Serial.print("Estimated Right wheel speed: ");
        Serial.print(omega_R);
        Serial.print(" rad/s");
        Serial.print("\n");
        Serial.print("Estimated Left wheel speed: ");
        Serial.print(omega_L);
        Serial.print(" rad/s");
        Serial.print("\n");

        Serial.print("Estimated Right wheel translational speed: ");
        Serial.print(v_R);
        Serial.print(" rad/s");
        Serial.print("\n");
        Serial.print("Estimated Left wheel translational speed: ");
        Serial.print(-1*v_L);
        Serial.print(" rad/s");
        Serial.print("\n");


        
        

        // Record the current time [ms]
        t_last = t_now;

        // Reset the encoder ticks counter
        encoder_ticks = 0;
        encoder_ticks2 = 0; 
    }

    // Set the wheel motor PWM command [0-255]
    u = 128;

    // Write to the output pins
    digitalWrite(M1, LOW); // Drive forward (left wheels)
    analogWrite(E1, u);    // Write left motors command
    digitalWrite(M2, LOW); // Drive forward (right wheels)
    analogWrite(E2, u);    // Write rightmotors command

}
