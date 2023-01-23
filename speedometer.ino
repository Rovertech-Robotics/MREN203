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

// Encoder digital pins

//for the right side the A channel is 12 and the B channel is 13 
//For the left motor it's 3 and 4. 
const byte SIGNAL_A = 3;
const byte SIGNAL_B = 4; 



int EA = 5; // Wheel PWM pin (must be a PWM pin)
int EB = 6; 
int I1 = 8; // Wheel direction digital pin 1
int I2 = 10; // Wheel direction digital pin 2
int I3 = 7; 
int I4 = 2; 

// Motor driver PWM pin
const byte E1 = 6;

// Motor driver direction pin
const byte M1 = 7;

// Motor PWM command variable [0-255]
byte u = 0;

// Encoder ticks per (motor) revolution (TPR)
const int TPR = 3000;

// Wheel radius [m]
const double RHO = 0.0625;

// Counter to keep track of encoder ticks [integer]
volatile long encoder_ticks = 0;

// Variable to store estimated angular rate of left wheel [rad/s]
double omega_L = 0.0;

// Sampling interval for measurements in milliseconds
const int T = 1000;

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

void setup()
{
    // Open the serial port at 9600 bps
    Serial.begin(9600);
    

    // Set the pin modes for the motor driver
    pinMode(M1, OUTPUT);

    // Set the pin modes for the encoders
    pinMode(SIGNAL_A, INPUT);
    pinMode(SIGNAL_B, INPUT);

    pinMode(EA, OUTPUT);
    pinMode(I1, OUTPUT); 
    pinMode(I2, OUTPUT);
    pinMode(EB, OUTPUT);
    pinMode(I3, OUTPUT); 
    pinMode(I4, OUTPUT);

    // Every time SIGNAL_A goes HIGH, this is a pulse
    attachInterrupt(digitalPinToInterrupt(SIGNAL_A), decodeEncoderTicks, RISING);

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
        omega_L = 2.0 * PI * ((double)encoder_ticks / (double)TPR) * 1000.0 / (double)(t_now - t_last);

        // Print some stuff to the serial monitor
        Serial.print("Encoder ticks: ");
        Serial.print(encoder_ticks);
        Serial.print("\t");
        Serial.print("Estimated left wheel speed: ");
        Serial.print(omega_L);
        Serial.print(" rad/s");
        Serial.print("\n");

        // Record the current time [ms]
        t_last = t_now;

        // Reset the encoder ticks counter
        encoder_ticks = 0;
    }

    // Set the wheel motor PWM command [0-255]
    u = 128;

    // Write to the output pins
    digitalWrite(M1, LOW); // Drive forward (left wheels)
    analogWrite(E1, u);    // Write left motors command
}
