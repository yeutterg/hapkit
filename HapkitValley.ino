//--------------------------------------------------------------------------
// Code to compare parameters when simulating a valley with the hapkit
// May 27, 2015
// Cassondra Brown, Zach Brong, Greg Yeutter
// Drexel University ECES 690 - Introduction to Haptics
// Based on code supplied by the original hapkit authors and modified by
// Cliff Bargar
// https://hapkit.wordpress.com/
// https://github.com/yeutterg/hapkit
//--------------------------------------------------------------------------

#include <math.h>

#define SAMPLE_PERIOD  (double)0.0004 // 0.4ms sample period

// DEBUG_SERIAL statements will only print if DEBUG is nonzero
#define DEBUG 0
#define DEBUG_SERIAL if(DEBUG) Serial

#define DEBUG_PIN 13

// Kinematics
#define HANDLE_LENGTH (double)0.07 //70.00 mm
#define SECTOR_RADIUS (double)0.0757 //calipers
#define MOTOR_RADIUS (double)0.00837 //m - calipers

// Select simulation
#define VALLEY_MIN           (char)'A'
#define VALLEY_MED           (char)'B'
#define VALLEY_MAX           (char)'C'

//defaults to
#define ENV_TYPE           VALLEY_MED

// Declare pins
int pwmPin = 5; // PWM output pin for motor 1
int dirPin = 8; // direction output pin for motor 1
int sensorPosPin = A2; // input pin for MR sensor
int fsrPin = A3; // input pin for FSR sensor

// Position tracking variables
int updatedPos = 0;     // keeps track of the latest updated value of the MR sensor reading
int prev_updatedPos = 0;// for velocity calc
int rawPos = 0;         // current raw reading from MR sensor
int lastRawPos = 0;     // last raw reading from MR sensor
int lastLastRawPos = 0; // last last raw reading from MR sensor
int flipNumber = 0;     // keeps track of the number of flips over the 180deg mark
int tempOffset = 0;
int rawDiff = 0;
int lastRawDiff = 0;
int rawOffset = 0;
int lastRawOffset = 0;
const int flipThresh = 700;  // threshold to determine whether or not a flip over the 180 degree mark occurred
boolean flipped = false;

// Kinematics variables
double xh = 0;           // position of the handle [m]
double vel_h = 0;        //handle velocity [m/s] - filtered

// Force output variables
double force = 0;           // force at the handle
double Tp = 0;              // torque of the motor pulley
double duty = 0;            // duty cylce (between 0 and 255)
unsigned int output = 0;    // output command to the motor


// --------------------------------------------------------------
// Setup function -- NO NEED TO EDIT
// --------------------------------------------------------------
void setup() 
{
  // Set up serial communication
  Serial.begin(9600);
  
  // Set PWM frequency 
  setPwmFrequency(pwmPin,1); 
  
  // Input pins
  pinMode(sensorPosPin, INPUT); // set MR sensor pin to be an input
  pinMode(fsrPin, INPUT);       // set FSR sensor pin to be an input

  // Output pins
  pinMode(pwmPin, OUTPUT);  // PWM pin for motor A
  pinMode(dirPin, OUTPUT);  // dir pin for motor A
  
  // Initialize motor 
  analogWrite(pwmPin, 0);     // set to not be spinning (0/255)
  digitalWrite(dirPin, LOW);  // set direction
  
  // Initialize position valiables
  lastLastRawPos = analogRead(sensorPosPin);
  lastRawPos = analogRead(sensorPosPin);
  
  //initialize force
  force = 0;
  
  // Debug pin
  pinMode(DEBUG_PIN,OUTPUT);
  
  //initialize output compare
  InitOC();
  
  Serial.println("Initialized");
  
  Serial.println("Select environment type: ");
  Serial.print("Valley Min Width\t\t\t");
  Serial.println(VALLEY_MIN);
  Serial.print("Valley Med Width\t\t\t");
  Serial.println(VALLEY_MED);
  Serial.print("Valley Max Width\t\t");
  Serial.println(VALLEY_MAX);

}


// --------------------------------------------------------------
// Main Loop
// --------------------------------------------------------------
void loop()
{
  //********************
  //position, velocity calculated in interrupt response
  //********************


  //allow user to select environment type
  static char env_type = ENV_TYPE;
  //check for serial --> if there's a character use it as environment selection
  if(Serial.available())
  {
    env_type = Serial.read();
    Serial.print("You've selected ");  
    switch(env_type)
    {
      case VALLEY_MIN:
        Serial.println("Valley Min Width");
      break;
      
      case VALLEY_MED:      
        Serial.println("Valley Med Width");  
      break;
  
      case VALLEY_MAX:
        Serial.println("Valley Max Width");
      break;
    }
  }

  //should be in a protected region ***********
  double local_xh = xh;
  double local_vel_h = vel_h;
  //*******************************************
  
  // Step B.8: print xh via serial monitor
  DEBUG_SERIAL.print(" \tx_handle ");
  DEBUG_SERIAL.print(local_xh,4);  
  DEBUG_SERIAL.print(" \tvel_handle ");
  DEBUG_SERIAL.print(local_vel_h,4);  
  
  //*************************************************************
  //*** Section 3. Assign a motor output force in Newtons *******  
  //*************************************************************
  
  
 
  //**************** VIRTUAL ENVIRONMENTS *********************
  // constants/coefficients
  // Hapkit range is +/- 5cm
  double center_pt = 0.00; //2.5cm on either side
  double g = 9.8; //m/s^2
  double mass = 0.10; //kg
  double bump_width = 0.03;
    
  switch(env_type)
  {
   
    case VALLEY_MIN:
      bump_width = 0.01; // 1cm width
    break;
    
    case VALLEY_MED:
      bump_width = 0.03; // 3cm width
    break;
    
    case VALLEY_MAX:
      bump_width = 0.05; // 5cm width
    break;    
    
  }
  
  //for bumps and valleys, have Hapkit apply the gravitational force that's parallel to the "ground"
      //using sinusoidal bumps/valleys, f = m*g*sin(pi*x_rel/length), where "x_rel" is position relative to the maximum (or minimum) point
    
    /*    
    //BUMP: point in range of center_pt +/- bump_width/2
    if(local_xh < -(center_pt - bump_width/2) & local_xh > -(center_pt + bump_width/2))
    {
      force = mass * g * sin(2 * PI * (local_xh - (-center_pt)) / bump_width);
    }
    */
    //VALLEY
    if(local_xh > (center_pt - bump_width/2) & local_xh < (center_pt + bump_width/2))
    {
      force = -mass * g * sin(2 * PI * (local_xh - (center_pt)) / bump_width);  
    }
    else
    {
      force = 0;
    }
  
  DEBUG_SERIAL.print(" \tForce");
  DEBUG_SERIAL.println(force);
  
  // Step C.2: Tp = ?;    // Compute the require motor pulley torque (Tp) to generate that force
  // Slide 27, lecture 2: 
  Tp =  (HANDLE_LENGTH * MOTOR_RADIUS) / (SECTOR_RADIUS ) * force;
 
  //*************************************************************
  //*** Section 4. Force output (do not change) *****************
  //*************************************************************
  
  // Determine correct direction for motor torque
  if(force < 0) { 
    digitalWrite(dirPin, HIGH);
  } else {
    digitalWrite(dirPin, LOW);
  }

  // Compute the duty cycle required to generate Tp (torque at the motor pulley)
  duty = sqrt(abs(Tp)/0.03);

  // Make sure the duty cycle is between 0 and 100%
  if (duty > 1) {            
    duty = 1;
  } else if (duty < 0) { 
    duty = 0;
  }  
  output = (int)(duty* 255);   // convert duty cycle to output signal
  analogWrite(pwmPin,output);  // output the signal

}

//helper functions
//********************************************************************

//calculated "updatedPos" variable
void calculatePosition()
{
  //toggle bit 5 high (pin 13)
  //PORTB |= B00100000;
  
  //update "previous updatedPos" before updating position
  prev_updatedPos = updatedPos;
  
  // Get voltage output by MR sensor
  rawPos = analogRead(sensorPosPin);  //current raw position from MR sensor

  // Calculate differences between subsequent MR sensor readings
  rawDiff = rawPos - lastRawPos;          //difference btwn current raw position and last raw position
  lastRawDiff = rawPos - lastLastRawPos;  //difference btwn current raw position and last last raw position
  rawOffset = abs(rawDiff);
  lastRawOffset = abs(lastRawDiff);
  
  // Update position record-keeping vairables
  lastLastRawPos = lastRawPos;
  lastRawPos = rawPos;
  
  // Keep track of flips over 180 degrees
  if((lastRawOffset > flipThresh) && (!flipped)) { // enter this anytime the last offset is greater than the flip threshold AND it has not just flipped
    if(lastRawDiff > 0) {        // check to see which direction the drive wheel was turning
      flipNumber--;              // cw rotation 
    } else {                     // if(rawDiff < 0)
      flipNumber++;              // ccw rotation
    }
    if(rawOffset > flipThresh) { // check to see if the data was good and the most current offset is above the threshold
      updatedPos = rawPos + flipNumber*rawOffset; // update the pos value to account for flips over 180deg using the most current offset 
      tempOffset = rawOffset;
    } else {                     // in this case there was a blip in the data and we want to use lastactualOffset instead
      updatedPos = rawPos + flipNumber*lastRawOffset;  // update the pos value to account for any flips over 180deg using the LAST offset
      tempOffset = lastRawOffset;
    }
    flipped = true;            // set boolean so that the next time through the loop won't trigger a flip
  } else {                        // anytime no flip has occurred
    updatedPos = rawPos + flipNumber*tempOffset; // need to update pos based on what most recent offset is 
    flipped = false;
  }
  
  //toggle bit 5 low (pin 13)
  //PORTB &= ~B00100000;  
}

//function for calculating actual position and velocity in m, m/s
void getVelocity()
{
  static double prev_xh;
  static double vel_h_uf;  //unfilitered velocity
  static double prev_vel_h_uf; //previous unfiltered velocity
  static double prev_vel_h; //previous filtered velocity
  //*************************************************************
  //*** Section 2. Compute position in meters *******************
  //*************************************************************
     
  // Step B.1: print updatedPos via serial monitor
//  DEBUG_SERIAL.print("Updated pos ");
//  DEBUG_SERIAL.print(updatedPos);
//  DEBUG_SERIAL.print("\tPrev updated pos ");
//  DEBUG_SERIAL.print(prev_updatedPos);
  
  // Step B.6: double ts = ?; // Compute the angle of the sector pulley (ts) in degrees based on updatedPos
  //y = -0.00970857x + 7.33554011 
  double theta_pulley = (-0.0097086 * updatedPos + 7.33554);  //degrees!
  //double prev_theta = (-0.0097086 * prev_updatedPos + 7.33554);
  //DEBUG_SERIAL.print(" \tpulley angle ");
  //DEBUG_SERIAL.print(theta_pulley);
  
  // Step B.7: xh = ?;       // Compute the position of the handle (in meters) based on ts (in radians)
  prev_xh = xh;
  xh = theta_pulley * PI / 180 * HANDLE_LENGTH;

  //calculate velocity
  prev_vel_h_uf = vel_h_uf;
  vel_h_uf = (xh - prev_xh) / SAMPLE_PERIOD;
  
  //low pass filter: corner freq f_sample/4, H(s) = 1 / (1 + s/wc), wc = 3,927 rad/s
  //tustin approximation w/ sample time 400us: Y = H(z) X, H(z) = (0.4399 z + 0.4399) / (z - 0.1202), Y = filtered, X = unfiltered
  prev_vel_h = vel_h;
  vel_h = 0.4399 * vel_h_uf + 0.4399 * prev_vel_h_uf + 0.1202 * prev_vel_h; //discrete low pass filter
}


// --------------------------------------------------------------
// Function to set PWM Freq -- DO NOT EDIT
// --------------------------------------------------------------
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x7; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}

//initializes the output compare on timer 1 for performing calculations
void InitOC()
{
  //*******OUTPUT COMPARE ON CHANNEL 1******
  
  //see section 15.7: of ATmega328 datasheet
  //register descriptions section 17.11, page 158
  
//  DDRB |= _BV(1); //set bit 5 as output direction, OC1A debug pin (Pin 9) ---> PIN 9 IS DIRECTION
  //B5 (11) will oscillate at half the freq
  
  //WGM3:0 set to 0,1,0,0 for output compare (clear timer on compare)
  //CTC mode, TOP = OCR1A, update immediate, TOV flag set on MAX
  //from table 15-4
  TCCR1A &= ~_BV(WGM11) & ~_BV(WGM10); 
  TCCR1B &= ~_BV(WGM13);
  TCCR1B |= _BV(WGM12);
  
  //debug output
  //COM1A1:0 set to 0,1 for toggle --> set OC1A (B1) bit high (will clear at end of calculation) --> just toggle instead
  //from table 15-1 on page 134
  TCCR1A |= _BV(COM1A1) | _BV(COM1A0);
  TCCR1A &= ~_BV(COM1A1);
  
  //CS12:0 set to 1,0,0 for clk/256 prescaling: runs on 16MHz clock
  //table 15-5
  TCCR1B &= ~_BV(CS10) & ~_BV(CS11);
  TCCR1B |= _BV(CS12);
  
  //****CHANGE OCR1A TO MODULATE FREQUENCY
  //page 126: freq = f_clk / (N * (1 + OCR1A)), N = 256, f_clk = 16MHz (the 2 in the equation is because of toggling)
  //want period of 400us --> 2.5kHz
  //2.5kHz = 16 MHz / (256 * (1 + OCR1A)) --> OCR1A = 24
  OCR1A = 24;
  
  //interrupt enable
  
  TIMSK1 |= _BV(OCIE1A); //output compare interrupt timer 1A
  
}

//Timer 1 interrupt for performing position calculations
ISR(TIMER1_COMPA_vect)
{
  PORTB |= B00100000;
  //*************************************************************
  //*** Section 1. Compute position in counts (do not change) ***  
  //*************************************************************

  calculatePosition();
  getVelocity();
  PORTB &= ~B00100000;
  //PORTB &= ~B00000001; //clear bit B1
}

