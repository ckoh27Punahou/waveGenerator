// library imports 
// NOTE on Platform IO use: You will have to import all of the non-standard Arduino libraries using the Platform IO library interface

#include <Arduino.h> // imports the standard Arduino library - used in Platform IO (VS Code Arduino programming software)
#include <AccelStepper.h> // Mike McCaully's AccelStepperLibrary
#include <Wire.h> // one of the default Arduino libraries
#include <hd44780.h> // used to interface with LCD (by Bill Perry)
#include <hd44780ioClass/hd44780_I2Cexp.h> // IF NOT WRITTEN ON Platform IO (instead written in Arduino Studio), this path will be different!
#include <SPI.h> // one of the default arduino libraries

// Pin Variables

const uint8_t stepPin = 9; // step pin for the stepper motor driver (PUL+, STP+, etc.)
const uint8_t dirPin = 8; // direction pin for the stepper motor driver (DIR+)
const uint8_t enablePin = 7; // enable pin for the stepper motor driver (EN+, MF+, etc.)

const uint8_t ampClk = 2; // clk pin for the amplitude rotary encoder (detecting rotation + direction)
const uint8_t ampDt = 3; // dt pin for the amplitude rotary encoder (detecting rotation + direction)
const uint8_t ampSw = 6; // sw pin for the amplitude rotary encoder (button click)
const uint8_t freqClk = 4; // clk pin for the frequency rotary encoder (detecting rotation + direction)
const uint8_t freqDt = 5; // dt pin for the amplitude rotary encoder (detecting rotation + direction)
const uint8_t freqSw = 10; // sw pin for the frequency rotary encoder (button click)

// Various Constants

const int stepsPerRevolution = 200; // this will always be 200 (not to be confused with microsteps per revolution)
const int microsteps = 6400/200; // microsteps per step (total microsteps per rev/normal stepsPerRevolution)
const int maxSpeed = 80000; // max speed of motors in steps/sec
float acceleration = 2500*stepsPerRevolution*microsteps; // acceleration constant

const float stepInterval = 0.1; // not an interval for steps - adjusts the speed of adjustment for amplitude and frequency

float maxAmplitude = 10; // Maximum amplitude in mm
float minAmplitude = 0 - stepInterval; // Minimum amplitude in mm
float maxFrequency = 5; // Maximum frequency in Hz
float minFrequency = 0.1 - stepInterval; // Minimum frequency in Hz

const float pulleyPitch = 0.04; // the pitch diamter of the pulley in meters (40mm)
float maxStrokeLength = 35; // 1/2 of the available length of movement. This is technically 40mm, but it's set to give slight clearance

const float reduction = 0.8; // effective amplitude (experimental amplitude/theoretical amplitude) 
const float compensation = 1/reduction; // compensates for amplitude losses from experimental setups

const int loadingTime = 5000; // milliseconds between many of the display steps - allows for the users to read the instructions 

// Actual Variables

float tankDepth = 120; // the depth of water in the tank (in mm)
float startingDepth = 20; // the depth of the wedge beneath the surface (lowest point at start) - in mm
float wedgeAngle = 20; // the angle of the wedge submerged in the water (in degrees)

float amplitude = maxAmplitude; // adjustable amplitude
float frequency = maxFrequency; // adjustable frequency

bool ampConfirmed = false; // used to check user amplitude confirmation (can be seen later)
bool freqConfirmed = false; // used to check user frequency confirmation (can be seen later)

int ampCurrentClk; // current amp rotary encoder clk pin state (expressed as HIGH or LOW)
int freqCurrentClk; // current freq rotary encoder clk pin state (expressed as HIGH or LOW)
int ampPreviousClk; // previous cycle amp rotary encoder clk pin state (expressed as HIGH or LOW) - used for referencing changes
int freqPreviousClk; // previous cycle freq rotary encoder clk pin state (expressed as HIGH or LOW) - used for referencing changes

int ampSwitch; // checking the status of the amp rotary encoder button press (expressed as LOW when pressed, high when not)
int freqSwitch; // checking the status of the freq rotary encoder button press (expressed as LOW when pressed, high when not)

float targetPosition = 0; // target position stepper moves to (updated throughout running loop)
float targetSpeed; // target velocity of stepper motor (updated throughout running loop)
float maxRotation; // the amplitude of the motor movement pattern (the motor moves in a sin wave - this is the amplitude of wave)

String waveType = ""; // tells the wave type ("Deep", "Intermediate", or "Shallow") - depends on wave tank depth and wavelength

// Object Declarations

// Using the AccelStepper Arduino library

AccelStepper stepper(AccelStepper::DRIVER, stepPin, dirPin);

// Using the hd44780 & hd44780_I2Cexp Arduino libraries
// LCD is 20x4 connected via I2C port on Arduino (SDA and SCL - Pins A4 and A5 respectively)

hd44780_I2Cexp lcd;

// Calculating the wavelength of the wave given the frequency and tank depth

float waveLengthC(float frequency, float depth) {

  //Converting to period (reciprocal of frequency) 
  float T = 1 / frequency;

  // gravity constant
  float g = 9.81;

  // Iterative solving for wavelength - pulled from existing MatLab function 
  // https://www.mathworks.com/matlabcentral/fileexchange/48141-wavelength_calculator

  float L0 = (g*T*T)/(2*PI);
  float guess = L0;
  float L = (g*T*T)/(2*PI) * tanh((2*PI) * (depth/guess));
  float diff = abs(L-guess);

  // iterative guessing
  while (diff > 0.01) {
    diff = abs(L-guess);
    guess = L + (0.5 * diff);
    L = (g*T*T)/(2*PI) * tanh((2*PI) * (depth/guess));
  }

  // returns wavelength
  return L;
}

// calculate the stroke length from starting depth of wedge (startingDepth), amplitude of desired wave (amplitude), 
// wavelength of desired wave (calculated from frequency and tank depth) and wedge angle (theta)
// 

float stroke(float startingDepth, float amplitude, float wavelength, float theta) {
  // converting wedge angle to radians
  theta = theta * PI / 180.0;

  // converting amplitude from mm to meters
  amplitude = amplitude / 1000.0;

  // calculating length of stroke given input parameters
  // solving for equation 9 (https://www.sciencedirect.com/science/article/abs/pii/S0029801821013573)
  float strokeLength = -startingDepth + sqrt((amplitude*wavelength / (2*PI*tan(theta))) + (startingDepth*startingDepth));

  // adjust for 80% of wave height from numerical to experimental
  // compensation is reciprocal of % (80% of height -> 125% compensation)
  strokeLength = strokeLength * compensation;

  // return solved stroke length
  return strokeLength;
}

void setup() { // this section of code only runs once

  // Local waveLength declaration

  float waveLength = 0.0;

  // LCD Initialization 
  Wire.begin();
  lcd.begin(20,4);
  lcd.backlight();

  // Motor enable pulse
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW);

  // Amplitude Rotary Encoder Initiation
  pinMode(ampClk, INPUT);
  pinMode(ampDt, INPUT);
  pinMode(ampSw, INPUT_PULLUP);

  // Frequency Rotary Encoder Initiation
  pinMode(freqClk, INPUT);
  pinMode(freqDt, INPUT);
  pinMode(freqSw, INPUT_PULLUP);

  // Setting initial state for rotary encoders
  ampPreviousClk = digitalRead(ampClk);
  freqPreviousClk = digitalRead(freqClk);

  // Initialization of motor settings
  stepper.setMaxSpeed(maxSpeed);
  stepper.setAcceleration(acceleration);
  stepper.setCurrentPosition(0);

  // stepper.moveTo(-200*microsteps);
  // while (stepper.distanceToGo() != 0) {
  //   stepper.run(); 
  // }

  //Introduction

  // Print on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Wave Generator");
  lcd.setCursor(0, 1);
  lcd.print("Controls v1.1");
  lcd.setCursor(0, 3);
  lcd.print("Charlie Koh 2025");

  delay(loadingTime);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("All units in mm");
  lcd.setCursor(0, 1);
  lcd.print("and Hz.");
  lcd.setCursor(0, 2);
  lcd.print("Click dial to");
  lcd.setCursor(0, 3);
  lcd.print("confirm values.");

  delay(loadingTime);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Max Frequency : ");
  lcd.setCursor(0, 1);
  lcd.print(maxFrequency + stepInterval);
  lcd.setCursor(0, 2);
  lcd.print("Max Amplitude: ");
  lcd.setCursor(0, 3);
  lcd.print(maxAmplitude + stepInterval);
  
  delay(loadingTime); 

  // Wedge Angle Confirmation - units are degrees
  bool wedgeAngleConfirmed = false;

  //User introduction
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Set Wedge Angle.");
  lcd.setCursor(0,1);
  lcd.print("Units are Degrees. ");

  delay(loadingTime);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Wedge Angle (Deg): ");
  lcd.setCursor(0, 1);
  lcd.print(wedgeAngle);

  // While the switch on either potentiometer is not pressed 
  while (!wedgeAngleConfirmed) {
    //Updating readings on the different rotary encoders
    ampCurrentClk = digitalRead(ampClk);
    freqCurrentClk = digitalRead(freqClk);

    ampSwitch = digitalRead(ampSw);
    freqSwitch = digitalRead(freqSw);

    // detecting and registering change in either rotary encoder
    if (ampCurrentClk != ampPreviousClk || freqCurrentClk != freqPreviousClk) {
      // detects positive or negative movmement
      if (ampCurrentClk != digitalRead(ampDt) || freqCurrentClk != digitalRead(freqDt)) {
        wedgeAngle += 1;
      } else {
        wedgeAngle -= 1;
      }

      if (wedgeAngle < 0) { // ensuring does not dip below minimum value
        wedgeAngle = 0;
      }

      // Updating display values
      lcd.setCursor(0, 0);
      lcd.print("Wedge Angle (Deg.): ");
      lcd.setCursor(0, 1);
      lcd.print(wedgeAngle);
    }

    // Detecting encoder switch press (pressed returns a low value) on either switch
    if (ampSwitch == LOW || freqSwitch == LOW) {
      wedgeAngleConfirmed = true; // confirms button press (breaks the while loop conditional)
      lcd.setCursor(0, 0);
      lcd.print("Wedge Angle set!"); // tells user that it's been pressed
      delay(1000);
    }

    // updating rotary encoder values - rotary encoder compares previous values to current values to detect change
    // direction change is detected using digitalRead(dtPin) and comparing with Clk pin
    ampPreviousClk = ampCurrentClk;
    freqPreviousClk = freqCurrentClk;
  }

  // Setting starting depth and tank depth (units are mm for this measurement)
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Set starting depth");
  lcd.setCursor(0, 1);
  lcd.print("and tank depth");
  lcd.setCursor(0, 2);
  lcd.print("Units are mm.");
  delay(loadingTime);

  // Conditionals defined for the while loop
  bool startingDepthConfirmed = false; // Amp Rotary Encoder corresponds
  bool tankDepthConfirmed = false; // Freq Rotary Encoder corresponds 

  // User display
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Starting Depth (mm): ");
  lcd.setCursor(0, 1);
  lcd.print(startingDepth);
  lcd.setCursor(0, 2);
  lcd.print("Tank Depth (mm): ");
  lcd.setCursor(0, 3);
  lcd.print(tankDepth);

  while (!startingDepthConfirmed || !tankDepthConfirmed) { // while either or both are not confirmed (button not pressed)
    
    // updating rotary encoder values
    ampSwitch = digitalRead(ampSw); 
    freqSwitch = digitalRead(freqSw);

    ampCurrentClk = digitalRead(ampClk);
    freqCurrentClk = digitalRead(freqClk);

    if (ampCurrentClk != ampPreviousClk && !startingDepthConfirmed) { // If not confirmed, and change in corresponding encoder value
      // amp = starting depth
      // detecting positive or negative changes
      if (ampCurrentClk != digitalRead(ampDt)) {
        startingDepth += 1;
      } else {
        startingDepth -= 1;
      }

      if (startingDepth < 0) { // ensuring does not dip below minimum value (minimum is minimum value, in this case 0)
        startingDepth = 0;
      }

      // user interface
      lcd.setCursor(0, 0);
      lcd.print("Starting Depth (mm): ");
      lcd.setCursor(0, 1);
      lcd.print(startingDepth);
    } 

    if (freqCurrentClk != freqPreviousClk && !tankDepthConfirmed) { // If not confirmed, and change in corresponding encoder value
      // freq = tank depth
      if (freqCurrentClk != digitalRead(freqDt)) {
        tankDepth += 1;
      } else {
        tankDepth -= 1;
      }

      if (tankDepth < 0) { // ensuring does not dip below minimum value
        tankDepth = 0;
      }

      // user interface
      lcd.setCursor(0, 2);
      lcd.print("Tank Depth (mm): ");
      lcd.setCursor(0, 3);
      lcd.print(tankDepth);
    }

    // Checking for confirmation (pressed switches return LOW value)
    if (ampSwitch == LOW) { // checking for starting depth confirmation
      startingDepthConfirmed = true; // changes conditional for while loop - starting depth can no longer be changed

      // informs user of updated status of confirmation
      lcd.setCursor(0, 0);
      lcd.print("Starting Depth set!");
      delay(1000);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Starting Depth (mm): ");
      lcd.setCursor(0, 1);
      lcd.print(startingDepth);
      lcd.setCursor(0, 2);
      lcd.print("Tank Depth (mm): ");
      lcd.setCursor(0, 3);
      lcd.print(tankDepth);
    }

    if (freqSwitch == LOW) { // checking for tank depth confirmation
      tankDepthConfirmed = true; // changes conditional for while loop - tank depth can no longer be changed

      // informs user of updated status of confirmation
      lcd.setCursor(0, 2);
      lcd.print("Tank Depth set!");
      delay(1000);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Starting Depth (mm): ");
      lcd.setCursor(0, 1);
      lcd.print(startingDepth);
      lcd.setCursor(0, 2);
      lcd.print("Tank Depth (mm): ");
      lcd.setCursor(0, 3);
      lcd.print(tankDepth);
    }

    // updating rotary encoder values - rotary encoder compares previous values to current values to detect change
    // direction change is detected using digitalRead(dtPin) and comparing with Clk pin
    ampPreviousClk = ampCurrentClk;
    freqPreviousClk = freqCurrentClk;
  }

  // converts mm to meters
  // this is necessary for the wavelength and stroke length calculations later
  startingDepth = startingDepth/1000;
  tankDepth = tankDepth/1000;

  // Informs user that they have correctly set both depths
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Depths set!");
  delay(loadingTime);

  // informs user that they are about to set starting wave settings (frequency and amplitude)
  // amplitude is in mm and frequency in Hz. 
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Set start amplitude");
  lcd.setCursor(0, 1);
  lcd.print("And frequency");
  lcd.setCursor(0, 2);
  lcd.print("Amplitude is mm.");
  lcd.setCursor(0, 3);
  lcd.print("Frequency is Hz.");
  delay(loadingTime); // allows the user to read what is presented

  // transition to user interface to change settings
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Amplitude (mm): ");
  lcd.setCursor(0, 1);
  lcd.print(amplitude);
  lcd.setCursor(0, 2);
  lcd.print("Frequency (Hz): ");
  lcd.setCursor(0, 3);
  lcd.print(frequency);

  while (!ampConfirmed || !freqConfirmed) { // while either one or both are not confirmed
    // updating rotary encoder values
    ampSwitch = digitalRead(ampSw);  
    freqSwitch = digitalRead(freqSw);

    ampCurrentClk = digitalRead(ampClk);
    freqCurrentClk = digitalRead(freqClk);

    if (ampCurrentClk != ampPreviousClk && !ampConfirmed) { // if change detected and amp is not confirmed yet
      
      // detects for positive and negative change 
      if (ampCurrentClk != digitalRead(ampDt)) {
        amplitude += stepInterval;
      } else {
        amplitude -= stepInterval;
      }

      if (amplitude < minAmplitude) { // ensures does not dip below minimum value
        amplitude = minAmplitude;
      }
      if (amplitude > maxAmplitude) { // ensures does not go above maximum value
        amplitude = maxAmplitude;
      }

      // updates the user display with new value
      lcd.setCursor(0, 0);
      lcd.print("Amplitude (mm): ");
      lcd.setCursor(0, 1);
      lcd.print(amplitude);

      ampPreviousClk = ampCurrentClk; // updates amp rotary encoder reference values
    }

    if (freqCurrentClk != freqPreviousClk && !freqConfirmed) { // if change detected and freq is not confirmed yet

      // detects positive and negative change
      if (freqCurrentClk != digitalRead(freqDt)) {
        frequency += stepInterval;
      } else {
        frequency -= stepInterval;
      }

      if (frequency < minFrequency) { // ensures does not dip below minimum value
        frequency = minFrequency;
      }
      if (frequency > maxFrequency) { // ensures does not go above maximum value
        frequency = maxFrequency;
      }

      // updates the user display with new value 
      lcd.setCursor(0, 2);
      lcd.print("Frequency (Hz): ");
      lcd.setCursor(0, 3);
      lcd.print(frequency);

      freqPreviousClk = freqCurrentClk; // updates freq rotary encoder reference values
    }

    if (ampSwitch == LOW) { //detects whether the amp switch is pressed (pressed returns LOW value)
      ampConfirmed = true; // changes conditional for while loop - amp value can no longer be changed

      // updates user that they have confirmed amp
      lcd.setCursor(0, 0);
      lcd.print("Amplitude set!");
      delay(1000);

      // resets back to normal display
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Amplitude (mm): ");
      lcd.setCursor(0, 1);
      lcd.print(amplitude);
      lcd.setCursor(0, 2);
      lcd.print("Frequency (Hz): ");
      lcd.setCursor(0, 3);
      lcd.print(frequency);
    }

    if (freqSwitch == LOW) { // detects whether the freq switch is pressed (pressed returns LOW value)
      freqConfirmed = true; // changes conditional for while loop - freq values can no longer be changed

      // updates user that they have confirmed freq
      lcd.setCursor(0, 2);
      lcd.print("Frequency set!");
      delay(1000);

      // resets back to normal display
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Amplitude (mm): ");
      lcd.setCursor(0, 1);
      lcd.print(amplitude);
      lcd.setCursor(0, 2);
      lcd.print("Frequency (Hz): ");
      lcd.setCursor(0, 3);
      lcd.print(frequency);
    }
  }

  // runs calculations after frequency, amplitude, tank depth, wedge starting depth, and wedge angle are all inputed and confirmed
  // frequency is in Hz, tank depth is in meters
  waveLength = waveLengthC(frequency, tankDepth); // returns the wavelength (function above)
  // starting depth is in meters, amplitude is in mm, wavelength is in meters, wedge angle is in degrees
  float strokeLength = stroke(startingDepth, amplitude, waveLength, wedgeAngle); // returns vertical stroke length of wedge (function above)

  // checks whether the stroke length is greater than the max stroke (mechanical hardstop)
  if (1000*strokeLength > maxStrokeLength) { // converts strokeLength from meters to mm (max stroke is already in mm)
    strokeLength = maxStrokeLength; // if conditional is met (strokeLength > max) then strokeLength is set to the maximum possible value

    // informs user that their input values returned a stroke length out of range
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Stroke Length");
    lcd.setCursor(0,1);
    lcd.print("Exceeds Maximum");
    lcd.setCursor(0,2);
    lcd.print("Reset To Max");
    lcd.setCursor(0,3);
    lcd.print(strokeLength); 
    delay(loadingTime); // gives user time to read information
  } 

  // the motor moves in a sin wave
  // defines the maximum (amplitude) of the sin wave using the desired linear motion in steps
  // strokeLength / pulleyPitch defines the amount of revolutions
  // microsteps and stepsPerRevolution turn it into step equivalent 
  maxRotation = stepsPerRevolution * microsteps * (strokeLength / pulleyPitch);

  // the displacement position is calculated as following: maxRotation * cos(2*PI*frequency*t) - t is time in seconds
  // the velocity curve (derivative of displacement) is as following: -2*PI*frequency*maxRotation * sin(2*PI*frequency*t)
  // The maximum value of velocity is 2*PI*frequency*maxRotation. This has to be less than or equal to max speed of stepper
  // Leaving us with the following: 2*PI*freqency*maxRotation <= maxSpeed
  // Taking the inverse of that statement gives the following: 2*PI*frequency*maxRotation > maxSpeed
  // Manipulating that expression leaves us with the following conditional
  if (maxRotation > (maxSpeed/(2*PI*frequency))) { // if the above condition is not met (it is above maximum speed of stpper)
    //informs the user that their settings returned settings too high
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Max Speed Exceeded.");
    lcd.setCursor(0,1); 

    while (maxRotation > (maxSpeed/(2*PI*frequency))) { // while the above condition is not met
      amplitude -=0.01; // iteratively adjusts the amplitude
      strokeLength = stroke(startingDepth, amplitude, waveLength, wedgeAngle); // calculates new strokeLength with adjusted amplitude
      maxRotation = stepsPerRevolution * microsteps * (strokeLength / pulleyPitch); // calculates new maxRotation with adjusted strokeLength
    }

    // finishes informing the user of their new amplitude
    lcd.print("New Amplitude (mm): ");
    lcd.setCursor(0,2);
    lcd.print(amplitude); 
    delay(5000); 

    // IMPORTANT: adjust the amplitude around the frequency, NOT the frequency around the amplitude
  }

  // identifies the wave type based on the following conditionals
  if (((2*PI)/waveLength)*tankDepth < PI/10) {
    waveType = "Shallow Wave Type";
  } else if(((2*PI)/waveLength)*tankDepth > PI) {
    waveType = "Deep Wave Type";
  } else {
    waveType = "Intermediate Wave Type";
  }

  // informs the user that the setup is complete 
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Setup complete!");

  // informs the user what wave type their inputs has returned
  lcd.setCursor(0, 1);
  lcd.print("Wave Type: ");
  lcd.setCursor(0, 2);
  lcd.print(waveType);
  delay(2000); 

  // resets to user operational display
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Wavelength (meters): " );
  lcd.setCursor(0, 1);
  lcd.print(waveLength);
  lcd.setCursor(0, 2);
  lcd.print("Wave Type: ");
  lcd.setCursor(0, 3);
  lcd.print(waveType);

  // reseting confirmations for changable settings
  // in the following loop these values have the option to be adjusted - these confirmations have to be reset before entering the loop
  ampConfirmed = false;
  freqConfirmed = false;
}

// Loop Variable Declarations

float waveLength = waveLengthC(frequency, tankDepth); // calculatin"g the wavelength

bool toggle = false; // alternates button toggle selection

int currentTime = millis(); // has the current time in milliseconds
int previousTime = currentTime; // reference time to compare to current time (elapsed time)

void loop() { // this section of code runs forever

  // updates the rotary encoder values for both amp and freq
  ampSwitch = digitalRead(ampSw);
  freqSwitch = digitalRead(freqSw);

  ampCurrentClk = digitalRead(ampClk);
  freqCurrentClk = digitalRead(freqClk);

  // runs the motor and periodically updates motor velocity and target position
  if (!toggle) { // runs if toggle == false
    currentTime = millis(); // updates the current time in milliseconds

    if (currentTime - previousTime > 5) { // if the time since previously checking target position and velocity is greater than 5 milliseconds

      // updates and recalculates target position and velocity
      // position values have range from [-maxRotation, maxRotation]
      targetPosition = maxRotation * cos(2 * PI * frequency * (currentTime / 1000.0)); 

      // velocity values have range from [-2*PI*frequency*maxRotation, 2*PI*frequency*maxRotation]
      targetSpeed = -2 * PI * frequency * maxRotation * sin(2 * PI * frequency * (currentTime / 1000.0)); 

      // updates and checks that the targetted motor velocity isn't greater than max
      if (targetSpeed > maxSpeed) { // if target velocity is greater than max
        targetSpeed = maxSpeed;  // update target velocity to max
      }

      stepper.moveTo(targetPosition); // update stepper target position (check AccelStepper for more information)
      stepper.setSpeed(targetSpeed); // update steper target velocity (check AccelStepper for more information)
      previousTime = currentTime; // updates reference timestamp for comparison
    }

    stepper.run(); // as long as toggle == false, the stepper should be moving to target position (check AccelStepper for more information)
  } 

  else { // if toggle = true, then toggle to allow user to change settings (similar to initial setup)

    while (!ampConfirmed || !freqConfirmed) { // while either one or both of the amp or freq settings are not confirmed

      // updating rotary encoder values (both freq and amp)
      ampSwitch = digitalRead(ampSw);
      freqSwitch = digitalRead(freqSw);

      ampCurrentClk = digitalRead(ampClk);
      freqCurrentClk = digitalRead(freqClk);

      if (ampCurrentClk != ampPreviousClk && !ampConfirmed) { // if change is detected, and amp is not yet confirmed
        // detect positive or negative changes
        if (ampCurrentClk != digitalRead(ampDt)) {
          amplitude += stepInterval;
        } else {
          amplitude -= stepInterval;
        }

        // updates user display with adjusted values
        lcd.setCursor(0, 0);
        lcd.print("Amplitude (mm): ");
        lcd.setCursor(0, 1);
        lcd.print(amplitude);

        if (amplitude < minAmplitude) { // ensures that amplitude does not dip below minimum
          amplitude = minAmplitude;
        }
        if (amplitude > maxAmplitude) { // ensures that amplitude does go above max amplitude
          amplitude = maxAmplitude; 
        }
      }

      if (freqCurrentClk != freqPreviousClk && !freqConfirmed) { // if change is detected, and freq is not yet confirmed
        // detects positive or negative cahgnes
        if (freqCurrentClk != digitalRead(freqDt)) {
          frequency += stepInterval;
        } else {
          frequency -= stepInterval;
        }
        
        // updates user display with adjusted values
        lcd.setCursor(0, 2);
        lcd.print("Frequency: ");
        lcd.setCursor(0, 3);
        lcd.print(frequency);

        if (frequency < minFrequency) { // ensures that frequency does not dip below minimum value 
          frequency = minFrequency;
        }
        if (frequency > maxFrequency) { // ensures that frequency does not go above maximum value
          frequency = maxFrequency;
        }
      }

      if (ampSwitch == LOW) { // detects whether amplitude switch is pressed (pressed returns LOW value)
        ampConfirmed = true; // changes while loop conditional - amplitude can no longer be changed

        // informs the user that they have confirmed amplitude 
        lcd.setCursor(0, 0);
        lcd.print("Amplitude set!");
        delay(1000);

        // resets back to normal display
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Amplitude (mm): ");
        lcd.setCursor(0, 1);
        lcd.print(amplitude);
        lcd.setCursor(0, 2);
        lcd.print("Frequency (Hz): ");
        lcd.setCursor(0, 3);
        lcd.print(frequency);
      }

      if (freqSwitch == LOW) { // detects whether frequency switch is pressed (pressed returns LOW value)
        freqConfirmed = true; // changes while loop conditional - frequency can no longer be changed 

        // informs the user that they have confirmed frequency
        lcd.setCursor(0, 2);
        lcd.print("Frequency set!");

        // resets back to normal display
        delay(1000);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Amplitude (mm): ");
        lcd.setCursor(0, 1);
        lcd.print(amplitude);
        lcd.setCursor(0, 2);
        lcd.print("Frequency (Hz): ");
        lcd.setCursor(0, 3);
        lcd.print(frequency);
      }

      // resets encoder reference values (both amp and freq)
      ampPreviousClk = ampCurrentClk;
      freqPreviousClk = freqCurrentClk;
    }

    // once both are confirmed (amp and freq) ensure that program does not immediately toggle back to making changes 
    toggle = false; // changes toggle so that it does not immediately trigger another conditional
    ampSwitch = HIGH; // changes switch reading so does not immediately trigger toggle 
    freqSwitch = HIGH; // changes switch reading so does not immediately trigger toggle 

    // recalculates updated wavelength and stroke length calculations after updated frequency and amplitude inputs
    waveLength = waveLengthC(frequency, tankDepth); // updates wavelength value
    float strokeLength = stroke(startingDepth, amplitude, waveLength, wedgeAngle); // updates stroke length value

    // checks mechanical limits - makes sure that stroke length does not exceed the mechanical max stroke 
    if (1000*strokeLength > maxStrokeLength) { // stroke length is converted from meters to mm (maxStrokeLength is in mm)
      // if it is greater than max stroke, lowers to max stroke length
      strokeLength = maxStrokeLength/1000; // updates stroke length, makes sure that it is adjusted to meters (maxStroke is in mm)

      // informs the user that their inputs caused a stroke length which exceeded the maximum possible stroke length
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Stroke Length");
      lcd.setCursor(0,1);
      lcd.print("Exceeds Maximum");
      lcd.setCursor(0,2);
      lcd.print("Reset To Max");
      lcd.setCursor(0,3);
      lcd.print(strokeLength); 
      delay(loadingTime); 
    } 

    // recalculates the maxRotation (given the adjusted stroke length values)
    // the motor moves in a sin wave
    // defines the maximum (amplitude) of the sin wave using the desired linear motion in steps
    // strokeLength / pulleyPitch defines the amount of revolutions
    // microsteps and stepsPerRevolution turn it into step equivalent
    maxRotation = stepsPerRevolution * microsteps * (strokeLength / pulleyPitch); 

    // the displacement position is calculated as following: maxRotation * cos(2*PI*frequency*t) - t is time in seconds
    // the velocity curve (derivative of displacement) is as following: -2*PI*frequency*maxRotation * sin(2*PI*frequency*t)
    // The maximum value of velocity is 2*PI*frequency*maxRotation. This has to be less than or equal to max speed of stepper
    // Leaving us with the following: 2*PI*freqency*maxRotation <= maxSpeed
    // Taking the inverse of that statement gives the following: 2*PI*frequency*maxRotation > maxSpeed
    // Manipulating that expression leaves us with the following conditional
    if (maxRotation > (maxSpeed/(2*PI*frequency))) { // checks that the target velocity does not exceed designated max velocity

      // informs the user that their input parameters resulted in a target velocity which exceeded max motor velocity
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Max Speed Exceeded.");
      lcd.setCursor(0,1); 

      // iteratively adjusts the amplitude so that the velocity is below the max motor velocity
      while (maxRotation > (maxSpeed/(2*PI*frequency))) {
        amplitude -=0.01; // iteratively lowers the amplitude values
        strokeLength = stroke(startingDepth, amplitude, waveLength, wedgeAngle); // recalculates the stroke length
        maxRotation = stepsPerRevolution * microsteps * (strokeLength / pulleyPitch); // recalculates the maxRotation (which in turn recalculates max velocity)
      }

      // informs the user of the updated amplitude
      lcd.print("New Amplitude: ");
      lcd.setCursor(0,2);
      lcd.print(amplitude); 
      delay(loadingTime);
    }

    // Identify Wave Type based on the following conditionals
    if (((2*PI)/waveLength)*tankDepth < PI/10) {
      waveType = "Shallow Wave Type";
    } else if(((2*PI)/waveLength)*tankDepth > PI) {
      waveType = "Deep Wave Type";
    } else {
      waveType = "Intermediate Wave Type";
    }

    // after all recalculations turns the wave generator back on

    // informs the user that the wave generator has turned back on
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Wave Generation: On");
    lcd.setCursor(0, 1);
    lcd.print("Wave Type: ");
    lcd.setCursor(0, 2);
    lcd.print(waveType);
    delay(2000);
    lcd.clear();

    // sets to default running user interface
    lcd.setCursor(0,0);
    lcd.print("Wavelength (meters): ");
    lcd.setCursor(0,1);
    lcd.print(waveLength);
    lcd.setCursor(0,2);
    lcd.print("Wave Type: ");
    lcd.setCursor(0,3);
    lcd.print(waveType);
  }

  // checks for button presses (this can't run if it just exited setting changes - both ampSwitch and freqSwitch were set to HIGH)
  if (freqSwitch == LOW || ampSwitch == LOW) {
    toggle = !toggle; // changes the toggle value (false -> true)
    if (toggle) { // a given statement, as this cannot run if toggle == true (flips to false)

      // tells the user that they just toggled the generator off
      // toggled to changing setting mode
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Wave Generation: Off");

      // resets the stepper back to home position (starting depth that was previously inputted)
      // note that this does not work well for steppers that lack an encoder
      stepper.moveTo(0);
      while (stepper.distanceToGo() != 0) {
        stepper.run(); 
      }

      // informs the user that they can now input new settings
      delay(1000);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Please input new settings."); 
      delay(1000);

      // changes the user interface to display amplitude and frequency 
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Amplitude (mm): ");
      lcd.setCursor(0, 1);
      lcd.print(amplitude);
      lcd.setCursor(0, 2);
      lcd.print("Frequency (Hz): ");
      lcd.setCursor(0, 3);
      lcd.print(frequency);
    }
    delay(500); // ensures that it does not trigger two loop cycles in a low (debounce delay)
  }
 
  // reset confirmation values
  ampConfirmed = false;
  freqConfirmed = false;
}