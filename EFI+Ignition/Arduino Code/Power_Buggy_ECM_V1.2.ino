// Engine Control Module program for Power Buggy ECM.
// Implements both fuel injection and ignition timing in one chip.
// Includes "rich idle" to enable easier startup with low operating emissions.
// Utilizes off-time modulation to enable very low duty cycle percentages.
// Version 1.2 (beta)
// (C) 2021 John Patterson Consulting, LLC
// You are free to modify and redistribute this code without restriction.

// ----------------- Program constants -----------------:

// Pin numbers:
const int pointsPin = 3;                                // Pin number for distributor points signal.
const int coilPin = 7;                                  // Pin number for ignition coil transistor.
const int redlineLEDPin = 12;                           // Pin number for redline indicator LED.
const int statusLEDPin = 13;                            // Pin number for status indicator LED.
const unsigned long blinkTime = 250;                    // LED status indicator blinking time (ms).

// Sensor parameters:
const int pulsesPerRevolution = 1;                      // Number of points pulses per engine rotation.

// Ignition timing parameters:
const unsigned long coilDwellTime = 1000;               // Amount of time (us) to charge the ignition coil.
const unsigned long programDelayTime = 200;             // Ignition compensation time for program execution delays (us).
const bool useFallingEdge = 0;                          // Indicates whether to use rising or falling edge to detect points timing.
//const double degreesAdvancePerRPM = 0.0005;
const double degreesAdvancePerRPM = 0.0;                // Amount of centrifugal advance (degrees per RPM) to apply to the timing.
const double RevLimiter = 4500;                         // Rev limiter RPM speed, above which spark is interrupted.
const double initialTiming = 8.0;                      // Amount of extra distributor advance (degrees) applied, into which program will add delays.

const double stallRPM = 100.0;                          // Minimum engine speed above which fuel will be dispensed.

// ----------------- Global Variables -----------------:
unsigned long pointsLastMicros = 0;                     // Timestamp of last recorded points triggering.
unsigned long pointsDiff = 100000000;                   // Difference in time from the last points triggering.
unsigned long coilDelayTimeMicros = 0;                  // Timestamp when coil is called to fire.
bool coilDelayWaiting = 0;                              // Flag to indicate if coil is currently waiting to fire.
unsigned long coilOnTimeMicros = 0;                     // Timestamp of coil turn-on.
bool coilOn = 0;                                        // Flag to indicate if coil is currently in the ON state.
unsigned long blinkLastMillis = 0;                      // Timestamp of last recorded LED blink.
bool blinkState = 0;                                    // State of status LED.
int RPMrunningAverageIndex = 0;                         // Index of running average value being edited.
bool lastPointsState = 0;                               // Last recorded value of the points input source.
double engineRPM = 0;                                   // Measured speed of the engine (RPM).
double delayAngle = 0;                                  // Calculated delay angle for spark timing.
unsigned long delayTime = 0;                            // Calculated delay time (us) for spark timing.
bool stalled = 0;                                       // Engine stalled state flag.

// ----------------- Program Functions -----------------:

// ---------------------------------------------------------------------------------------------------------------------
// Function to compute RPM based on time since last points opening:
double RPM()
{
  // If pointsDiff is longer than the time since points() was last called,
  // just use pointsDiff to compute the RPM:
  if(pointsDiff > (micros() - pointsLastMicros))
  {
    // RPM = (Pulses/sec)/(Pulses/Revolution)*(60 sec/min)
    return (1000000.0/pointsDiff)/pulsesPerRevolution*60.0;
  }
  // If it has been longer than pointsDiff since points() was last called,
  // use the elapsed time to compute the RPM:
  else
  {
    // RPM = (Pulses/sec)/(Pulses/Revolution)*(60 sec/min)
    return (1000000.0/(micros() - pointsLastMicros))/pulsesPerRevolution*60.0;
  }
}

// ---------------------------------------------------------------------------------------------------------------------
// Function to calculate how long to delay spark ignition timing:
unsigned long timeToFire()
{
  // Initialize delay angle to base distributor advance:
  delayAngle = initialTiming;

  // Subtract coil dwell time and program delay time from timing delay:
  //               1000 us + 200 us
  //              ------------------
  //                         us
  //                1000000 ---
  //                          s                °
  // ej: 12 ° - ----------------------- x 360 --- = 6.24 °
  //                         s                rev
  //                     60 ---
  //                        min
  //                 ---------------
  //                    800 rev
  //                        ---
  //                        min
  //
  delayAngle = delayAngle - ((coilDwellTime + programDelayTime)/1000000.0)/(60.0/engineRPM)*360.0;
  
  // Subtract centrifugal advance from delay:
  delayAngle = delayAngle - degreesAdvancePerRPM*engineRPM;

  // Ensure timing delay does not go below zero:
  if(delayAngle < 0.0)
  {
    delayAngle = 0.0;
  }

  // Convert degrees of timing delay to microseconds:
  //                      s
  //                  60 ---
  //       12            min               us
  // ej: ------ rev x -------- x 1.000.000 --- = 2500 us
  //      360             rev               s  
  //                  800 ---
  //                      min
  unsigned long timeToFireMicros = (unsigned long)((delayAngle/360.0)*(60.0/engineRPM)*1000000);

  return timeToFireMicros;
}

// ---------------------------------------------------------------------------------------------------------------------
// Function to check if points have been triggered:
bool checkPoints()
{
  // Measure state of the points:
  bool pointsState = digitalRead(pointsPin);

  // If falling edge is selected, evaluate if this type of transistion has occurred:
  if(useFallingEdge)
  {
    // Trigger on falling edge:
    if(!pointsState && lastPointsState)
    {
      lastPointsState = pointsState;
      return 1;
    }
  }
  else
  // If rising edge is selected, evaluate if this type of transistion has occurred:
  {
    // Trigger on rising edge:
    if(pointsState && !lastPointsState)
    {
      lastPointsState = pointsState;
      return 1;
    }
  }

  // If no relevant transition has occurred, return 0:
  lastPointsState = pointsState;
  return 0;
}

// ---------------------------------------------------------------------------------------------------------------------
// Function to check rev limiter and fire coil:
void fireCoil()
{
  // Start coil ON timer if rev limiter is not exceeded:
  if(engineRPM < RevLimiter)
  {
    // Turn on coil and begin timer:
    coilOn = 1;
    coilOnTimeMicros = micros();
    digitalWrite(coilPin, HIGH);
    
    // De-Illuminate rev limiter LED if rev limiter is not exceeded:
    digitalWrite(redlineLEDPin, LOW);
  }
  else
  {
    // Illuminate rev limiter LED if rev limiter is exceeded:
    digitalWrite(redlineLEDPin, HIGH);
  }
}

// ---------------------------------------------------------------------------------------------------------------------
// Function to detect if engine is stalled:
void checkStalled()
{
  //Determine if the engine is currently stalled:
  stalled = (engineRPM < stallRPM);
  
  // If engine is running, keep LED on and steady:
  if(!stalled)
  {
    digitalWrite(statusLEDPin, HIGH);
  }
  // If engine is stalled, blink LED:
  else if(millis() - blinkLastMillis > blinkTime)
  {
    blinkLastMillis = millis();
    blinkState = !blinkState;
    digitalWrite(statusLEDPin, blinkState);
  }
}

// ---------------------------------------------------------------------------------------------------------------------
// Arduino startup function (runs once at program startup):
void setup() {
  // Start Serial port:
  Serial.begin(115200);
  Serial.println("Inicializando configuraciones del controlador");
  Serial.println(" ");

  // Set pin modes:
  pinMode(coilPin, OUTPUT);
  pinMode(redlineLEDPin, OUTPUT);
  pinMode(statusLEDPin, OUTPUT);

  // Start Serial port:
  Serial.println("Fin configuraciones del controlador");
  Serial.println(" ");
  
}

// ---------------------------------------------------------------------------------------------------------------------
// Arduino loop function (runs over and over again):
void loop() {
  // Check if points have been triggered:
  if(checkPoints())
  {
    // Reset points timer to update RPM measurement accurately:
    pointsDiff = micros() - pointsLastMicros;
    pointsLastMicros = micros();
    
    // Calculate how long to wait before firing HEI coil:
    delayTime = timeToFire();

    // Start coil firing timer:
    coilDelayTimeMicros = micros();
    coilDelayWaiting = 1;
  }

  // Check if it is time to fire the coil:
  if(coilDelayWaiting && (micros() - coilDelayTimeMicros > delayTime))
  {
    // Reset waiting flag:
    coilDelayWaiting = 0;
    
    // Check rev limiter and fire coil:
    fireCoil();
  }

  // If coil is ON, check if sufficient time has passed to turn it OFF:
  if(coilOn && (micros() - coilOnTimeMicros > coilDwellTime))
  {
    // Turn OFF coil (make spark):
    coilOn = 0;
    digitalWrite(coilPin, LOW);
  }

  // Check if engine is stalled:
  checkStalled();
  
}
