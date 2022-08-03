#include <Arduino.h>
#include <A4988.h>
#include <movingAvg.h>
#include <Bounce2.h> // library to debounce the push button

// using a 200-step motor (most common)
#define MOTOR_STEPS 200

// configure the pins connected
#define DIR 8
#define STEP 9
#define MS1 10
#define MS2 11
#define MS3 12
A4988 stepper(MOTOR_STEPS, DIR, STEP, MS1, MS2, MS3);

// configure the speed of the pump
#define RPM 36
#define MICROSTEPS 1
#define FDEGREES 400 // how many steps to rotate in a forward (infusion) direction 
#define BDEGREES 150 // how many steps to rotate in a backward (withdrawal) direction 

const uint8_t Lick = 7;    // pushbutton connected to digital pin 7
const uint8_t Pushbut = 6;    // red push button connected to digital pin 6
const uint8_t Switch = 5;    // toggle switch above red push button connected to digital pin 5
const uint8_t REWARDLED = 4;    // pushbutton connected to digital pin 4
const uint8_t TOUCHLED = 3;    // pushbutton connected to digital pin 3
const uint8_t Enable = 2;    // enables pump, buzzer, and sensor
const uint8_t Buzzer = 12;     // buzzer pushbutton conected to digital pin 12
unsigned int Pushval = 0;      // red push button
unsigned int Switchval = 0;      // toggle switch above red push button

const uint16_t SETUP_INTERVAL = 250; // ms
const uint16_t LOOP_INTERVAL = 5; // ms Specifies how frequently the licker is read
const uint8_t LICKER_MEASUREMENTS = 30;
// reward parameters (in ms)
const uint16_t PUMP_INFUSION = 2000; 
const uint16_t PUMP_WITHDRAWAL = 2500; 
const uint16_t BUZZER_INTERVAL = 1000;

uint32_t loop_time;
uint32_t buzzer_time;
uint32_t pumpf_time; 
uint32_t pumpb_time;
uint32_t lick_time;
bool reward;
String command = "";
bool licked; // a global variable to report if lick has occured
bool licked_wait; // a global variable to report that a set amount of time has elapsed from the recorded lick

// threshold for a minimum deviation of sensor reading to signal that licking has occured
float threshold;
const uint8_t min_threshold = 7;
// mean baseline value of the sensor
float avg;
Bounce button = Bounce(); // Instantiate a Bounce object

// defines possible states
enum states {
  NONE, // NONE state is just for setup
  WAITING,
  BUZZER,
  PUMPF, // PUMP forward rotation state
  PUMPB  // PUMP backward rotation state
};

states prior_state, state; // Global variables to store the prior and current states

/*
** Returns a boolean value that indicates whether the current time, t, is later than some prior 
** time, t0, plus a given interval, dt.  The condition accounts for timer overflow / wraparound.
*/
bool it_is_time(uint32_t t, uint32_t t0, uint16_t dt) {
  return ((t >= t0) && (t - t0 >= dt)) ||         // The first disjunct handles the normal case
            ((t < t0) && (t + (~t0) + 1 >= dt));  //   while the second handles the overflow case
}


/*
 * Get the standard deviation from an array of ints
 */
float getStdDev(int* vals, float avg, int arrayCount) {
  long total = 0;
  for (int i = 0; i < arrayCount; i++) {
    total = total + sq((vals[i] - avg));
  }

  float variance = total/(float)arrayCount;
  float stdDev = sqrt(variance);
  return stdDev;
}

bool parse_command() {
  return command.equals("reward");
}  

void waiting() {
    Switchval = digitalRead(Switch); // reads the state of the toggle switch above red push button
    button.update();

    // receive serial communication from Psychopy
    if (Serial.available()) {
      command = Serial.readStringUntil('\r');
      command.trim();
      reward = parse_command();
    }
    
    if (Switchval == HIGH) { // toggle switch turned to the right
      if (state != prior_state) {   // If we are entering the state, do initialization stuff
         prior_state = state;
         digitalWrite(Enable, LOW);
      }
   
      if (button.fell() || (reward)) { 
         state = BUZZER;
      }
  } 
  if (Switchval == LOW && button.fell()) {
    delay(10);
    stepper.rotate(3000);
  }
}

void buzzer() {
  uint32_t t;
  t = millis();
 
  if (state != prior_state) {   // If we are entering the state, do initialization stuff
     prior_state = state; 
     buzzer_time = millis();
     digitalWrite(Buzzer, HIGH);
     digitalWrite(REWARDLED, HIGH);
     digitalWrite(TOUCHLED, HIGH);
     }

  if (it_is_time(t, buzzer_time, BUZZER_INTERVAL)) {
    digitalWrite(Buzzer, LOW);  
    state = PUMPF;
  }  
}
  
void pumpf() {
  uint32_t t;
  uint16_t res;
  float deviation;  
  if (state != prior_state) {   // If we are entering the state, do initialization stuff
     prior_state = state;
     stepper.rotate(FDEGREES);
     delay(10);
     pumpf_time = millis();
     licked = false;
     licked_wait = false;
  }
  button.update();
  t = millis();
  // the state advances to PUMPB only marmoset has licked 
  res = analogRead(A0); 
  deviation = abs(res - avg);
  if ( (deviation > threshold) && !licked ) {
    lick_time = millis(); 
    licked = true;
    Serial.print("licked");
    Serial.print("\r\n"); 
  }
  if (it_is_time(t, lick_time, 1000)) {
    licked_wait = true;
  }
  
  if( (it_is_time(t, pumpf_time, PUMP_INFUSION)) && licked_wait ) {
    state = PUMPB;
  }
  // pressing the red push button also advances to the next state and send command to Psychopy to terminate the training session
  if (button.fell()) {
    Serial.print("terminate");
    Serial.print("\r\n"); 
    state = PUMPB;
  }
}

void pumpb() {
  uint32_t t;
  if (state != prior_state) {   // If we are entering the state, do initialization stuff
     prior_state = state;
     digitalWrite(REWARDLED, LOW);
     digitalWrite(TOUCHLED, LOW);
     delay(100);
     stepper.rotate(-BDEGREES);
     pumpb_time = millis();
  }
  
  t = millis(); 
  if (it_is_time(t, pumpb_time, PUMP_WITHDRAWAL)) {
     digitalWrite(Enable, HIGH);
     state = WAITING; 
  }
}

void setup() {
  // Set target motor RPM to 1RPM and microstepping to 1 (full step mode)
  stepper.begin(RPM, MICROSTEPS);
  stepper.enable();

  button.attach(Pushbut, INPUT_PULLUP); // Attach the debouncer to a pin with INPUT_PULLUP mode
  button.interval(25); // Use a debounce interval of 25 milliseconds
 
  pinMode(Lick, INPUT);  // sets the digital pin 7 as input
  pinMode(Switch, INPUT);  // sets the digital pin 5 as input
  pinMode(REWARDLED, OUTPUT);  // sets the digital pin 4 as output
  pinMode(TOUCHLED, OUTPUT);  // sets the digital pin 3 as output
  pinMode(Enable, OUTPUT);  // sets the digital pin 2 as output
  pinMode(Buzzer, OUTPUT); // sets the digital pin 12 as ouput
  digitalWrite(REWARDLED, LOW);
  digitalWrite(TOUCHLED, LOW);
  digitalWrite(Enable, LOW);
  Serial.begin(115200);

  reward = false;
  licked = false;
  licked_wait = false;

  // take baseline sensor readings
  uint16_t res;
  float sd;
  delay(SETUP_INTERVAL);
  movingAvg lickerMean(LICKER_MEASUREMENTS);
  lickerMean.begin();
  for (int i = 0; i < LICKER_MEASUREMENTS; i++) {
     res = analogRead(A0);
     lickerMean.reading(res);
     delay(SETUP_INTERVAL); 
  }
  avg = lickerMean.getAvg();
  sd = getStdDev(lickerMean.getReadings(), avg, LICKER_MEASUREMENTS);
  Serial.print("Average is: ");
  Serial.println(avg); 
  Serial.print("Standard deviation is: ");
  Serial.println(sd); 
  threshold = 5 * sd;
  threshold = max(threshold, min_threshold);
  prior_state = NONE;
  state = WAITING;
}

void loop() {
  switch (state) {
    case WAITING:
      waiting();
      break;
    case BUZZER:
      buzzer();
      break;
    case PUMPF:
      pumpf();
      break;
    case PUMPB:
      pumpb();
      break;
  }
}
