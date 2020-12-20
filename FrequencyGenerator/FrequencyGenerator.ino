#include <LiquidCrystal.h>
#define LCDREFRESHTIME 50
unsigned long lastScreenRefresh;

// Rotary Encoder Inputs
#define CLK 7
#define DT 8
#define SW 9
int counter = 0;
int currentStateCLK;
int lastStateCLK;
String currentDir ="";
unsigned long lastButtonPress = 0;

// RPM struct
#define MAX_RPM 11000
#define MIN_RPM 150
int currentRPM;
int increments;
const int steps = 50;
const int fastSteps = 500;

// RPM output
#define OUT_PIN 10
unsigned long halfPeriod;
unsigned long lastHalfTick;
bool state;

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  // Setup Serial Monitor
  //Serial.begin(9600);
  
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("RPM Emulator");

  lastScreenRefresh = millis();

  // Set encoder pins as inputs
  pinMode(CLK,INPUT);
  pinMode(DT,INPUT);
  pinMode(SW, INPUT_PULLUP);

  // Read the initial state of CLK
  lastStateCLK = digitalRead(CLK);

  currentRPM = MIN_RPM;
  increments = fastSteps; 
  halfPeriod = 30000000 / currentRPM;
  lastHalfTick = micros();
  pinMode(OUT_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  
}

void loop() {

  // Read the current state of CLK
  currentStateCLK = digitalRead(CLK);

  // If last and current state of CLK are different, then pulse occurred
  // React to only 1 state change to avoid double count
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){

    // If the DT state is different than the CLK state then
    // the encoder is rotating CCW so decrement
    if (digitalRead(DT) != currentStateCLK) {
      counter --;
      currentDir ="CCW";
      currentRPM -= increments;
    } else {
      // Encoder is rotating CW so increment
      counter ++;
      currentDir ="CW";
      currentRPM += increments;
    }

    if (currentRPM > MAX_RPM) currentRPM = MAX_RPM;
    if (currentRPM < MIN_RPM) currentRPM = MIN_RPM;

    halfPeriod = 30000000 / currentRPM;
    lastHalfTick = micros();
  }

  // Remember last CLK state
  lastStateCLK = currentStateCLK;

  // Read the button state
  int btnState = digitalRead(SW);

  //If we detect LOW signal, button is pressed
  if (btnState == LOW) {
    //if 20ms have passed since last LOW pulse, it means that the
    //button has been pressed, released and pressed again
    if (millis() - lastButtonPress > 20) {
      //Serial.println("Button pressed!");
      if (increments == steps) increments = fastSteps;
      else increments = steps;
    }

    // Remember last button press event
    lastButtonPress = millis();
  }

  if (micros() >  lastHalfTick + halfPeriod)
  {
    lastHalfTick += halfPeriod; // should always be > 0
    state = !state;
    digitalWrite(OUT_PIN, state);
    digitalWrite(LED_BUILTIN, state);
  }

  if (millis() - lastScreenRefresh > LCDREFRESHTIME)
  {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("RPM (t/mn)  ");

    if (increments > 100) lcd.print("x"); 
    else if (increments > 10) lcd.print(" x");
    else lcd.print("  x");
    lcd.print(increments);
    
    lcd.setCursor(0, 1);
    lcd.print(currentRPM);
    
    lastScreenRefresh = millis();
  }
}
