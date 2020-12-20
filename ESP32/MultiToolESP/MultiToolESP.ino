
  

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>
#include <ClickButton.h>


#define      LAMBDA_PIN 34
#define         TPS_PIN 31
#define         RPM_PIN 12
#define      BUTTON_PIN 13
#define    OLED_SDA_PIN 21
#define    OLED_SCL_PIN 22
Adafruit_SH1106 display(OLED_SDA_PIN, OLED_SCL_PIN);


#define SERIALPERFCOUNTERS false
#define SAMPLINGRATE 20
#define SCREENREFRESHRATE 20

// Counters
// // global counters
unsigned long global_totalMicros, global_valueMicros, global_logicMicros, global_GPUMicros;
unsigned long UPS;
// // values
unsigned long samplingTime = 1000000/SAMPLINGRATE;
unsigned long local_valuesMicros;
bool sampleRateConstrained = false;
// // screen updates
unsigned long local_GPUMicros;
unsigned long screenRefreshTime = 1000000/SCREENREFRESHRATE; // 0 to unrestrict

// Screen logic
ClickButton button(BUTTON_PIN, LOW, CLICKBTN_PULLUP);
byte state = 0;

// Values reading & smoothing
const int lambda_numReadings = 15;
int lambda_readings[lambda_numReadings];      // the readings from the analog input
int lambda_readIndex = 0;                     // the index of the current reading
int lambda_total = 0;                         // the running total
int lambda_average = 0;                       // the average

// AFR Gauge
#define afrMin 10
#define afrMax 1100
#define pi 3.14159265359
#define pi2 2 * pi
float startAngleD, startAngle;
float endAngleD, endAngle;
int centerX, centerY, radius;
int gaugeMin, gaugeMax;
bool gauge_active;
void InitGauge(int min, int max) {
    float centerD = 270; //Angle where the center of the gauge will be
    float widthD = 40; //Angle that the gauge will be wide
    startAngleD = centerD - widthD;
    endAngleD   = centerD + widthD;
    centerX    = 63;//    Center of arc X (pixels)
    centerY    = 100;//    Center of arc Y (pixels)
    radius     = 65;//    Radious of arc (pixels)
    startAngle = startAngleD / 360 * pi2;
    endAngle   = endAngleD   / 360 * pi2;
    gaugeMin = min;
    gaugeMax = max;
    gauge_active = false;
}

// RPM GAUGE =======
long currentRPM;
bool lastState;
unsigned long lastPickUp;

// =========== LOGIC ===============

void setup()   {                
  Serial.begin(115200);

  ScreenInit();
  BootAnimation();

  InitLambdaSmoothing();
  InitRPM();

  InitGauge(afrMin, afrMax);

  global_totalMicros = micros();

  
}


void loop() {

  UpdateValues();

  ControllerLogic();

  ScreenUpdate();

// -------------
  UPS = micros() - global_totalMicros;
  UPS = 1000000 / UPS;
  if (SERIALPERFCOUNTERS){
    Serial.print("TOTAL : "); Serial.print(micros() - global_totalMicros); 
    Serial.print("("); Serial.print(UPS);Serial.println(")");
  }
  global_totalMicros = micros();    
}

// ============= VALUES ===========

void UpdateValues(){
  global_valueMicros = micros();
// -----------

  if (sampleRateConstrained && (micros()-local_valuesMicros < samplingTime))
  {
    if (!SERIALPERFCOUNTERS)
      return;
    Serial.print("sampling: ");Serial.print(micros() - global_valueMicros); Serial.print(" / ");
    return;
  }
  local_valuesMicros = micros();
  lambda_updateValue();

  UpdateRPM();
    

// -----------
if (!SERIALPERFCOUNTERS)
  return;
Serial.print("sampling: ");Serial.print(micros() - global_valueMicros); Serial.print(" / ");
}

// ============= BUTTONS ===========

void UpdateInputs(){
  button.Update();
}

// ============= SCREENS ===========


void ScreenUpdate(){
  global_GPUMicros = micros();
  // ----------------
  
  if((micros() - local_GPUMicros) < screenRefreshTime)
  {
    return; // only update if enough time has passed
  }

  drawFPS();
  local_GPUMicros = micros();
  
  switch (state){
    case 0: // AFR gauge direct display
    AFRGauge();
    break;

    case 1: // RPM gauge direct display TODO
    DisplayRPM();
    break;

    case 2: // Calibration screen : show min and max TODO
    display.clearDisplay();
    display.setTextColor(INVERSE);
    display.setCursor(15,20);
    display.setTextSize(1);
    display.print("Calibration Screen");
    display.display();
    break;

    case 3: // Recording
    display.clearDisplay();
    display.setTextColor(INVERSE);
    display.setCursor(21,20);
    display.setTextSize(1);
    display.print("Recording screen");
    display.display();
    break;

    default:
    break;
  }
  
// -----------
if (!SERIALPERFCOUNTERS)
  return;
Serial.print("GPU: ");Serial.print(micros() - global_logicMicros); Serial.print(" / ");
}

void ControllerLogic(){
  global_logicMicros = micros();
// -----------
  UpdateInputs();
  if (button.clicks == 1){
    state += 1;
    if (state > 3) state = 0;
    SwitchScreen();
  }

  
// -----------
if (!SERIALPERFCOUNTERS)
  return;
Serial.print("logic: ");Serial.print(micros() - global_logicMicros); Serial.print(" / ");
}

void SwitchScreen(){
  display.clearDisplay();
  gauge_active = false;
}

void SwitchScreen(byte _state){
  display.clearDisplay();
  state = _state;
  gauge_active = false;
}

void drawFPS(){
    display.setTextColor(WHITE, BLACK);
    display.setCursor(0,57);
    display.setTextSize(1);
    display.print(1040000 / (micros() - local_GPUMicros));


    display.setCursor(128-30,57);
    display.print("     ");
    if (UPS > 10000)
      display.setCursor(98,57);  
    else if (UPS > 1000)
      display.setCursor(104,57);
    else if (UPS > 100)
      display.setCursor(110,57);
    else if (UPS > 10)
      display.setCursor(116,57);
    else
      display.setCursor(122,57);
    display.print(UPS);
}

// ============= RPM GAUGE ===============

void InitRPM(){
  pinMode(RPM_PIN, INPUT_PULLUP);
  lastState = LOW;
  lastPickUp = micros();
  currentRPM = 800;
}

void DisplayRPM(){
    display.setTextColor(WHITE,BLACK);
    display.setCursor(6,16);
    display.setTextSize(2);
    display.print("             ");
    display.setCursor(6,16);
    display.print("RPM: ");
    display.println(currentRPM);
    display.display();
}

void UpdateRPM(){
  if (digitalRead(RPM_PIN) == HIGH)
  {
    if (lastState == LOW) // pickup
    {
      lastState = HIGH;
      currentRPM = 60000000 / (micros() - lastPickUp);
      lastPickUp = micros();
    }
  }
  else
  {
    if (lastState == HIGH) lastState = LOW;
  }
}

// ============= AFR GAUGE ===============


void AFRGauge(){  
  if (gauge_active == false){ // init if needed
    display.clearDisplay();
    GaugeBegin();
  }
  drawGaugeData((float)lambda_average);
}

int getLambdaMillivolts(){
  return analogRead(LAMBDA_PIN) * 3300 / 4096;
}


void lambda_updateValue(){
// subtract the last reading:
  lambda_total = lambda_total - lambda_readings[lambda_readIndex];
  // read from the sensor:
  lambda_readings[lambda_readIndex] = getLambdaMillivolts();
  // add the reading to the total:
  lambda_total = lambda_total + lambda_readings[lambda_readIndex];
  // advance to the next position in the array:
  lambda_readIndex = lambda_readIndex + 1;
  // if we're at the end of the array...
  if (lambda_readIndex >= lambda_numReadings) {
    // ...wrap around to the beginning:
    lambda_readIndex = 0;
  }
  // calculate the average:
  lambda_average = lambda_total / lambda_numReadings;
}


void InitLambdaSmoothing(){
  // initialize all the readings to 0:
  for (int lambda_thisReading = 0; lambda_thisReading < lambda_numReadings; lambda_thisReading++) {
    lambda_readings[lambda_thisReading] = 0;
  }
}

void drawValue(float value) {
    display.setTextColor(INVERSE);
    display.setCursor(52,50);
    display.setTextSize(1);
    display.println((int)value);
}
  
float scale(float inScaleMin, float inScaleMax, float outScaleMin, float outScaleMax, float value){
    value = min(value, (float)afrMax);
    return ((value - inScaleMin) / (inScaleMax - inScaleMin) * (outScaleMax-outScaleMin)) + outScaleMin;
} 
  
float angleToXD(float centerX, float radius, float angleD) {
    float angle = (angleD / 360) * (pi2);
    return centerX+radius*cos(angle); // Calculate arc point (X)
}
float angleToYD(float centerY, float radius, float angleD) {
    float angle = (angleD / 360) * (pi2);
    return centerY+radius*sin(angle); // Calculate arc point (Y)
}

void drawArc(float startAngle, float endAngle, float segments, int centerX, int centerY, int radius) {
    float resolution = (endAngle-startAngle)/segments; // Calculates steps in arc based on segments
    float x = centerX+radius*cos(startAngle); // Calculate start point of arc (X)
    float y = centerY+radius*sin(startAngle); // Calculate start point of arc (Y)
    display.writePixel(x,y,WHITE); // Place starting point of arc
  
    for (float angle = startAngle; angle < endAngle; angle += resolution) { // Sweep arc
        x = centerX+radius*cos(angle); // Calculate arc point (X)
        y = centerY+radius*sin(angle); // Calculate arc point (Y)
        display.writePixel(x,y,WHITE);
    }
}

void drawNeedle(float angle, float startAngle, float endAngle, float centerX, float centerY, int radius, int color){
    float leftX = angleToXD(centerX, radius+1, angle - 5);
    float leftY = angleToYD(centerY, radius+1, angle - 5);
  
    float rightX = angleToXD(centerX, radius+1, angle + 5);
    float rightY = angleToYD(centerY, radius+1, angle + 5);
  
    float topX = angleToXD(centerX, radius+30, angle);
    float topY = angleToYD(centerY, radius+30, angle);
  
    display.fillTriangle(leftX,leftY,topX,topY,rightX,rightY,color);
}

void drawGaugeLines(float startAngle, float endAngle, float centerX, float centerY, int radius){
    drawArc(startAngle, endAngle, 150, centerX, centerY, radius + 30);
    drawArc(startAngle, endAngle, 110, centerX, centerY, radius - 1);
    drawArc(startAngle, endAngle, 110, centerX, centerY, radius - 4);
}

void drawGaugeFrame() {
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    
    display.setCursor(0, 0);
    display.println("Lean");
    display.setCursor(100, 0);
    display.println("Rich");
  
    display.setTextSize(2);
    display.setCursor(45,15);
    display.println("AFR");
  
    drawGaugeLines(startAngle, endAngle, centerX, centerY, 65);
}

void drawGaugeData(float value) {
    float angle = scale(gaugeMin,gaugeMax,startAngleD,endAngleD,value);
  
    drawValue(value);
    drawNeedle(angle, startAngle, endAngle, centerX, centerY, radius, INVERSE);  
    display.display();
    drawNeedle(angle, startAngle, endAngle, centerX, centerY, radius, INVERSE); //erase the needle
    drawValue(value); // erase the value
}

void GaugeBegin() {
    drawGaugeFrame();
    gauge_active = true;
}

// ===============BOOT SEQUENCE ==============

void BootAnimation(){
  // Display Text
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);
  display.println("Connecting to SD.");
  display.display();

  delay(500);

  display.println("SD Ready.");
  display.display();

  delay(500);

  display.print("Loading gauge");
  display.display();

  for (int i = 0 ; i < 4 ; i++){
    delay(100);
    i == 4 ? display.println('.') : display.print('.');
    display.display();
  }

  SwitchScreen(0);
}

void ScreenInit(){
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SH1106_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  Wire.setClock(400000);
  // init done

  display.setRotation(2);

  SplashScreen();
}

void SplashScreen() {
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(2000);

  // Clear the buffer.
  display.clearDisplay();
  display.display();
}
