#include <SPI.h>              // SPI connection (SD & oled screen)
#include <Wire.h>             // I2C screen connection
#include <Adafruit_GFX.h>     // GFX
#include <Adafruit_SH1106.h>  // OLED screen
#include <ClickButton.h>      // button debounce
#include <SD.h>               // SD card

#define VERSION "1.0"
#define SDFILENAME "/MultitoolRecords.txt"

#define      LAMBDA_PIN 34
#define         TPS_PIN 35
#define         RPM_PIN 12
#define      BUTTON_PIN 13
#define    OLED_SDA_PIN 21
#define    OLED_SCL_PIN 22

Adafruit_SH1106 display(OLED_SDA_PIN, OLED_SCL_PIN);

#define SERIAL_PERF_COUNTERS_ON false
#define SAMPLING_RATE 20
#define LOGGING_RATE 12
#define SCREEN_REFRESH_RATE 20
#define BOOT_SEQUENCE_TIME_INTERVAL 170
#define CALIBRATION_INTERVAL 4000000 // in us
#define ICON_DISPLAY_INTERVAL 1500000 // in us

#pragma region Counters
// // global counters
unsigned long global_totalMicros, global_valueMicros, global_logicMicros, global_GPUMicros;
unsigned long UPS;
// // values
unsigned long samplingTime = 1000000/SAMPLING_RATE;
unsigned long local_valuesMicros;
bool sampleRateConstrained = false;
// // screen updates
unsigned long local_GPUMicros;
unsigned long screenRefreshTime = 1000000/SCREEN_REFRESH_RATE; // 0 to unrestrict
//  Data logging
unsigned long local_dataLogMicros;
unsigned long dataLoggingTime = 1000000/LOGGING_RATE;
unsigned long fileSize;
// // Calibration
unsigned long local_CalibrationMicros;
// // BMP display
unsigned long local_bmpMicros;
#pragma endregion

#pragma region Controller logic
ClickButton button(BUTTON_PIN, LOW, CLICKBTN_PULLUP);
byte state = 0;
#pragma endregion

#pragma region Values
// Lambda reading and smoothing
const int lambda_numReadings = 15;
int lambda_readings[lambda_numReadings];      // the readings from the analog input
int lambda_readIndex = 0;                     // the index of the current reading
int lambda_total = 0;                         // the running total
int lambda_average = 0;                       // the average

// RPM reading
long currentRPM;
bool lastState;
unsigned long lastPickUp;
#pragma endregion

#pragma region AFR Gauge variables
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
#pragma endregion

#pragma region SDCard

SDFile myFile;
String dataString = "";
String formattedFileSize = "";
bool recording = false;
int recordCounter;
bool recordValid;
bool hasTheFileBeenReset = false;
bool hasTheFileBeenRemoved = false;

#pragma endregion

#pragma region TPS (Software)

const int tps_numReadings = 15;
int tps_readings[tps_numReadings];      // the readings from the analog input
int tps_readIndex = 0;                     // the index of the current reading
int tps_total = 0;                         // the running total
int tps_average = 0;                       // the average

bool calibrating = false;
bool hasCalibrated = false;

//float start_position = 0.0f;
int TPSMaxValue;
int TPSMinValue;

#pragma endregion

#pragma region Bitmaps

bool bmpDisplay;
bool bmpHasDisplayed;

#define LOGO_HEIGHT   48
#define LOGO_WIDTH    48
static const uint8_t PROGMEM CalibrateLogo [] {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xe0, 0x00, 0x00, 
0x00, 0x00, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 
0x00, 0x00, 0x00, 0x00, 0x20, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x20, 0xe0, 0x00, 0x00, 0x00, 0x00, 
0x20, 0x20, 0x00, 0x10, 0x00, 0x00, 0x20, 0x20, 0x00, 0x6c, 0x00, 0x00, 0x20, 0x20, 0x01, 0x83, 
0x00, 0x00, 0x20, 0xe0, 0x06, 0x00, 0xc0, 0x00, 0x20, 0xe0, 0x18, 0x00, 0x30, 0x00, 0x20, 0x20, 
0x60, 0x00, 0x0c, 0x00, 0x20, 0x20, 0x70, 0x00, 0x1c, 0x00, 0x20, 0x20, 0x58, 0x00, 0x34, 0x00, 
0x20, 0xe0, 0x46, 0x00, 0xc4, 0x00, 0x20, 0xe0, 0x41, 0x83, 0x04, 0x00, 0x20, 0x20, 0x40, 0x6c, 
0x04, 0x00, 0x20, 0x20, 0x40, 0x10, 0x04, 0x00, 0x20, 0x20, 0x40, 0x10, 0x04, 0x00, 0x20, 0xe0, 
0x40, 0x10, 0x04, 0x00, 0x20, 0xe0, 0x40, 0x10, 0x04, 0x00, 0x20, 0x20, 0x40, 0x10, 0x04, 0x00, 
0x20, 0x20, 0x40, 0x10, 0x04, 0x00, 0x20, 0x20, 0x40, 0x10, 0x04, 0x00, 0x20, 0xe0, 0x60, 0x10, 
0x0c, 0x00, 0x20, 0xe0, 0x18, 0x10, 0x30, 0x00, 0x20, 0x20, 0x06, 0x10, 0xc0, 0x00, 0x20, 0x20, 
0x01, 0x93, 0x00, 0x00, 0x20, 0x20, 0x00, 0x54, 0x00, 0x00, 0x20, 0xe0, 0x00, 0x38, 0x00, 0x00, 
0x20, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x20, 0x20, 0x00, 0x00, 
0x00, 0x00, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 
0xff, 0xff, 0xff, 0xfc, 0x00, 0x11, 0x8c, 0x63, 0x18, 0xc4, 0x00, 0x11, 0x8c, 0x63, 0x18, 0xc4, 
0x00, 0x10, 0x00, 0x00, 0x00, 0x04, 0x00, 0x10, 0x00, 0x00, 0x00, 0x04, 0x00, 0x10, 0x00, 0x00, 
0x00, 0x04, 0x00, 0x10, 0x00, 0x00, 0x00, 0x04, 0x00, 0x10, 0x00, 0x00, 0x00, 0x04, 0x00, 0x1f, 
0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t PROGMEM BeginRecordingLogo [] {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 
0x00, 0x00, 0x00, 0x03, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x3f, 
0xff, 0xff, 0xfc, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x00, 
0x01, 0xff, 0xff, 0xff, 0xff, 0x80, 0x03, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x07, 0xff, 0xff, 0xff, 
0xff, 0xe0, 0x07, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x0f, 0xff, 
0xff, 0xff, 0xff, 0xf0, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xf8, 
0x3f, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x3f, 0xff, 0xff, 0xff, 
0xff, 0xfc, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x7f, 0xff, 
0xff, 0xff, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xfe, 
0x7f, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 
0xff, 0xfe, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x3f, 0xff, 
0xff, 0xff, 0xff, 0xfc, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xfc, 0x3f, 0xff, 0xff, 0xff, 0xff, 0xfc, 
0x1f, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x1f, 0xff, 0xff, 0xff, 0xff, 0xf8, 0x0f, 0xff, 0xff, 0xff, 
0xff, 0xf0, 0x0f, 0xff, 0xff, 0xff, 0xff, 0xf0, 0x07, 0xff, 0xff, 0xff, 0xff, 0xe0, 0x07, 0xff, 
0xff, 0xff, 0xff, 0xe0, 0x03, 0xff, 0xff, 0xff, 0xff, 0xc0, 0x01, 0xff, 0xff, 0xff, 0xff, 0x80, 
0x00, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xfe, 0x00, 0x00, 0x3f, 0xff, 0xff, 
0xfc, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x03, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 
0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t PROGMEM TrashLogo [] {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0xff, 0xff, 
0xe0, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xe0, 0x00, 0x00, 0x18, 0x00, 0x00, 0x60, 0x00, 0x00, 0x18, 
0x00, 0x00, 0x60, 0x00, 0x00, 0x18, 0x00, 0x00, 0x60, 0x00, 0x00, 0x1f, 0xff, 0xff, 0xe0, 0x00, 
0x7f, 0xff, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7f, 0xff, 0xff, 0xff, 0xff, 0xfe, 0x01, 0xc0, 
0x00, 0x00, 0x07, 0x00, 0x01, 0xc0, 0x00, 0x00, 0x07, 0x00, 0x01, 0xc7, 0x03, 0x81, 0xc7, 0x00, 
0x01, 0xc7, 0x03, 0x81, 0xc7, 0x00, 0x01, 0xc7, 0x03, 0x81, 0xc7, 0x00, 0x01, 0xc7, 0x03, 0x81, 
0xc7, 0x00, 0x01, 0xc7, 0x03, 0x81, 0xc7, 0x00, 0x01, 0xc7, 0x03, 0x81, 0xc7, 0x00, 0x01, 0xc7, 
0x03, 0x81, 0xc7, 0x00, 0x01, 0xc7, 0x03, 0x81, 0xc7, 0x00, 0x01, 0xc7, 0x03, 0x81, 0xc7, 0x00, 
0x01, 0xc7, 0x03, 0x81, 0xc7, 0x00, 0x01, 0xc7, 0x03, 0x81, 0xc7, 0x00, 0x01, 0xc7, 0x03, 0x81, 
0xc7, 0x00, 0x01, 0xc7, 0x03, 0x81, 0xc7, 0x00, 0x01, 0xc7, 0x03, 0x81, 0xc7, 0x00, 0x01, 0xc7, 
0x03, 0x81, 0xc7, 0x00, 0x01, 0xc7, 0x03, 0x81, 0xc7, 0x00, 0x01, 0xc7, 0x03, 0x81, 0xc7, 0x00, 
0x01, 0xc7, 0x03, 0x81, 0xc7, 0x00, 0x01, 0xc7, 0x03, 0x81, 0xc7, 0x00, 0x01, 0xc7, 0x03, 0x81, 
0xc7, 0x00, 0x01, 0xc7, 0x03, 0x81, 0xc7, 0x00, 0x01, 0xc7, 0x03, 0x81, 0xc7, 0x00, 0x01, 0xc7, 
0x03, 0x81, 0xc7, 0x00, 0x01, 0xc7, 0x03, 0x81, 0xc7, 0x00, 0x01, 0xc7, 0x03, 0x81, 0xc7, 0x00, 
0x01, 0xc7, 0x03, 0x81, 0xc7, 0x00, 0x01, 0xe0, 0x00, 0x00, 0x0e, 0x00, 0x00, 0xe0, 0x00, 0x00, 
0x0e, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x3e, 0x00, 0x00, 0x7f, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x3f, 
0xff, 0xff, 0xf8, 0x00, 0x00, 0x0f, 0xff, 0xff, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t PROGMEM StopRecordingLogo [] {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 
0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 
0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 
0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 
0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 
0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 
0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 
0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 
0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 
0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 
0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 
0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 
0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 
0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 
0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 
0xff, 0xe0, 0x0f, 0xff, 0xfc, 0x7f, 0xff, 0xe0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static const uint8_t PROGMEM NotCalibratedLogo [] {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xe0, 0x00, 0x00, 
0x00, 0x00, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x20, 0x21, 0xc0, 0x00, 0x00, 0xe0, 0x20, 0x23, 
0xe0, 0x00, 0x01, 0xf0, 0x20, 0xe3, 0xf0, 0x00, 0x03, 0xf0, 0x20, 0xe3, 0xf8, 0x00, 0x07, 0xf0, 
0x20, 0x21, 0xfc, 0x00, 0x0f, 0xe0, 0x20, 0x20, 0xfe, 0x00, 0x1f, 0xc0, 0x20, 0x20, 0x7f, 0x00, 
0x3f, 0x80, 0x20, 0xe0, 0x3f, 0x80, 0x7f, 0x00, 0x20, 0xe0, 0x1f, 0x80, 0xfe, 0x00, 0x20, 0x20, 
0x0f, 0xc1, 0xfc, 0x00, 0x20, 0x20, 0x0f, 0xe3, 0xf8, 0x00, 0x20, 0x20, 0x07, 0xf7, 0xf0, 0x00, 
0x20, 0xe0, 0x03, 0xff, 0xe0, 0x00, 0x20, 0xe0, 0x01, 0xff, 0xc0, 0x00, 0x20, 0x20, 0x00, 0xff, 
0x80, 0x00, 0x20, 0x20, 0x00, 0x7f, 0x00, 0x00, 0x20, 0x20, 0x00, 0xff, 0x80, 0x00, 0x20, 0xe0, 
0x01, 0xff, 0xc0, 0x00, 0x20, 0xe0, 0x03, 0xff, 0xe0, 0x00, 0x20, 0x20, 0x07, 0xf7, 0xf0, 0x00, 
0x20, 0x20, 0x0f, 0xe3, 0xf0, 0x00, 0x20, 0x20, 0x1f, 0xc1, 0xf8, 0x00, 0x20, 0xe0, 0x3f, 0x81, 
0xfc, 0x00, 0x20, 0xe0, 0x7f, 0x00, 0xfe, 0x00, 0x20, 0x20, 0xfe, 0x00, 0x7f, 0x00, 0x20, 0x21, 
0xfc, 0x00, 0x3f, 0x80, 0x20, 0x23, 0xf8, 0x00, 0x1f, 0xc0, 0x20, 0xe3, 0xf0, 0x00, 0x0f, 0xc0, 
0x20, 0xe3, 0xe0, 0x00, 0x07, 0xc0, 0x20, 0x21, 0xc0, 0x00, 0x03, 0x80, 0x20, 0x20, 0x00, 0x00, 
0x00, 0x00, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x3f, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3f, 
0xff, 0xff, 0xff, 0xfc, 0x00, 0x11, 0x8c, 0x63, 0x18, 0xc4, 0x00, 0x11, 0x8c, 0x63, 0x18, 0xc4, 
0x00, 0x10, 0x00, 0x00, 0x00, 0x04, 0x00, 0x10, 0x00, 0x00, 0x00, 0x04, 0x00, 0x10, 0x00, 0x00, 
0x00, 0x04, 0x00, 0x10, 0x00, 0x00, 0x00, 0x04, 0x00, 0x10, 0x00, 0x00, 0x00, 0x04, 0x00, 0x1f, 
0xff, 0xff, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

#pragma endregion

// =========== LOGIC ===============

void setup()   {                
  Serial.begin(115200);

  // Booting with feedback on screen
  ScreenInit();
  SDInit();
  InputsInit();

  delay(BOOT_SEQUENCE_TIME_INTERVAL);

  InitLambdaSmoothing();
  InitRPM();
  TPSInit();

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
  if (SERIAL_PERF_COUNTERS_ON){
    Serial.print("TOTAL : "); Serial.print(micros() - global_totalMicros); 
    Serial.print("("); Serial.print(UPS);Serial.println(")");
  }
  global_totalMicros = micros();    
}

// ============= VALUES ============

void UpdateValues(){
  global_valueMicros = micros();
// -----------

  if (sampleRateConstrained && (micros()-local_valuesMicros < samplingTime))
  {
    if (!SERIAL_PERF_COUNTERS_ON)
      return;
    Serial.print("sampling: ");Serial.print(micros() - global_valueMicros); Serial.print(" / ");
    return;
  }
  local_valuesMicros = micros();

  lambda_updateValue();

  UpdateRPM();

  UpdateTPS();
    

// -----------
if (!SERIAL_PERF_COUNTERS_ON)
  return;
Serial.print("sampling: ");Serial.print(micros() - global_valueMicros); Serial.print(" / ");
}

// ============= BUTTONS ===========

void UpdateInputs(){
  button.Update();
}

void InputsInit(){
  button.debounceTime = 20;
  button.multiclickTime = 250;
}

// =============BMP display ========

void BMPShow(const uint8_t * logo){ // only once
  bmpDisplay = true;
  gauge_active = false; // just in case we were in gauge screen, make sure to redraw gauge after display
  display.fillRect(0,0, 128, 57, BLACK); // clear upper screen
  display.drawBitmap(
    (display.width()  - LOGO_WIDTH) / 2,
    (57 - LOGO_HEIGHT) / 2,
    logo,
    LOGO_WIDTH,
    LOGO_HEIGHT,
    WHITE);
  local_bmpMicros = micros();
}

bool BMPShouldDisplay(){
  if (!bmpDisplay) return false;

  if (micros() > ICON_DISPLAY_INTERVAL + local_bmpMicros)
  {
    bmpDisplay = false;
  }

  return bmpDisplay;
}

void BMPClear(){
  bmpDisplay = false;
  display.fillRect(0,0, 128, 57, BLACK); // clear upper screen
}

// =============== TPS =============

void TPSLogic(){
  if (button.clicks == -2) // long click ; enable calibration mode
  {
    calibrating = !calibrating;
    local_CalibrationMicros = micros();
    TPSMinValue = tps_average;
    TPSMaxValue = tps_average;
    if (calibrating)
      BMPShow(CalibrateLogo);
  }

  if (calibrating){
    TPSCalibrate(tps_average);
  }
  
}

void UpdateTPS(){

  // subtract the last reading:
  tps_total = tps_total - tps_readings[tps_readIndex];
  // read from the sensor:
  tps_readings[tps_readIndex] = getTPSPosition();
  // add the reading to the total:
  tps_total = tps_total + tps_readings[tps_readIndex];
  // advance to the next position in the array:
  tps_readIndex = tps_readIndex + 1;
  // if we're at the end of the array...
  if (tps_readIndex >= tps_numReadings) {
    // ...wrap around to the beginning:
    tps_readIndex = 0;
  }
  // calculate the average:
  tps_average = tps_total / tps_numReadings;
}

int getTPSPosition(){
  return analogRead(TPS_PIN);
}

void TPSCalibrate(int value){
  if (!calibrating) return;

  if (micros() > local_CalibrationMicros + 4000000){
    calibrating = false;
    hasCalibrated = true;
    return;
  }

  TPSMaxValue = max(TPSMaxValue, value);
  TPSMinValue = min(TPSMinValue, value);
}

void TPSScreen(){
    display.fillRect(0,0, 128, 57, BLACK);
    display.setTextSize(2);
    display.setCursor(0,0);
    if (calibrating)
      display.print("CALIBRATING");
    else
      display.print("TPS        ");
    display.setTextSize(1);
    display.drawLine(0, 19, 126, 19, WHITE);

    display.setCursor(0,24);
    display.print("MIN : ");
    display.println(TPSMinValue);
    display.print("VAL : ");
    display.println(tps_average);
    display.print("MAX : ");
    display.println(TPSMaxValue);

    display.display();
}

void TPSInit(){
  
  calibrating = false;
  hasCalibrated = false;

  TPSMaxValue = 700;
  TPSMinValue = 0;
}

// ============= SD ================

void SDScreen(){
  display.fillRect(0,0, 128, 57, BLACK);

  display.setTextSize(1);
  display.drawLine(0, 19, 126, 19, WHITE);

  if (recording)
  {
    display.fillCircle(114,7,6, WHITE);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(0,0);
    display.setTextSize(2);
    display.println("LOGGING");
    display.setTextSize(1);
  }
  else
  {
    display.fillCircle(114,7,6, BLACK);
    display.setTextColor(WHITE, BLACK);
    display.setCursor(0,0);
    display.setTextSize(2);
    display.println("LOG    ");
    display.setTextSize(1);
  }

  display.setCursor(0,24);
  display.print("SD:");
  if (SD.cardType() != CARD_NONE)
  {
    display.setCursor(18,24);
    if(recordValid)
    {
      display.print("OK");
      display.print(" (");
      if (fileSize/1000 > 100) display.print("-");
      else display.print(fileSize/1000);
      display.println(" kB)");
    } else
    {
      display.println("ERROR");
      display.println("PLEASE FORMAT SD");
      display.display();
      return;
    }
    
    display.print("SR:"); display.print(LOGGING_RATE); display.println(" Hz");

    display.setTextSize(2);
    display.setCursor(128-(12*4),24);
    display.setTextColor(WHITE, BLACK);
    display.print("    ");

    if (recordCounter >= 1000)
      display.setCursor(128-(12*4),24);
    else if (recordCounter >= 100)
      display.setCursor(128-(12*3),24);
    else if (recordCounter >= 10)
      display.setCursor(128-(12*2),24);
    else
      display.setCursor(128-12,24);
    display.print(recordCounter);
    display.setTextSize(1);
  }  
  else
  {
    display.println("Not Mounted");
    display.display();
    return;
  }

  display.setCursor(0,44);
  if (hasTheFileBeenReset)
    display.print("File reset!");
  else if (hasTheFileBeenRemoved)
    display.print("File deleted!");
  else
    display.print("             ");

  display.display();
}

void SDOpenLogFile(bool erase = false){
  if (myFile != 0) // if already open, close it
    SDCloseFile();
  
  if (erase)
  {
    myFile = SD.open(SDFILENAME, FILE_WRITE);
    fileSize = myFile.size();
    hasTheFileBeenReset = true;
    hasTheFileBeenRemoved = false;
  }
  else
  {
    myFile = SD.open(SDFILENAME, FILE_APPEND);
    fileSize = myFile.size();
    hasTheFileBeenReset = false;
    hasTheFileBeenRemoved = false;
  }
}

void SDLogic(){

  if (button.clicks == -1) // long click ; switch record on/off
  {
    if (!hasCalibrated)
    {
      BMPShow(NotCalibratedLogo);
    }
    else
    {
      if (recording)  StopRecording();
      else            BeginRecording();
    }
  }

  if (button.clicks==-3 && !recording)
  {
    SD.remove(SDFILENAME);
    hasTheFileBeenRemoved = true;
    BMPShow(TrashLogo);
  }

  if (recording) SDRecord();
}

void BeginRecording(){
  if (recording) return;
  if (myFile == 0) SDOpenLogFile();

  recording = true;
  hasTheFileBeenReset = false;
  hasTheFileBeenRemoved = false;

  BMPShow(BeginRecordingLogo);

  // register min and max value at time = -1 for reference
  AppendRecord(0, TPSMinValue, 0, 0);
  AppendRecord(0, TPSMaxValue, 0, 0);

  recordCounter = 0;
  
  local_dataLogMicros = micros();
}

void StopRecording(){
  fileSize = myFile.size();
  SDCloseFile();
  recording = false;
  hasTheFileBeenReset = false;
  hasTheFileBeenRemoved = false;
  BMPShow(StopRecordingLogo);
}

void AppendRecord(unsigned long time, int TPS, int lambda, int RPM ){
  dataString = "";
  // gather data
  dataString += time;
  dataString += ",";
  dataString += TPS;
  dataString += ",";
  dataString += lambda;
  dataString += ",";
  dataString += RPM;

  if (myFile.println(dataString)){
    recordCounter++;
    recordValid = true;
  }
}

void SDRecord(){
  if (myFile == 0) return;

  if ((micros() - local_dataLogMicros) < dataLoggingTime)
    return;
  
  local_dataLogMicros = micros();

  AppendRecord(millis(), tps_average, lambda_average, currentRPM);
}

void SDCloseFile(){
  if (myFile == 0) return;
  fileSize = myFile.size();
  myFile.close();
}

void SDInit(){

  Serial.println("Initializing SD Card");
    display.println("Initializing SD Card");
    display.display();
    delay(BOOT_SEQUENCE_TIME_INTERVAL);

  if(!SD.begin()){
    Serial.println("Card Mount Failed");
    display.println("Card Mount Failed");
    display.display();
    delay(BOOT_SEQUENCE_TIME_INTERVAL);
    return;
  } else {
    Serial.println("Card Mounted");
    display.println("Card Mounted");
    display.display();
    delay(BOOT_SEQUENCE_TIME_INTERVAL);
  }

  uint8_t cardType = SD.cardType();

  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    display.println("No SD card attached");
    display.display();
    delay(BOOT_SEQUENCE_TIME_INTERVAL);
    return;
  }

  Serial.print("SD Card Type: ");
  display.print("SD Card Type: ");
  if(cardType == CARD_MMC){
    Serial.println("MMC");
    display.println("MMC");
  } else if(cardType == CARD_SD){
    Serial.println("SDSC");
    display.println("SDSC");
  } else if(cardType == CARD_SDHC){
    Serial.println("SDHC");
    display.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
    display.println("UNKNOWN");
  }

  display.display();
  delay(BOOT_SEQUENCE_TIME_INTERVAL);

  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize); Serial.println();
  display.printf("SD Card Size: %lluMB\n", cardSize); display.println();

  display.display();
  delay(BOOT_SEQUENCE_TIME_INTERVAL);

  SDOpenLogFile(false);
  fileSize = myFile.size();
  SDCloseFile();
  recordValid = true;

  Serial.print("Log file size: " );Serial.print(fileSize/1000);; Serial.println(" kB");
  display.print("Log file size: " );display.print(fileSize/1000);; display.println(" kB");

  display.display();
  delay(BOOT_SEQUENCE_TIME_INTERVAL* 2);
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

  if (BMPShouldDisplay())
  {
    display.display();
    if (SERIAL_PERF_COUNTERS_ON)
      Serial.print("GPU: ");Serial.print(micros() - global_logicMicros); Serial.print(" / ");
    return;
  }

  switch (state){
    case 0: // AFR gauge direct display
    AFRGauge();
    break;

    case 1: // RPM
    DisplayRPM();
    break;

    case 2: // TPS
    TPSScreen();
    break;

    case 3: // LOGGING
    SDScreen();
    break;

    default:
    break;
  }
  
// -----------
if (!SERIAL_PERF_COUNTERS_ON)
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

  SDLogic();

  TPSLogic();

  
// -----------
if (!SERIAL_PERF_COUNTERS_ON)
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
    //display.drawLine(0,55,128,55, WHITE); // too much!
    display.setTextColor(WHITE, BLACK);
    display.setCursor(0,57);
    display.setTextSize(1);
    display.print(1040000 / (micros() - local_GPUMicros));

    display.setCursor(24,57);
    if (recording)
    {
      display.print("REC");
    }
    else
    {
      display.print("   ");
    }

    display.setCursor(72,57);
    if (calibrating)
    {
      display.print("CAL");
    }
    else
    {
      display.print("   ");
    }
    

    display.setCursor(128-30,57);
    display.print("      ");
    if (UPS >= 10000)
      display.setCursor(98,57);  
    else if (UPS >= 1000)
      display.setCursor(104,57);
    else if (UPS >= 100)
      display.setCursor(110,57);
    else if (UPS >= 10)
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
    display.fillRect(0,0, 128, 57, BLACK);
    display.setTextSize(2);
    display.setCursor(0,0);
    display.print("RPM");
    display.drawLine(0, 19, 126, 19, WHITE);
    display.setTextSize(3);
    display.setCursor(0,26);
    display.print(currentRPM);

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
    display.setCursor(54,46);
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

void ScreenInit(){
  // by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SH1106_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3D (for the 128x64)
  Wire.setClock(400000);
  // init done

  display.setRotation(2);
  display.display();

  SplashScreen();

  display.setCursor(0,0);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.print("Version : "); display.println(VERSION);
  display.display();
  delay(BOOT_SEQUENCE_TIME_INTERVAL);
}

void SplashScreen() {
  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(1750);

  // Clear the buffer.
  display.clearDisplay();
  display.display();
}
