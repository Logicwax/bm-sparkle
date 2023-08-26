#include <FastLED.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define LION_BIKE 1
//#define KIKI_BIKE 1

#ifdef LION_BIKE
#define FRAME_NUM_LEDS   50
#define TAIL_NUM_LEDS    32
#endif

#ifdef KIKI_BIKE
#define FRAME_NUM_LEDS   58
#define TAIL_NUM_LEDS    33
#endif

#define NUM_LEDS FRAME_NUM_LEDS + TAIL_NUM_LEDS
#define CHASE_DELAY_TIME                55
#define CHASE_GROUP_SIZE                 3
#define SPARKLE_LED_ON_TIME             40
#define MOTION_ACCEL_SENSE_THRESHOLD     2
#define MOTION_GYRO_SENSE_THRESHOLD      1
#define SPARKLECOUNT                    10
#define CHASE_FADE_RATE                100
#define SPARKLE_FADE_RATE              100
#define IDLE_TIMEOUT                   100

#define BUTTON_INPUT      2
#define BUTTON_GND        9
#define FRAME_DATA_PIN   12
#define FRAME_CLOCK_PIN  10
#define TAIL_DATA_PIN    11
#define TAIL_CLOCK_PIN   13

CRGB leds[FRAME_NUM_LEDS];
CRGB tail_leds[FRAME_NUM_LEDS];

Adafruit_BNO055 bno = Adafruit_BNO055(55);
int buttonState = 0;  // variable for reading the pushbutton status.
int state = 0;;

int idleCounter = 0;
bool idleState;

struct xyzvector {
  double x;
  double y;
  double z;
};

xyzvector prevAccelValues, prevGyroValues;

void ledReMap(int index, uint32_t color) {
  if (index > TAIL_NUM_LEDS) {
    // This is on main frame.
    leds[index - TAIL_NUM_LEDS] = color;
  } else {
    // This is on the tail, which is beginning of array, and reverse order.
    tail_leds[TAIL_NUM_LEDS - index] = color;
  }
}

void setup() {
  Wire.begin();          // join i2c bus (address optional for master).
  Serial.begin(115200);  // start serial communication at 115200bps.
  Serial.println("hello!");
  LEDS.addLeds<WS2801, FRAME_DATA_PIN, FRAME_CLOCK_PIN, RGB>(leds, FRAME_NUM_LEDS);
  LEDS.addLeds<WS2801, TAIL_DATA_PIN, TAIL_CLOCK_PIN, RGB>(tail_leds, TAIL_NUM_LEDS);

  pinMode(BUTTON_GND, OUTPUT);    //button source.
  digitalWrite(BUTTON_GND, LOW);  //button source (turn into a ground).
  pinMode(BUTTON_INPUT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_INPUT), changeState, RISING);

  if(!bno.begin()) {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);
    bno.setExtCrystalUse(true);
}

void loop() {
  if (isMotion(getAccel(), getGyro())) {
    idleCounter = 0;
  } else {
    if (idleCounter < IDLE_TIMEOUT) {
      idleCounter++;
    }
  }
  if (state == 2) {
    chaseMode(colorWheel360(getCompassHeading()));
  } else if (idleCounter == IDLE_TIMEOUT) {
    chaseMode(colorWheel360(getCompassHeading()));
  } else {
    sparkle(getCompassHeading());
  }
}  //end main loop.

void changeState() {
  static unsigned long last_interrupt_time = 0;
  unsigned long interrupt_time = millis();
    // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (interrupt_time - last_interrupt_time > 200) {
    if (state == 2)
      state = 0;
    else
      state++;
  }
  last_interrupt_time = interrupt_time;
  Serial.println((String)"State: " + state);
}

void chaseMode(uint32_t color) {
  // Head towards handle bars.
  for (int ledIndex = 0; ledIndex < NUM_LEDS; ledIndex += CHASE_GROUP_SIZE) {
    for (int groupIndex = ledIndex; (groupIndex < ledIndex + CHASE_GROUP_SIZE) && (groupIndex < NUM_LEDS); groupIndex++) {
      //leds[groupIndex] = color;
      ledReMap(groupIndex, color);
    }
    LEDS.show();
    delay(CHASE_DELAY_TIME);
    fadeToBlackBy(leds, FRAME_NUM_LEDS, CHASE_FADE_RATE);
    fadeToBlackBy(tail_leds, TAIL_NUM_LEDS, CHASE_FADE_RATE);
    // Kick out if motion detected.
    if (isMotion(getAccel(), getGyro()) && state <= 1) {
      fadeToBlackBy(leds, FRAME_NUM_LEDS, 255);
      fadeToBlackBy(tail_leds, TAIL_NUM_LEDS, 255);
      idleCounter = 0;
      return;
    }
  }
  // Return back from handle bars.
  for (int ledIndex = NUM_LEDS - 1; ledIndex >= 0; ledIndex -= CHASE_GROUP_SIZE) {
    for (int groupIndex = ledIndex; (groupIndex > ledIndex - CHASE_GROUP_SIZE) && (groupIndex >= 0); groupIndex--) {
      //leds[groupIndex] = color;
      ledReMap(groupIndex, color);
    }
    LEDS.show();
    delay(CHASE_DELAY_TIME);
    fadeToBlackBy(leds, FRAME_NUM_LEDS, CHASE_FADE_RATE);
    fadeToBlackBy(tail_leds, TAIL_NUM_LEDS, CHASE_FADE_RATE);
    // Kick out if motion detected.
    if (isMotion(getAccel(), getGyro()) && state == 0) {
      fadeToBlackBy(leds, FRAME_NUM_LEDS, 255);
      fadeToBlackBy(tail_leds, TAIL_NUM_LEDS, 255);
      idleCounter = 0;
      return;
    }
  }
}

struct xyzvector getAccel() {
  xyzvector accel;
  imu::Vector<3> accelVector = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  accel.x = abs(accelVector.x());
  accel.y = abs(accelVector.y());
  accel.z = abs(accelVector.z());
  return accel;
}

struct xyzvector getGyro() {
  xyzvector gyro;
  imu::Vector<3> gyroVector = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  gyro.x = abs(gyroVector.x());
  gyro.y = abs(gyroVector.y());
  gyro.z = abs(gyroVector.z());
  return gyro;
}

int getCompassHeading() {
  // Get a new sensor event.
  sensors_event_t event;
  bno.getEvent(&event);
  return event.orientation.x;
}

void sparkle(int compassHeading) {
  for (int i=0; i < SPARKLECOUNT; i++) {
    leds[random(FRAME_NUM_LEDS)] = colorWheel360(compassHeading);
    tail_leds[random(TAIL_NUM_LEDS)] = colorWheel360(compassHeading);
  }
  LEDS.show();
  delay(SPARKLE_LED_ON_TIME);
  if (state == 1) {
    fadeToBlackBy(leds, FRAME_NUM_LEDS, 150);
    fadeToBlackBy(tail_leds, TAIL_NUM_LEDS, 150);
  } else {
    fadeToBlackBy(leds, FRAME_NUM_LEDS, SPARKLE_FADE_RATE);
    fadeToBlackBy(tail_leds, TAIL_NUM_LEDS, SPARKLE_FADE_RATE);
  }
  LEDS.show();
}

bool isMotion(xyzvector accelValues, xyzvector gyroValues) {
  bool motion = false;
  if (abs(accelValues.x - prevAccelValues.x) > MOTION_ACCEL_SENSE_THRESHOLD ||
    abs(accelValues.y - prevAccelValues.y) > MOTION_ACCEL_SENSE_THRESHOLD ||
    abs(accelValues.z - prevAccelValues.z) > MOTION_ACCEL_SENSE_THRESHOLD ||
    abs(gyroValues.x - prevGyroValues.x) > MOTION_GYRO_SENSE_THRESHOLD ||
    abs(gyroValues.y - prevGyroValues.y) > MOTION_GYRO_SENSE_THRESHOLD ||
    abs(gyroValues.z - prevGyroValues.z) > MOTION_GYRO_SENSE_THRESHOLD
  ) {
    //Serial.println(abs(accelValues.x - prevAccelValues.x));
    //Serial.println(abs(accelValues.y - prevAccelValues.y));
    //Serial.println(abs(accelValues.z - prevAccelValues.z));
    //Serial.println(abs(gyroValues.x - prevGyroValues.x));
    motion = true;
  } else {
    motion = false;
  }
  prevAccelValues.x = accelValues.x;
  prevAccelValues.y = accelValues.y;
  prevAccelValues.z = accelValues.z;
  prevGyroValues.x = gyroValues.x;
  prevGyroValues.y = gyroValues.y;
  prevGyroValues.z = gyroValues.z;
  return motion;
}

// Create a 24 bit color value from R,G,B
uint32_t Color(byte r, byte g, byte b) {
  uint32_t fullColor;
  fullColor = r;
  fullColor <<= 8;
  fullColor |= g;
  fullColor <<= 8;
  fullColor |= b;
  return fullColor;
}

uint32_t reScaler(int unscaledNum, int minOUT, int maxOUT, int minIN, int maxIN) {
  return (maxOUT - minOUT) * (unscaledNum - minIN) / (maxIN - minIN) + minOUT;
}

uint32_t colorWheel360(int WheelPos) {
  int colorVal = reScaler((WheelPos % 90), 0, 255, 0, 90);
  if (WheelPos < 90)
    return Color(255 - colorVal, colorVal, 0);  //fading out red, fading in green
  else if (WheelPos >= 90 && WheelPos < 180)
    return Color(0, 255 - colorVal, colorVal);  //fading in blue, fading out green
    else if (WheelPos >= 180 && WheelPos < 270)
      return Color(colorVal, colorVal, 255);  //fading in white, fading out blue
      else if (WheelPos >= 270)
        return Color(255, 255 - colorVal, 255 - colorVal);  //fading in red, fading out white
}
