#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_TCS34725.h>

// Set the LCD address to 0x27 for a 16 chars and 2-line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

#define in1 9
#define in2 7
#define in3 8
#define in4 10
#define enA 6
#define enB 5

const int ir_l = A0;
const int ir_r = A1;

int m1_speed = 180;
int m2_speed = 180;

double Kp = 2.0;  // Proportional gain
double Ki = 0.0;  // Integral gain
double Kd = 0.0;  // Derivative gain
double error = 0;
double lastError = 0;
double integral = 0;
double derivative = 0;

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  TCCR0B = TCCR0B & B11111000 | B00000010;
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(ir_l, INPUT);
  pinMode(ir_r, INPUT);

  // Initialize LCD
  lcd.init(); // Use .init() instead of .begin() to avoid errors
  lcd.backlight();
  lcd.clear();
  lcd.print("Line Follower");

  // Initialize color sensor
  if (tcs.begin()) {
    lcd.clear();
    lcd.print("Color Sensor");
  } else {
    lcd.clear();
    lcd.print("Color Sensor Error");
    while (1);
  }
}

void loop() {
  const int val1 = digitalRead(ir_l);
  const int val2 = digitalRead(ir_r);

  error = val1 - val2;
  integral += error;
  derivative = error - lastError;

  double turn = Kp * error + Ki * integral + Kd * derivative;

  // Adjust motor speeds based on the turn value
  int leftSpeed = m1_speed - turn;
  int rightSpeed = m2_speed + turn;

  // Ensure motor speeds are within the valid range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  // Update motor speeds
  analogWrite(enA, leftSpeed);
  analogWrite(enB, rightSpeed);

  lastError = error;

  // Read and display color
  readColor();
}

void readColor() {
  uint16_t clear, red, green, blue;
  tcs.getRawData(&red, &green, &blue, &clear);
  uint16_t sum = clear;
  float r = red / sum;
  float g = green / sum;
  float b = blue / sum;

  if (r > 0.5 && g < 0.5 && b < 0.5) {
    lcd.clear();
    lcd.print("Color: Red");
  } else if (r < 0.5 && g > 0.5 && b < 0.5) {
    lcd.clear();
    lcd.print("Color: Green");
  } else if (r < 0.5 && g < 0.5 && b > 0.5) {
    lcd.clear();
    lcd.print("Color: Blue");
  } else {
    lcd.clear();
    lcd.print("Color: Unknown");
  }
}

void stop() {
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}
