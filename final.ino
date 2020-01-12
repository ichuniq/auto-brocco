#include <Arduino_FreeRTOS.h>
#include <LiquidCrystal_I2C.h>
#define VALID_RANGE 25

// Set pins on I2C chip for LCD connections:
LiquidCrystal_I2C lcd(0x3f, 16, 2);
const int buzzer = 9;
const int trigPin = 2, echoPin = 3; // pin of ultra-sonic sensor
// const int pResistor_1 = A0;

/* variables */
int state = 0;
int control_vol = 0;
long time_taken;
int dist = 0;
int distL, prev_distL = 0;
unsigned long startTime = 0;
byte unlocked = 1;

TaskHandle_t TaskHandle_1;
TaskHandle_t TaskHandle_2;
TaskHandle_t TaskHandle_3;

void taskSonic(void *pvParameters);
void taskLEDTone(void *pvParameters);
void taskRFID(void *pvParameters);

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
  Serial.println(F("setup"));

  lcd.init();          // initialize LCD
  lcd.backlight();     // open LCD backlight
  lcd.setCursor(0, 0); // setting cursor
  lcd.print("Hello, world!");
  delay(500);
  lcd.clear(); // clear all

  xTaskCreate(taskMain, "Task1", 128, NULL, 1, &TaskHandle_1);
  // xTaskCreate(taskTone, "Task2", 128, NULL, 1, &TaskHandle_2);
  // xTaskCreate(taskRFID, "Task3", 128, NULL, 1, &TaskHandle_3);
  vTaskStartScheduler();
}

/*  Function to calculate distance  */
void calculate_distance(int trigger, int echo) {
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  /* Reads Echo pin, returns sound travel time in ms */
  time_taken = pulseIn(echo, HIGH);
  dist = time_taken * 0.034 / 2;
  if (dist > 60) {
    dist = 60;
  }
}

void control() {
  calculate_distance(trigPin, echoPin);
  // Serial.println(dist);

  //  record  distL every 500 ms
  if ((millis() - startTime > 500)) {
    prev_distL = distL;
    distL = dist;
    // Serial.print(F("distL = "));
    Serial.println(distL);
    startTime = millis();
  }

  if (dist <= VALID_RANGE) {
    if (distL == prev_distL) { // hasn't change for 500ms
      if (control_vol == 0) {
        if (distL >= 10 && distL <= 15) {
          Serial.println(F("enter control mode"));
          control_vol = 1;
        }
      }
    }

    if (control_vol) {
      // Serial.println(F("control mode"));
      delay(50);
      distL = dist;
      if (distL <= 12) {
          Serial.println("Volume--");
          delay(100);
      } else if (distL > 12 && distL < 25) {
          Serial.println("Volume++");
          delay(100);
      }
    }
    //prev_distL = distL;
  } else {
    // Move forward
    if (distL - prev_distL <= -10) {
      Serial.println("Switch");
    } 
    control_vol = 0;
    // prev_distL = distL;
  }
}

void taskMain(void *pvParameters) {
  (void)pvParameters;
  for (;;) {
    if (state == 0) {
      lcd.setCursor(0, 0);
      if (unlocked) {
          lcd.print("Control mode");
          control();
      } else {
          lcd.print("Hello");
          lcd.print("Locked");
      }
    }
      vTaskDelay(10);
  }
}

/*
void taskTone(void *pvParameters) { // TaskHandle_2
  (void)pvParameters;
  for (;;) {
    if (state == 1) {

    }
    vTaskDelay(10);
  }
}

void taskRFID(void *pvParameters) { // TaskHandle_3
  (void)pvParameters;
  for (;;) {
    if (state == 2) {

    }
    vTaskDelay(10);
  }
}
*/

void loop() {}
