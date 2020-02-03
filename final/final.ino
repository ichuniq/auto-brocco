#include "pitch.h"
#include <Arduino_FreeRTOS.h>
#include <LiquidCrystal_I2C.h>
#include <MFRC522.h>
#include <SPI.h>
#include <Wire.h>
#define VALID_RANGE 25

#define RST_PIN A1 // reset pin
#define SS_PIN 10  // select pin for the chip
#define IRQ_PIN 2  // interrupt pin

// create MFRC522 object
MFRC522 mfrc522(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;

volatile bool bNewInt = false;
byte regVal = 0x7F;
void activateRec(MFRC522 mfrc522);
void clearInt(MFRC522 mfrc522);

// Set pins on I2C chip for LCD connections:
LiquidCrystal_I2C lcd(0x3f, 16, 2);
const int buzzer = 9;
const int IRSensor = 7;
const int trigPin = 4, echoPin = 3; // pin of ultra-sonic sensor
// const int pResistor_1 = A0;

/* variables */
int state = 0;
int control_vol = 0;
long time_taken;
int dist = 0;
int distL, prev_distL = 0;
unsigned long startTime = 0;

unsigned long startControlTime = 0;

byte unlocked = 0;
byte pass_id[4] = {0x8E, 0xBA, 0xD1, 0x20};
byte id_buff[4];

byte show_wrong_id = 0;
byte correct = 1;

/* to catch first interrupt case with empty ID*/
byte first_interrupt = 0;

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

  SPI.begin();
  mfrc522.PCD_Init(); // init MFRC522
  delay(5);
  /* read and printout the MFRC522 version (valid values 0x91 & 0x92)*/
  Serial.print(F("Ver: 0x"));
  byte readReg = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
  Serial.println(readReg, HEX);

  /* setup the IRQ pin*/
  pinMode(IRQ_PIN, INPUT_PULLUP);

  regVal = 0xA0; // rx irq
  mfrc522.PCD_WriteRegister(mfrc522.ComIEnReg, regVal);
  /*Activate the interrupt*/
  attachInterrupt(digitalPinToInterrupt(IRQ_PIN), readCard, FALLING);
  bNewInt = false; // interrupt flag

  pinMode(IRSensor, INPUT); // sensor pin INPUT

  xTaskCreate(taskMain, "Task1", 128, NULL, 1, &TaskHandle_1);
  xTaskCreate(taskTone, "Task2", 128, NULL, 1, &TaskHandle_2);
  xTaskCreate(taskRFID, "Task3", 128, NULL, 1, &TaskHandle_3);
  vTaskStartScheduler();
}

void loop() {}

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
  if (dist > 50) {
    dist = 50;
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
      delay(10);
      if (distL <= 12) {
        Serial.println(F("Volume--"));
      } else if (distL > 12 && distL < 25) {
        Serial.println(F("Volume++"));
      }
    }
  } else {
    control_vol = 0;
  }

  if (digitalRead(IRSensor) == 0)
    if (dist < 5)
      Serial.println("SwitchBack");
    else if (dist > 10 && dist < 20)
      Serial.println("CntrlRight");
    else if (dist >= 20 && dist < 50)
      Serial.println("CntrlLeft");
    else
      Serial.println("Switch");
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
        lcd.print("Locked");
      }
    }
    vTaskDelay(10);
  }
}

void taskTone(void *pvParameters) { // TaskHandle_2
  (void)pvParameters;
  for (;;) {
    if (state == 1) {
      lcd.clear();
      lcd.setCursor(0, 0);
      if (first_interrupt) {
        if (correct) {
          if (unlocked) { // correct card, second time
            unlocked = 0;
            Serial.println(F("Quit"));
            lcd.print("Good-bye!");
            lcd.setCursor(0, 1);
            lcd.print((millis() - startControlTime) / 1000);
            lcd.setCursor(5, 1);
            lcd.print("secs");

            if (dist < 5) {
              march();
            } else {
              tone(buzzer, 500);
              delay(1000);
            }
          } else {
            unlocked = 1;
            lcd.print("Welcome!");
            tone(buzzer, 500);
            delay(1000);
            startControlTime = millis();
          }
        } else {
          unlocked = 0;
          lcd.print("Wrong ID!");
          tone(buzzer, 1000);
          lcd.setCursor(6, 0);
          if (show_wrong_id) {
            // lcd.print(id_buff);
            for (byte i = 0; i < 4; i++) {
              lcd.print((byte)id_buff[i], HEX);
            }
          }
          delay(1000);
        }
      }
      first_interrupt++;
      noTone(buzzer);
      lcd.clear();
      state = 0;
    }
    vTaskDelay(10);
  }
}

void taskRFID(void *pvParameters) { // TaskHandle_3
  (void)pvParameters;
  for (;;) {
    if (state == 2) {
      if (bNewInt) {
        Serial.print(F("Interrupt. "));
        mfrc522.PICC_ReadCardSerial(); // read the tag data
        // Show some details of the PICC (that is: the tag/card)
        dump_byte_array(mfrc522.uid.uidByte, mfrc522.uid.size);

        lcd.setCursor(6, 0);
        if (id_buff[0] != 0) {
          for (byte i = 0; i < 4; i++) {
            lcd.print((byte)id_buff[i], HEX);
          }
        }

        clearInt(mfrc522);
        mfrc522.PICC_HaltA();
        bNewInt = false;

        state = 1;
      }
    }
    activateRec(mfrc522);
    vTaskDelay(10);
  }
}

void dump_byte_array(byte *buffer, byte bufferSize) {
  for (byte i = 0; i < bufferSize; i++) {
    Serial.print(buffer[i] < 0x10 ? " 0" : " ");
    Serial.print(buffer[i], HEX);
    id_buff[i] = buffer[i];
    if (id_buff[i] != pass_id[i]) {
      correct = 0;
      show_wrong_id = 1;
    } else {
      correct = 1;
      show_wrong_id = 0;
    }
  }
}

/* MFRC522 interrupt serving routine */
void readCard() {
  bNewInt = true;
  state = 2;
}

/*
 * The function sending to the MFRC522 the needed commands to activate the
 * reception
 */
void activateRec(MFRC522 mfrc522) {
  mfrc522.PCD_WriteRegister(mfrc522.FIFODataReg, mfrc522.PICC_CMD_REQA);
  mfrc522.PCD_WriteRegister(mfrc522.CommandReg, mfrc522.PCD_Transceive);
  mfrc522.PCD_WriteRegister(mfrc522.BitFramingReg, 0x87);
}

/*
 * The function to clear the pending interrupt bits after interrupt serving
 * routine
 */
void clearInt(MFRC522 mfrc522) {
  mfrc522.PCD_WriteRegister(mfrc522.ComIrqReg, 0x7F);
}

/* music */
void march() {
  tone(buzzer, LA3, Q);
  // delay duration should always be 1 ms more than the note in order to separate them.
  delay(1 + Q); 
  tone(buzzer, LA3, Q);
  delay(1 + Q);
  tone(buzzer, LA3, Q);
  delay(1 + Q);
  tone(buzzer, F3, E + S);
  delay(1 + E + S);
  tone(buzzer, C4, S);
  delay(1 + S);

  tone(buzzer, LA3, Q);
  delay(1 + Q);
  tone(buzzer, F3, E + S);
  delay(1 + E + S);
  tone(buzzer, C4, S);
  delay(1 + S);
  tone(buzzer, LA3, H);
  delay(1 + H);

  tone(buzzer, E4, Q);
  delay(1 + Q);
  tone(buzzer, E4, Q);
  delay(1 + Q);
  tone(buzzer, E4, Q);
  delay(1 + Q);
  tone(buzzer, F4, E + S);
  delay(1 + E + S);
  tone(buzzer, C4, S);
  delay(1 + S);

  tone(buzzer, Ab3, Q);
  delay(1 + Q);
  tone(buzzer, F3, E + S);
  delay(1 + E + S);
  tone(buzzer, C4, S);
  delay(1 + S);
  tone(buzzer, LA3, H);
  delay(1 + H);
}
