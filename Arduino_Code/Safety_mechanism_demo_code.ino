#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <ESP32Servo.h>

#define SENSOR1_XSHUT 5   
#define SENSOR2_XSHUT 15  
#define SERVO_PIN 12      

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
#define SENSOR1_ADDRESS 0x30
#define SENSOR2_ADDRESS 0x31

Servo rejectArm;
const int ARM_REST_ANGLE = 45;    
const int ARM_REJECT_ANGLE = 90;  

int intendedPills = 0;
int countedPills = 0;
bool isSessionActive = false;

bool pillPassedToF1 = false; 

unsigned long lastPrintTime = 0;
const unsigned long PRINT_INTERVAL = 200;

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  rejectArm.setPeriodHertz(50);  
  rejectArm.attach(SERVO_PIN, 500, 2400);  
  rejectArm.write(ARM_REST_ANGLE);  

  Wire.begin();
  pinMode(SENSOR1_XSHUT, OUTPUT); 
  pinMode(SENSOR2_XSHUT, OUTPUT);
  
  digitalWrite(SENSOR1_XSHUT, LOW); 
  digitalWrite(SENSOR2_XSHUT, LOW); 
  delay(10);
  
  digitalWrite(SENSOR1_XSHUT, HIGH); 
  delay(10); 
  if (!lox1.begin(SENSOR1_ADDRESS)) {
    Serial.println("[ERROR] Failed to find VL53L0X #1!");
    while (1);
  }

  digitalWrite(SENSOR2_XSHUT, HIGH); 
  delay(10); 
  if (!lox2.begin(SENSOR2_ADDRESS)) {
    Serial.println("[ERROR] Failed to find VL53L0X #2!");
    while (1);
  }

  Serial.println("\n=== Sequential Pill Counter System Initialized ===");
  Serial.println("Servo holding at rest (45°).");
  Serial.println("Please enter the number of intended pills in the Serial Monitor:");
}

void loop() {
  if (Serial.available() > 0) {
    int input = Serial.parseInt();
    if (input > 0) {
      intendedPills = input;
      countedPills = 0;
      isSessionActive = true;
      pillPassedToF1 = false;
      rejectArm.write(ARM_REST_ANGLE); 
      
      Serial.print("\n>>> TARGET SET: Expecting ");
      Serial.print(intendedPills);
      Serial.println(" pill(s). Start dropping pills now.");
    }
    while(Serial.available() > 0) { Serial.read(); } 
  }

  if (isSessionActive) {
    VL53L0X_RangingMeasurementData_t m1, m2;
    lox1.rangingTest(&m1, false);
    lox2.rangingTest(&m2, false);

    int dist1 = (m1.RangeStatus != 4) ? m1.RangeMilliMeter : 999;
    int dist2 = (m2.RangeStatus != 4) ? m2.RangeMilliMeter : 999;

    if (!pillPassedToF1 && dist1 < 40) {
      pillPassedToF1 = true; 
      Serial.println("\n[LOGIC] Pill passed ToF1 -> 1st condition met.");
    }

    if (pillPassedToF1 && dist2 < 60) {
      countedPills++; 
      pillPassedToF1 = false; 
      
      Serial.print("[LOGIC] Pill passed ToF2 -> Count = ");
      Serial.println(countedPills);

      if (countedPills > intendedPills) {
        rejectArm.write(ARM_REJECT_ANGLE); 
        Serial.print(" [OVERCOUNT] ");
        Serial.print(countedPills);
        Serial.print(" > ");
        Serial.print(intendedPills);
        Serial.println(" (Target Exceeded)! Servo motor rotates to 90°.");
        isSessionActive = false; 
      }
    }

    if (isSessionActive) {
      rejectArm.write(ARM_REST_ANGLE);
    }

    if (millis() - lastPrintTime >= PRINT_INTERVAL) {
      lastPrintTime = millis();
      Serial.print("ToF1: "); Serial.print(dist1);
      Serial.print("mm | ToF2: "); Serial.print(dist2);
      Serial.print("mm | Gate Status: "); Serial.print(pillPassedToF1 ? "ARMED" : "WAITING");
      Serial.print(" | Count: "); Serial.print(countedPills);
      Serial.print("/"); Serial.println(intendedPills);
    }
  }
}