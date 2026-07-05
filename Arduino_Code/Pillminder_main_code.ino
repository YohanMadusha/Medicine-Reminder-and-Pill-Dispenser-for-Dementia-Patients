#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <ESP32Servo.h>

const char* ssid = "Pillminder_ESP32";
const char* password = "12345678";
WebServer server(80);

bool enableSafetySystem = false; 

unsigned long syncEpoch = 0;
int timezoneOffsetSecs = 0;
unsigned long syncMillis = 0;

struct SlotSchedule {
  int slotNumber;
  int tabletsPerDose;
  String scheduleType; 
  String scheduleTimeStr; 
  unsigned long lastTriggeredEpoch; 
};

SlotSchedule schedules[5];

int slotQueue[10];
int pillQueue[10];
int queueHead = 0;
int queueTail = 0;

void pushToQueue(int slotIdx, int pills) {
  if ((queueTail + 1) % 10 != queueHead) {
    slotQueue[queueTail] = slotIdx;
    pillQueue[queueTail] = pills;
    queueTail = (queueTail + 1) % 10;
  }
}

bool popFromQueue(int &slotIdx, int &pills) {
  if (queueHead == queueTail) return false;
  slotIdx = slotQueue[queueHead];
  pills = pillQueue[queueHead];
  queueHead = (queueHead + 1) % 10;
  return true;
}

enum SystemState {
  IDLE,
  DISPENSING,        
  CLEARING_BELT,
  PAUSED_FOR_REJECT,  
  DISPENSING_WATER
};

SystemState currentState = IDLE;
String dispenserStatusStr = "IDLE"; 

int activeMotorIndex = -1;  
int targetPills = 0;       
int countedPills = 0;      

const long STEPS_PER_PILL = 3072; 

AccelStepper motor1(AccelStepper::DRIVER, 13, 14); 
AccelStepper motor2(AccelStepper::DRIVER, 27, 26); 
AccelStepper motor3(AccelStepper::DRIVER, 25, 33); 
AccelStepper motor4(AccelStepper::DRIVER, 32, 4);  
AccelStepper motor5(AccelStepper::DRIVER, 16, 17); 
AccelStepper* posMotors[] = {&motor1, &motor2, &motor3, &motor4, &motor5};

#define CONVEYOR_STEP_PIN 19                       
#define CONVEYOR_DIR_PIN 18                        
AccelStepper conveyor(AccelStepper::DRIVER, CONVEYOR_STEP_PIN, CONVEYOR_DIR_PIN);

const long CONVEYOR_STEPS_PER_REV = 800; 
const long CONVEYOR_STEPS_TO_CLEAR = 5 * CONVEYOR_STEPS_PER_REV; 

Servo rejectArm;
#define SERVO_PIN 12                               
const int ARM_REST_ANGLE = 90;     
const int ARM_REJECT_ANGLE = 0;    
bool isRejecting = false;
unsigned long rejectTriggerTime = 0;
const unsigned long SENSOR_TO_ARM_DELAY = 400; 
const unsigned long ARM_SWEEP_DURATION = 1000; 

#define PUMP_PIN 23                                
#define TRIG_PIN 2                                 
#define ECHO_PIN 34 

#define SENSOR1_XSHUT 5                            
#define SENSOR2_XSHUT 15                           
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
#define SENSOR1_ADDRESS 0x30
#define SENSOR2_ADDRESS 0x31

#define IR_CUP_PIN 35 
bool waitingForPickup = false; 
bool medicineTaken = false;    
bool manualIrBeamBroken = false; 

unsigned long lastWaterMonitorTime = 0;
const int WATER_MONITOR_INTERVAL = 2000; 
int calculatedWaterPercent = 100;
float filteredDistance = 100.0; 
const float EMA_ALPHA = 0.3;    
bool isFirstWaterReading = true; 
const unsigned long PUMP_RUN_TIME_MS = 10000; 
unsigned long pumpStartTime = 0;

unsigned long dispenseStartTime = 0;
const unsigned long MAX_DISPENSE_TIME_MS = 60000; 
bool dispenseTimedOut = false;

unsigned long lastToFCheck = 0;
const int ToF_INTERVAL = 20; 
int baselineDistance = 100; 
bool stage1Detected = false;
unsigned long stage1Time = 0;
const int CONFIRMATION_TIMEOUT = 300; 

unsigned long monitorStartTime = 0;
unsigned long monitorWindowDuration = 300000; 

unsigned long getLocalEpoch() {
  if (syncEpoch == 0) return 0;
  return syncEpoch + timezoneOffsetSecs + ((millis() - syncMillis) / 1000);
}

void setup() {
  Serial.begin(115200);

  WiFi.softAP(ssid, password);
  server.on("/sensor_data", HTTP_GET, handleSensorData);
  server.on("/sync_schedule", HTTP_POST, handleSyncSchedule);
  server.on("/trigger_demo_dose", HTTP_POST, handleTriggerDose);
  server.on("/simulate_beam_break", HTTP_POST, handleSimulateBeam);
  server.begin();

  pinMode(PUMP_PIN, INPUT); 
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(IR_CUP_PIN, INPUT);

  for(int i = 0; i < 5; i++) {
    posMotors[i]->setMaxSpeed(250.0);      
    posMotors[i]->setAcceleration(50.0);
    posMotors[i]->setMinPulseWidth(20); 
    schedules[i].slotNumber = i + 1;
    schedules[i].tabletsPerDose = 0;
    schedules[i].scheduleType = "FIXED_LABEL";
    schedules[i].scheduleTimeStr = "00:00";
    schedules[i].lastTriggeredEpoch = 0;
  }
  conveyor.setMaxSpeed(4000.0); 
  conveyor.setAcceleration(2000.0); 

  rejectArm.setPeriodHertz(50); 
  rejectArm.attach(SERVO_PIN, 500, 2400); 
  rejectArm.write(ARM_REST_ANGLE); 

  pinMode(SENSOR1_XSHUT, OUTPUT); pinMode(SENSOR2_XSHUT, OUTPUT);
  digitalWrite(SENSOR1_XSHUT, LOW); digitalWrite(SENSOR2_XSHUT, LOW); delay(10);
  digitalWrite(SENSOR1_XSHUT, HIGH); delay(10); lox1.begin(SENSOR1_ADDRESS);
  digitalWrite(SENSOR2_XSHUT, HIGH); delay(10); lox2.begin(SENSOR2_ADDRESS);

  Serial.println("System Ready.");
}

void loop() {
  server.handleClient();
  handleRejectServo(); 
  monitorCupPickup(); 
  

  switch (currentState) {
    case IDLE:
      if (millis() - lastWaterMonitorTime >= WATER_MONITOR_INTERVAL && !waitingForPickup) {
        lastWaterMonitorTime = millis();
        
        int dist = getWaterDistanceMM(); 
        
        
        if (dist != 999) { 
          if (isFirstWaterReading) {
            filteredDistance = dist; 
            isFirstWaterReading = false;
          } else {
            
            filteredDistance = (EMA_ALPHA * dist) + ((1.0 - EMA_ALPHA) * filteredDistance);
          }
          
          
          calculatedWaterPercent = constrain(map((int)filteredDistance, 180, 40, 0, 100), 0, 100);
          
          Serial.print("[WATER] Raw: "); Serial.print(dist); 
          Serial.print("mm | Filtered: "); Serial.print(filteredDistance); 
          Serial.print("mm | "); Serial.print(calculatedWaterPercent); Serial.println("%");
        } else {
          Serial.println("[WATER] 999 Error Spike Ignored");
        }

        
        VL53L0X_RangingMeasurementData_t m1, m2;
        lox1.rangingTest(&m1, false);
        lox2.rangingTest(&m2, false);
        Serial.print("[ToF1] "); Serial.print(m1.RangeMilliMeter);
        Serial.print("mm | [ToF2] "); Serial.println(m2.RangeMilliMeter);
      }
      if (queueHead != queueTail) {
        int sIdx, pCount;
        if (popFromQueue(sIdx, pCount)) {
          activeMotorIndex = sIdx;
          targetPills = pCount;
          posMotors[activeMotorIndex]->setCurrentPosition(0);
          posMotors[activeMotorIndex]->moveTo(targetPills * STEPS_PER_PILL);
          currentState = DISPENSING;
          dispenserStatusStr = "DISPENSING";
          dispenseStartTime = millis();
          dispenseTimedOut = false;
        }
      }
      break;

    case DISPENSING:
      posMotors[activeMotorIndex]->run();
      if (posMotors[activeMotorIndex]->distanceToGo() == 0 ||
          (millis() - dispenseStartTime > MAX_DISPENSE_TIME_MS)) {
        if (millis() - dispenseStartTime > MAX_DISPENSE_TIME_MS) {
          dispenseTimedOut = true;
          Serial.println("[WARN] Motor timeout! Possible jam on slot " + String(activeMotorIndex + 1));
          posMotors[activeMotorIndex]->stop();
        }
        conveyor.setCurrentPosition(0);
        conveyor.moveTo(CONVEYOR_STEPS_TO_CLEAR);
        currentState = CLEARING_BELT;
        dispenserStatusStr = "CLEARING_BELT";
      }
      break;

    case CLEARING_BELT:
      conveyor.run(); 
      if (conveyor.distanceToGo() == 0) {
        if (queueHead != queueTail) {
          int sIdx, pCount;
          popFromQueue(sIdx, pCount);
          activeMotorIndex = sIdx;
          targetPills = pCount;
          posMotors[activeMotorIndex]->setCurrentPosition(0);
          posMotors[activeMotorIndex]->moveTo(targetPills * STEPS_PER_PILL);
          currentState = DISPENSING;
          dispenserStatusStr = "DISPENSING";
          dispenseStartTime = millis();
          dispenseTimedOut = false;
        } else {
          pinMode(PUMP_PIN, OUTPUT); digitalWrite(PUMP_PIN, LOW); 
          pumpStartTime = millis(); 
          currentState = DISPENSING_WATER;
          dispenserStatusStr = "DISPENSING_WATER";
        }
      }
      break;

    case DISPENSING_WATER:
      if (millis() - pumpStartTime >= PUMP_RUN_TIME_MS) {
        pinMode(PUMP_PIN, INPUT);
        currentState = IDLE;
        
        waitingForPickup = true;
        monitorStartTime = millis();
        dispenserStatusStr = "MONITORING";
        manualIrBeamBroken = false;  
        Serial.println("[STATE] Pump done. Monitoring for cup pickup...");
      }
      break;
  }
}

void handleSensorData() {
  StaticJsonDocument<200> doc;
  doc["weightGrams"] = 0.0;
  doc["waterLevelPercent"] = calculatedWaterPercent;
  
  bool realIr = (digitalRead(IR_CUP_PIN) == HIGH);
  doc["irBeamBroken"] = realIr || manualIrBeamBroken;
  doc["doseStatus"] = dispenserStatusStr;

  unsigned long currentLocalEpoch = getLocalEpoch();
  int currentHour = (currentLocalEpoch % 86400) / 3600;
  int currentMin = ((currentLocalEpoch % 86400) % 3600) / 60;
  char timeBuf[10];
  sprintf(timeBuf, "%02d:%02d", currentHour, currentMin);
  doc["espTime"] = timeBuf;

  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleTriggerDose() {
  
  if (currentState != IDLE) {
    server.send(409, "application/json", "{\"status\":\"busy\",\"message\":\"System is currently dispensing. Wait for completion.\"}");
    return;
  }

  if (server.hasArg("plain") == false) {
    server.send(400, "text/plain", "Body not received");
    return;
  }
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, server.arg("plain"));
  if (error) {
    server.send(400, "text/plain", "Invalid JSON");
    return;
  }
  
  queueHead = 0; 
  queueTail = 0;

  JsonArray slotsArray = doc["slots"].as<JsonArray>();
  if (doc.containsKey("monitorWindowMs")) {
    monitorWindowDuration = doc["monitorWindowMs"].as<unsigned long>();
  }

  for (JsonObject slotObj : slotsArray) {
    int slotIndex = slotObj["slotNumber"].as<int>() - 1;
    if (slotIndex >= 0 && slotIndex < 5) {
      int pills = slotObj["pills"].as<int>();
      if (pills <= 0) pills = 1; 
      pushToQueue(slotIndex, pills);
    }
  }
  
  server.send(200, "application/json", "{\"status\":\"success\"}");
}

void handleSyncSchedule() {
  if (server.hasArg("plain") == false) {
    server.send(400, "text/plain", "Body not received");
    return;
  }
  StaticJsonDocument<1024> doc;
  DeserializationError error = deserializeJson(doc, server.arg("plain"));
  if (error) {
    server.send(400, "text/plain", "Invalid JSON");
    return;
  }
  
  syncEpoch = doc["timestamp"].as<unsigned long>();
  timezoneOffsetSecs = doc["timezoneOffsetSeconds"].as<int>();
  syncMillis = millis();

  JsonArray slotsArray = doc["slots"].as<JsonArray>();
  for (JsonObject slotObj : slotsArray) {
    int slotNum = slotObj["slotNumber"].as<int>();
    if (slotNum >= 1 && slotNum <= 5) {
      int idx = slotNum - 1;
      schedules[idx].tabletsPerDose = slotObj["tabletsPerDose"].as<int>();
      schedules[idx].scheduleType = slotObj["scheduleType"].as<String>();
      schedules[idx].scheduleTimeStr = slotObj["scheduleTimeStr"].as<String>();
    }
  }
  
  server.send(200, "application/json", "{\"status\":\"synced\"}");
}

void handleSimulateBeam() {
  manualIrBeamBroken = true;
  if (waitingForPickup) {
    medicineTaken = true;
    waitingForPickup = false;
    dispenserStatusStr = "TAKEN";
  }
  server.send(200, "application/json", "{\"status\":\"beam_broken\"}");
}

void checkAutonomousSchedule() {
  if (syncEpoch == 0 || currentState != IDLE) return;

  unsigned long currentLocalEpoch = getLocalEpoch();
  int currentHour = (currentLocalEpoch % 86400) / 3600;
  int currentMin = ((currentLocalEpoch % 86400) % 3600) / 60;

  for (int i = 0; i < 5; i++) {
    if (schedules[i].tabletsPerDose <= 0) continue;

    if (currentLocalEpoch - schedules[i].lastTriggeredEpoch < 60) continue;

    bool shouldTrigger = false;

    if (schedules[i].scheduleType == "FIXED_LABEL") {
      int cIndex = schedules[i].scheduleTimeStr.indexOf(':');
      if (cIndex > 0) {
        int tHour = schedules[i].scheduleTimeStr.substring(0, cIndex).toInt();
        int tMin = schedules[i].scheduleTimeStr.substring(cIndex + 1).toInt();
        
        int targetMinOfDay = tHour * 60 + tMin;
        int currentMinOfDay = currentHour * 60 + currentMin;
        int diff = currentMinOfDay - targetMinOfDay;
        if (diff >= 0 && diff <= 2) {
          shouldTrigger = true;
        }
      }
    } else if (schedules[i].scheduleType == "INTERVAL") {
      int intervalHours = schedules[i].scheduleTimeStr.toInt();
      if (intervalHours > 0) {
        
        if (currentHour % intervalHours == 0 && currentMin <= 2) {
          shouldTrigger = true;
        }
      }
    }

    if (shouldTrigger) {
      schedules[i].lastTriggeredEpoch = currentLocalEpoch;
      pushToQueue(i, schedules[i].tabletsPerDose);
    }
  }
}

void monitorCupPickup() {
  if (waitingForPickup) {
    bool isCupMissing = (digitalRead(IR_CUP_PIN) == HIGH) || manualIrBeamBroken;

    if (isCupMissing) {
      medicineTaken = true;        
      waitingForPickup = false;   
      dispenserStatusStr = "TAKEN";
      Serial.println(">>> SENSOR TRIGGERED: Medicine TAKEN. <<<");
    } else {
      if (millis() - monitorStartTime > monitorWindowDuration) {
        waitingForPickup = false;
        dispenserStatusStr = "NOT_TAKEN";
        Serial.println(">>> TIMEOUT: Medicine MISSED. <<<");
      }
    }
  } else if (dispenserStatusStr == "TAKEN" || dispenserStatusStr == "NOT_TAKEN") {
    if (millis() - monitorStartTime > (monitorWindowDuration + 5000)) {
       dispenserStatusStr = "IDLE";
       manualIrBeamBroken = false;
    }
  }
}

int getWaterDistanceMM() {
  int readings[3];
  int validCount = 0;

  for (int i = 0; i < 3; i++) {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH, 30000); 
    int d = (duration * 0.343) / 2;
    
    if (duration > 0 && d > 0 && d < 400) { 
      readings[validCount] = d;
      validCount++;
    }
    delay(10); 
  }

  
  if (validCount == 0) return 999; 

  
  if (validCount >= 2) {
    if (readings[0] > readings[1]) { int temp = readings[0]; readings[0] = readings[1]; readings[1] = temp; }
    if (validCount == 3) {
      if (readings[1] > readings[2]) { int temp = readings[1]; readings[1] = readings[2]; readings[2] = temp; }
      if (readings[0] > readings[1]) { int temp = readings[0]; readings[0] = readings[1]; readings[1] = temp; }
      return readings[1]; 
    }
  }
  return readings[0]; 
}

void triggerRejectSequence() {
  isRejecting = true;
  rejectTriggerTime = millis();
}

void handleRejectServo() {
  if (!isRejecting) return;
  unsigned long timeSinceTrigger = millis() - rejectTriggerTime;
  if (timeSinceTrigger >= SENSOR_TO_ARM_DELAY && timeSinceTrigger < (SENSOR_TO_ARM_DELAY + ARM_SWEEP_DURATION)) {
    rejectArm.write(ARM_REJECT_ANGLE); 
  } else if (timeSinceTrigger >= (SENSOR_TO_ARM_DELAY + ARM_SWEEP_DURATION)) {
    rejectArm.write(ARM_REST_ANGLE);
    isRejecting = false;
  }
}

bool checkPillDrop() {
  VL53L0X_RangingMeasurementData_t measure;
  if (!stage1Detected) {
    lox1.rangingTest(&measure, false);
    if (measure.RangeStatus != 4 && measure.RangeMilliMeter < (baselineDistance - 20)) {
      stage1Detected = true;       
      stage1Time = millis();       
    }
  } else {
    lox2.rangingTest(&measure, false);
    if (measure.RangeStatus != 4 && measure.RangeMilliMeter < (baselineDistance - 20)) {
      stage1Detected = false; 
      return true; 
    }
    if (millis() - stage1Time > CONFIRMATION_TIMEOUT) {
      stage1Detected = false; 
    }
  }
  return false;
}