# Medicine-Reminder-and-Pill-Dispenser-for-Dementia-Patients
A smart mechatronics system for automated medicine dispensing and caregiver monitoring using ESP32.
# 🏥 Medicine Reminder and Pill Dispenser for Dementia Patients

![Project Banner](<img width="1052" height="1125" alt="image" src="https://github.com/user-attachments/assets/c73210a7-c637-46c1-84fc-90cf75669493" />
)

## 📖 Overview

This project was developed as part of **ME2041 – Fundamentals of Mechatronics** at the **University of Moratuwa**.

The objective of this project is to assist dementia patients by automatically dispensing medicines according to a predefined schedule while allowing caregivers to monitor and manage the system remotely through a mobile application.

---

## 🎯 Features

- Automated medicine dispensing
- Five medicine compartments
- Conveyor belt transportation
- Automatic water dispensing
- Mobile application for caregivers
- Wi-Fi communication using ESP32
- Medicine pickup detection
- Water level monitoring
- Pill detection using ToF sensors

---

## ⚙ Hardware Components

- ESP32
- NEMA 17 Stepper Motor
- TB6600 Stepper Motor Driver
- 28BYJ-48 Stepper Motors
- A4988 Stepper Motor Driver
- Servo Motor
- VL53L0X Time-of-Flight Sensors
- IR Sensor
- Ultrasonic Sensor
- Water Pump
- 12V Power Supply
- Buck Converter
- Custom 3D Printed Parts

---

## 💻 Software

- Arduino IDE
- Android Studio
- SolidWorks

---

## 🏗 System Architecture

```
Android App
      │
      ▼
 Wi-Fi Communication
      │
      ▼
     ESP32
      │
 ┌──────────────┐
 │              │
 ▼              ▼
Motors      Sensors
 │              │
 ▼              ▼
Dispensing  Monitoring
```

---

## 📷 Prototype

<img width="234" height="414" alt="Picture1" src="https://github.com/user-attachments/assets/7c6c7078-60f4-4446-b3d2-effc4fbea974" />


---

## 📱 Mobile Application
<img width="759" height="959" alt="image" src="https://github.com/user-attachments/assets/fa160b93-1c29-44ec-a24c-7eb9aaea8972" />



---

## 👨‍💻 My Contribution

My primary contribution to this project was the design and implementation of the conveyor belt mechanism.

Responsibilities included:

- Selecting and integrating the NEMA 17 stepper motor.
- Driving the conveyor using the TB6600 stepper motor driver.
- Developing the conveyor control logic using the AccelStepper library.
- Testing and optimizing smooth pill transportation.

---

## 🚀 Engineering Challenges

One of the main challenges was the limited number of GPIO pins available on the ESP32.

To overcome this, we integrated the A4988 stepper motor driver, allowing the ESP32 to control stepper motors using only STEP and DIR signals instead of directly controlling all motor phases. This reduced GPIO usage while maintaining accurate motor control.

---

## 👥 Team

Developed by:

- Ishwar Kasthuriarachchi
- Pahan Pinibindu
- Anjana Chathumini
- S Naveendran
- Yohan Madusha

---

## 📄 License

MIT License
