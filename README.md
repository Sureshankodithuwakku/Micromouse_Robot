# Micromouse Robot 🐭🤖

This repository documents the design and implementation of a **Micromouse Robot** built for maze navigation.  
The project integrates **sensor fusion, motor control, and PCB design** into a compact embedded system capable of navigating a maze environment.

---

## 🔧 Hardware Components
- **ESP32 microcontroller** – central processing unit
- **3x Time-of-Flight (ToF) sensors** – distance measurement and obstacle detection
- **MPU6050 IMU** – orientation tracking and stability correction
- **N20 DC motors with encoders** – precise motion control
- **TB6612FNG motor driver** – efficient bidirectional motor control
- **Custom PCB** – designed from schematic, fabricated via JLCPCB

---

## 📐 PCB Design & Fabrication
- Designed the circuit schematic and PCB layout  
- Sent design to **JLCPCB** for fabrication  
- Completed **component placement and soldering**  
- PCB ensured compactness, reduced wiring complexity, and improved signal reliability  

Schematic and layout files are included in this repository.

---

## ⚙️ System Features
- **Closed-loop control** using motor encoders for accurate movement  
- **Sensor integration** for obstacle detection and navigation  
- **Compact PCB design** for robust and maintainable hardware  

---

## 🚀 Future Work
- Implementation of **Flood Fill algorithm** for efficient maze-solving and path optimization  
- Refinement of control algorithms for improved speed and accuracy  

---

## 📂 Repository Contents
- `schematic/` – circuit schematic files  
- `pcb_layout/` – PCB design files  
- `firmware/` – source code for ESP32 (to be updated)  
- `docs/` – project documentation and images  
- `code/` - sensor debugging code 

---

## 📸 Project Highlights
- Schematic design   -Done
- PCB fabrication   -Done
- Component placement and soldering   -Done
- Hardware integration  -Done  
- Next: Algorithm implementation (Flood Fill) 🔍  

---

![pere](https://github.com/user-attachments/assets/01001511-3ed2-4dd8-972c-a5b94606a687)


`Micromouse` `ESP32` `ToF Sensor` `MPU6050` `TB6612FNG` `N20 Motors` `Encoders` `PCB Design` `JLCPCB` `Robotics` `Embedded Systems`

---

## 📜 License
This project is licensed under the MIT License – see the [LICENSE](LICENSE) file for details.
