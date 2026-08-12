# UAV-Thrust-Stand

A robust and affordable thrust testing platform for UAV (Unmanned Aerial Vehicle) propulsion systems. This project provides the necessary code and documentation to measure, log, and analyze the thrust produced by drone motors and propellers.

##  Features

* **Real-Time Data Logging:** Record thrust, RPM, voltage, and current data in real-time.
* **Hardware Integration:** Compatible with common load cells (e.g., HX711) and microcontrollers (e.g., Arduino/ESP32).
* **Data Visualization:** Easily export data for plotting thrust vs. throttle or efficiency curves.
* **Modular Design:** Easy to adapt for different motor sizes and thrust ranges.

---

##  Hardware Requirements

* **Load Cell:** 5kg to 20kg load cell (depending on motor size)
* **Amplifier:** HX711 Load Cell Amplifier board
* **Microcontroller:** Arduino Uno/Nano or ESP32
* **ESC:** Electronic Speed Controller suited for your motor
* **Power Supply / LiPo Battery:** Matching the motor/ESC voltage specifications
* **Frame:** 3D-printed or aluminum extrusion thrust stand frame

---

##  Software & Dependencies

* **Arduino IDE** (or PlatformIO)
* **Arduino Libraries:**
  * `HX711` by Bogdan Necula
  * `Servo.h` (Standard Arduino library for ESC control)
* **Python 3.x** (optional, for logging and plotting via serial interface)

---

## 🔧 Installation & Calibration

1. **Clone the Repository:**
   ```bash
   git clone [https://github.com/KerryXy804/UAV-Thrust-Stand.git](https://github.com/KerryXy804/UAV-Thrust-Stand.git)
   cd UAV-Thrust-Stand
2. **Wiring Setup:**
   | Component | Microcontroller Pin |
   | :--- | :--- |
   | **HX711 DT** | Pin 2 (example) |
   | **HX711 SCK** | Pin 3 (example) |
   | **ESC Signal** | Pin 9 (PWM) |
   | **GND** | GND |

3. **Sensor Calibration:**
   * Open the calibration sketch in `firmware/` and upload it to your board.
   * Place a known reference weight (e.g., 100g, 500g) on the load cell.
   * Record the calculated `calibration_factor` and save it into your main control code.

---

## 📊 Usage

1. **Upload Main Code:** Upload the main firmware code with your updated calibration factor.
2. **Setup Safety:** Secure the thrust stand, wear eye protection, and clear the testing zone.
3. **Run Test:**
   * Open the Serial Monitor (Baud Rate: `115200`).
   * Increase throttle incrementally to record thrust output at various throttle points.
   * Parse serial data into `.csv` files for further performance plotting.

---

##  Safety Warnings

>  **DANGER:** High-speed rotating propellers pose a serious hazard!

* Always wear protective eye and face shields when running tests.
* Ensure the thrust stand is securely bolted down to a stable workbench.
* Keep all loose clothing, hair, and hands away from the propeller arc.
* Have an immediate emergency cutoff/kill switch ready.

---

##  License

Distributed under the MIT License. See `LICENSE` for more information.
