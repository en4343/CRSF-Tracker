# 📡 CRSF-Tracker 
A high-performance, open-source Antenna Tracker designed specifically for modern Long-Range FPV.

This tracker natively sniffs your CRSF/ExpressLRS telemetry directly out of the air using the ESP32's low-latency ESP-NOW protocol. It grabs the GPS coordinates your flight controller is already broadcasting and points your high-gain patch antennas dead-center at your aircraft.

### ⚠️ Critical Prerequisite: Your Transmitter 
Your ExpressLRS transmitter **MUST have a hardware "Backpack" chip installed**. The backpack is a secondary ESP32 or ESP8285 chip inside your radio/module dedicated to communicating with ground station gear.
* **Supported:** Most modern external modules (Radiomaster Ranger, Happymodel ES24TX, BetaFPV Micro) and internal modules (Radiomaster Boxer, TX16S MKII, GX12).
* **Unsupported:** Older or ultra-budget internal modules. Please verify your radio's specifications before building!

---

## ✨ Key Features
* **100% Wireless Data Link:** Reads native CRSF telemetry packets over ESP-NOW. No extra hardware required on the drone/plane.
* **The "Gatekeeper" Safety:** Refuses to calibrate until both the ground station and the aircraft have a rock-solid 8+ satellite 3D lock.
* **Dual Calibration Modes:** Supports an optional BNO085 hardware compass for instant setup, or a "Visual Calibration Mode" for budget builds.
* **Auto-Expiring NVRAM Failsafe:** Survives mid-flight power losses by instantly restoring your calibration math.
* **Live RF Trim (Optional):** Supports a physical potentiometer knob to micro-adjust your pan tracking mid-flight for maximum video clarity.

---

## 🛒 Hardware Shopping List

| Component | Recommendation & Notes |
| :--- | :--- |
| **Microcontroller** | [ESP32 Dev Board](https://www.amazon.com/dp/B08D5ZD528) (Standard 30 or 38-pin module). |
| **Servos** | [2x DS3218 20kg 270° Servos](https://www.amazon.com/dp/B08MTQ1QD1). *Must* be the 270-degree version for proper pan rotation. |
| **Display** | [ELEGOO 0.96" OLED](https://www.amazon.com/dp/B0D2RMQQHR). Must be 4-Pin I2C and use the **SSD1306** chip (avoid SH1106). |
| **Ground GPS** | Any standard UBlox FPV GPS module (e.g., BN-220, Walksnail M10, Matek M10). |
| **Power (BEC)** | [Castle Creations 10A BEC](https://www.readymaderc.com/products/details/castle-creations-bec-switching-regulator-10-amp-peak). **Crucial:** Never power servos from the ESP32's 5V pin. Use a dedicated 5V BEC (3A-5A continuous minimum) wired to the servos, sharing a common ground with the ESP32. |
| **Wiring** | [2x Wago Lever Nuts](https://www.digikey.com/short/q0m7mf3m). Highly recommended for cleanly distributing shared 5V and Ground lines. |
| **Switches** | [2x Momentary Buttons](https://www.amazon.com/dp/B07931588C). |
| **Servo Tester** | [Standard RC Servo Tester](https://www.amazon.com/dp/B08DM2CP3) to find your physical center PWM values. |
| **Capacitor** | [1000µF to 3300µF (10V-25V)](https://www.amazon.com/dp/B07R432MR2). Wire across the 5V and Ground Wago connectors to prevent heavy servos from causing voltage drops/reboots. |
| **Resistors** | [2x 1kΩ - 4.7kΩ Resistors](https://www.digikey.com/short/29dhzrrp). Wire as pull-down resistors (between the servo signal wire and ground) to prevent violent startup twitches. |
| *(Optional)* **Compass** | [Adafruit BNO085 9-DOF IMU](https://www.amazon.com/dp/B0CDGZMLPP). Selected because it does not require the "figure-8" calibration standard drone compasses need. |
| *(Optional)* **Trim Knob** | [10k Linear Potentiometer](https://www.amazon.com/dp/B082FCRQS2). |

🖨️ **3D Model Files:** Print the custom pan/tilt mechanics and electronics housing here: [MakerWorld: CRSF Antenna Tracker] *(Link)*

---

## 🚀 Setup & Configuration

### Step 1: Software & Libraries
Use the **Arduino IDE** (tested with ESP32 Board Package v3.3.x). Install these libraries via the Library Manager:
* `ESP32Servo` by Kevin Harrington
* `SparkFun u-blox GNSS v3` by SparkFun
* `Adafruit BNO08x`, `Adafruit SSD1306`, and `Adafruit GFX Library` by Adafruit

### Step 2: Find your ELRS Binding MAC Address
Because this tracker sniffs raw ESP-NOW packets, it must impersonate your specific transmitter by converting your ELRS Binding Phrase into a 6-digit UID array.
1. Go to the [ExpressLRS UID Generator](https://www.expresslrs.org/hardware/spi-receivers/#binding-phrase-via-cli).
2. Type your secret Binding Phrase into the box.
3. Copy the UID bytes output (e.g., `252, 223, 149, 33, 43, 223`).
4. Open `config.h` and paste those numbers into the `BINDING_MAC` array.

### Step 3: Configure Your Hardware Settings
Open `config.h` and configure your specific build:
1. **Find Servo Centers:** Plug your servos into a tester, physically center your pan/tilt mechanisms, and note the exact microsecond values (e.g., 1480 for Pan, 1550 for Tilt). Enter these into `PAN_CENTER_PWM` and `TILT_HORIZON_PWM`.
2. **Toggles:** Set `#define USE_COMPASS` and `#define USE_TRIM_KNOB` to `false` if you didn't install those physical components.

### Step 4: Radio & Flight Controller Setup
1. **Radio:** Ensure your TX Backpack is flashed with your binding phrase. In your model setup, turn **Telemetry ON**. Run the ELRS Lua Script and ensure the Backpack is enabled.
2. **ArduPilot Users (Crucial Fix):** If you use ArduPilot, you **must disable CRSF Passthrough** (Bit 8 / Value 256 in `RC_OPTIONS`). Passthrough bundles telemetry into a custom format the tracker cannot read. Disabling it restores the standard CRSF GPS packets the tracker needs.

### Step 5: Flash the ESP32
Connect your ESP32 via USB. Select **ESP32 Dev Module** in the boards menu, select your COM port, and upload. 
*(Tip: Block the 5V pin on your USB cable with tape to prevent the board from trying to pull servo power from your PC).*

---

## 🎯 Daily Flight Operations

1. **Boot Sequence:** Power up the ground station. The tracker waits for its local GPS to hit 8 satellites. Power up your aircraft; the tracker LED will blink until it receives the drone's telemetry confirming it also has 8 satellites.
2. **Calibration (Required before every flight):**
   * **With Compass:** Face the tripod toward your flight area. When the screen says "Ready," hold the calibrate button for 1 second.
   * **Without Compass:** Walk your powered aircraft 20-30m directly in front of the tracker. Physically rotate the tripod so the antennas point dead-center at the plane, then hold the calibrate button for 1 second.
3. **Flight:** The servos lock dead-center until the aircraft flies beyond the `MIN_TRACKING_DIST` (default 2 meters), at which point smooth tracking begins.

---

## 💡 Advanced Features & Troubleshooting

<details>
<summary><b>🛡️ The "Power Bump" Failsafe (Memory Restore)</b></summary>

Every time you calibrate, the tracker saves its Home location, servo math, and a live GPS timestamp. If your tracker loses power mid-flight and reboots, it rapidly checks this memory. 

If the tracker is still within 100m of its saved home, it bypasses the normal calibration requirements, restores the math, and immediately resumes tracking. This memory automatically expires after 3 hours. **To manually clear the memory** (e.g., moving to a new spot within 3 hours), hold the calibrate button for 5 seconds until the screen reads "RELEASE TO CLEAR".
</details>

<details>
<summary><b>🌬️ The Cross-Wind Launch Trick</b></summary>

Your pan servo has 135° of travel left and right from its calibration center. If you calibrate while pointing at a cross-wind launch pad 90° away from your main flight area, you will limit your tracking range during the actual flight. 

Instead: Calibrate the tracker pointing toward the *center* of your intended flight airspace. Once calibrated, pick up your plane and walk to your launch pad. The tracker will follow you, and as you launch and turn toward your main area, it will smoothly center itself with maximum travel available.
</details>

<details>
<summary><b>📺 Troubleshooting: OLED Screen is Black</b></summary>

If the code flashed successfully but the screen is dead, your OLED likely uses an alternate I2C address. Open `CRSF_Tracker.ino`, find `if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))`, change `0x3C` to `0x3D`, and re-upload.
</details>

<details>
<summary><b>🎛️ Live RF Trim Knob</b></summary>

If installed, turning the potentiometer sweeps the entire tracker array up to 20° left or right mid-flight. This lets you manually dial in the invisible RF lobe of your patch antennas for the absolute best video feed without having to land and recalibrate.
</details>
