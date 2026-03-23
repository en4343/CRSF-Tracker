📡 CRSF-Tracker
A high-performance, open-source Antenna Tracker designed specifically for modern Long-Range FPV.

Instead of relying on heavy onboard encoders or complex Mavlink wiring, this tracker natively sniffs your CRSF/ExpressLRS telemetry directly out of the air using the ESP32's low-latency ESP-NOW protocol. It grabs the GPS coordinates your flight controller is already broadcasting and points your high-gain patch antennas dead-center at your aircraft.

⚠️ Critical Prerequisite: Your Transmitter (Radio)
Your ExpressLRS transmitter MUST have a hardware "Backpack" chip installed. The backpack is a secondary ESP32 or ESP8285 chip inside your radio/module dedicated to communicating with ground station gear.

Most modern external modules (Radiomaster Ranger, Happymodel ES24TX, BetaFPV Micro) and modern internal modules (Radiomaster Boxer, TX16S MKII, GX12) have this built-in.

Some older or ultra-budget internal modules do not have a backpack chip. Please verify your radio's specifications before building!

✨ Key Features
100% Wireless Data Link: Reads native CRSF telemetry packets over ESP-NOW. No extra hardware required on the drone/plane!

The "Gatekeeper" Safety: Refuses to calibrate until both the ground station and the aircraft have a rock-solid 8+ satellite 3D lock.

Dual Calibration Modes: Supports an optional BNO085 hardware compass for instant setup, or a "Visual Calibration Mode" for budget builds.

Auto-Expiring NVRAM Failsafe: Survives mid-flight power losses by instantly restoring your calibration math (expires automatically after 3 hours).

Live RF Trim (Optional): Supports a physical potentiometer knob to micro-adjust your pan tracking mid-flight for maximum video clarity.

🛒 Hardware Shopping List
To build this tracker, you will need the following components:

Microcontroller: ESP32 Dev Board (Standard 30 or 38-pin module).
https://www.amazon.com/dp/B08D5ZD528

Servos: 2PCS DS3218 20kg 270-Degree Servos (Full metal gear, high torque, waterproof. You absolutely need the 270° version for proper pan rotation).
https://www.amazon.com/dp/B08MTQ1QD1

Display: ELEGOO 0.96 Inch OLED Display (Must be 4-Pin I2C, and must use the SSD1306 chip. Do not buy the 1.3" SH1106 versions!).
https://www.amazon.com/dp/B0D2RMQQHR

Ground Station GPS: Any standard FPV GPS module with a UBlox chip (e.g., BN-220, Walksnail M10, Matek M10).

Power Supply (BEC): Castle Creations 10A BEC. A high-quality 5V BEC capable of at least 3A-5A continuous draw. Note: Do not power the heavy 20kg servos directly from the ESP32's 5V pin, you will fry the board! Use a dedicated BEC wired to the servos, and share the common ground.
https://www.readymaderc.com/products/details/castle-creations-bec-switching-regulator-10-amp-peak

Switches: 2X Momentary Button Switches.
https://www.amazon.com/dp/B07931588C

Servo Tester: To get the PWM values for center and maximum up/down range
https://www.amazon.com/dp/B08DM2CP3

Wiring: 2X Wago lever nuts/connectors (Highly recommended for cleanly splitting the 5V and Ground lines. Home Depot also sells these and probably other hardware stores near you. One Wago has all your grounds connected together and the other all your positive leads connected together).
https://www.digikey.com/short/q0m7mf3m

* **(Highly Recommended) Power Capacitor:** 1x 1000µF to 3300µF (10V, 16V, or 25V) Electrolytic Capacitor. Wired across the 5V and Ground Wago Connectors to prevent the heavy servos from causing voltage drops and rebooting the ESP32. (https://www.amazon.com/dp/B07R432MR2)

(Highly Recommended) Resistors: 2x 1kΩ - 4.7kΩ resistors (to place inline on the servo signal wires).
https://www.digikey.com/short/29dhzrrp

(Optional) Compass: Adafruit BNO085 9-DOF IMU. Note, I went with this chip because it doesn't require calibration like drone GPS chips.
https://www.amazon.com/dp/B0CDGZMLPP

(Optional) Trim Knob: Standard 10k linear potentiometer.
https://www.amazon.com/dp/B082FCRQS2

🖨️ 3D Model Files
The custom 3D-printed parts for the pan/tilt mechanics and electronics housing can be found here: MakerWorld: CRSF Antenna Tracker

Wiring Diagram
<img width="3085" height="8192" alt="ESP32 Controller Power-2026-03-23-063612" src="https://github.com/user-attachments/assets/3d7d37c4-5e34-43d7-a3b0-3aa658b68f0e" />



💻 Software & Library Requirements
The software used to compile the code and load it onto the ESP32 is the Arduino IDE.

Before compiling, you must install the following libraries via the Arduino IDE Library Manager (Sketch -> Include Library -> Manage Libraries):

ESP32Servo by Kevin Harrington

SparkFun u-blox GNSS v3 by SparkFun

Adafruit BNO08x by Adafruit

Adafruit SSD1306 by Adafruit

Adafruit GFX Library by Adafruit

Note: This project was successfully built and tested using the ESP32 Board Package v3.3.x by Espressif Systems. If a future major version update breaks compilation, use the Arduino Boards Manager to roll back to a 3.3.x release.

🚀 Quickstart Guide
Step 1: Configure Your Radio
Your radio must be told to broadcast its telemetry to the tracker.

Ensure you have flashed your TX Backpack firmware with the same Binding Phrase as your TX module and receiver.

Open your radio's Model Setup page and ensure Telemetry is turned ON. (If your radio isn't receiving GPS coordinates from your flight controller, the tracker has nothing to track!)

Open the ExpressLRS Lua Script on your radio, scroll down to Backpack, and ensure it is enabled and communicating.

Step 2: Find your ELRS Binding MAC Address (Crucial!)
Because this tracker sniffs raw ESP-NOW packets, it must impersonate your specific transmitter. You need to convert your ELRS Binding Phrase into a 6-digit UID array.

Go to the ExpressLRS UID Generator.

Type your secret Binding Phrase into the "Binding Phrase" box.

Look at the UID (bytes) output. It will look something like this: 252, 223, 149, 33, 213, 228.

Open config.h in this repository and paste those 6 numbers into the BINDING_MAC array.

Step 3: Find Your Servo Center Points
Not all 3D-printed gears and servos are perfectly aligned from the factory. I highly recommend using a cheap RC Servo Tester to physically find the exact PWM microsecond values where your specific pan servo faces dead-forward, and your tilt servo is perfectly level with the horizon.

Plug your servos into a tester and center your mechanical build.

Note the microsecond values (e.g., 1480 for Pan, 1550 for Tilt).

Enter these values into the PAN_CENTER_PWM and TILT_HORIZON_PWM definitions in config.h.

Step 4: Configure Hardware Toggles
Open config.h and configure your build:

If you did not install a BNO085 Compass, change #define USE_COMPASS to false.

If you did not install a pan-trim knob, change #define USE_TRIM_KNOB to false.

Step 5: Flash to the ESP32!
If you are new to the Arduino IDE, the ESP32 requires a quick initial setup before you can upload code.

Add ESP32 Board Support:

Go to File > Preferences.

In the "Additional Boards Manager URLs" field, paste this exact link:
https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

Click OK. Next, go to Tools > Board > Boards Manager, search for esp32 (by Espressif Systems), and click Install.

Select Your Board & Port:

Plug your ESP32 into your computer via USB. I would recommend using a USB cable that has the 5v pin blocked so not to accidently get power draw from your computer if battery isn't plugged into BEC. I used a small piece of kapton tape to cover 5v pin of a usb-a to usb-micro cable.

Go to Tools > Board > esp32 and select ESP32 Dev Module.

Go to Tools > Port and select the COM port that appeared when you plugged in the board.

Upload:

Click the right-pointing arrow (Upload) at the top left of the IDE.

Troubleshooting Tip: If the IDE output window says "Connecting..." and dots start appearing, press and hold the physical BOOT button on your ESP32 board for 1-2 seconds to force it into flashing mode!

🛠️ Troubleshooting
📺 Is your OLED screen black?
If your code compiled and flashed successfully, but your OLED screen remains completely black, you likely have a screen with an alternate I2C address.

Open AntTrack.ino in the Arduino IDE.

Scroll down to void setup() and look for this line:
if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))

Change 0x3C to 0x3D and re-upload the code.

✈️ ArduPilot Users: The "CRSF Passthrough" Trap
If you are flying a fixed-wing on ArduPilot, your tracker might sit there completely lifeless even if your ground station has a GPS lock and your radio is connected.

The Problem: If you enable "CRSF Passthrough Telemetry" in ArduPilot (typically used to run the Yaapu Telemetry script on your radio's screen), ArduPilot stops broadcasting standard CRSF GPS packets (Frame 0x02). Instead, it bundles all telemetry into a custom, compressed format that the tracker cannot read. Without those standard GPS packets, the tracker doesn't know where the plane is, and the servos will not wake up.

The Fix: You must disable CRSF Passthrough to restore standard GPS telemetry.

Connect your flight controller to Mission Planner (or QGroundControl).

Go to your Full Parameter Tree (or Parameter List) and search for RC_OPTIONS.

Check the bitmask value. "CRSF Passthrough" is controlled by Bit 8 (Value: 256).

If it is enabled, uncheck the "Passthrough" box in the bitmask helper (or manually subtract 256 from the total integer value).

Write the parameters and reboot your flight controller. It will instantly revert to broadcasting standard CRSF GPS packets, and your tracker will wake up!

🎯 Daily Flight Operations
1. Boot Sequence & Connection
When you power up the ground station, the tracker will wait for its local GPS to achieve 8 satellites. Once the ground station is locked, power up your aircraft. The tracker will blink its status LED until it receives an ELRS telemetry packet from your drone/plane confirming it also has 8 satellites.

2. Calibration
You must calibrate the tracker before every flight so it knows exactly how it is oriented in the world.

If using the BNO085 Compass (USE_COMPASS true): Place the tripod down facing your general flight area. Wait for the screen to say "Ready", and hold the calibration button for 1 second. The compass calculates True North and instantly aligns the math.

If NOT using a Compass (USE_COMPASS false): Carry your powered aircraft 20-30 meters away from the tracker. Walk back to the ground station, physically rotate the tripod or pan gear until the antennas are pointing dead-center at the aircraft, and hold the calibrate button for 1 second. Ensure you do not bump the tripod during the flight!

3. Flight
Once calibrated, the tracker will lock its servos dead-center until the aircraft flies beyond the MIN_TRACKING_DIST (default 2 meters). Once the aircraft leaves the "deadzone", the servos will wake up and begin tracking smoothly.

🛡️ The "Power Bump" Failsafe (Memory Restore)
There is nothing worse than accidentally kicking your ground station power cable when your aircraft is miles away. Because you cannot perform a visual calibration while the plane is in the air, this tracker features an automatic, time-aware non-volatile memory failsafe.

How it Saves You (When it Kicks In):
Every time you successfully calibrate the tracker, it silently saves its Home GPS coordinates, the exact servo offset math, and the live GPS atomic timestamp to the ESP32's internal flash memory.
If your tracker loses power mid-flight and reboots, it rapidly checks this memory. If it detects that the tracker is still sitting within 100 meters of its saved home location AND your drone is currently broadcasting from further away, it assumes you suffered a mid-flight power failure. It will instantly bypass the calibration gatekeeper, restore the previous math, and immediately resume tracking your aircraft!

The Auto-Expiration (3-Hour Timer):
You don't have to worry about the tracker accidentally using yesterday's calibration data. Because the system saves the absolute GPS timestamp, the failsafe memory automatically expires 3 hours after your last calibration. If you pack up and come back the next day, the tracker will safely ignore the old memory and wait for a fresh calibration.

How to Manually Clear It (The 5-Second Hold):
If you drive to a completely new flying location within that 3-hour window, the tracker might falsely think it just rebooted from a previous flight. To completely wipe the memory and start fresh:

Press and hold the calibration button for 5 seconds.

The OLED screen will prominently display "RELEASE TO CLEAR".

Release the button. The screen will confirm "FAILSAFE CLEARED," allowing you to perform a normal 1-second calibration for your new location.

💡 Pro-Tips & Advanced Usage
🔧 The "Startup Twitch" & Flashing Issues (Why you need a resistor)
ESP32 microcontrollers are incredibly sensitive during their boot sequence. When you try to flash new firmware or power on the tracker, the ESP32 might fail to connect to your computer, or your heavy 20kg servos might violently twitch and slam against their physical limits. This happens because the servo's internal circuitry can interfere with the ESP32's "strapping pins," confusing the board about whether it should be booting up or waiting for a firmware upload.

The Fix: Solder a simple 1kΩ to 4.7kΩ resistor inline on the PWM data wire between the ESP32 and each servo. This isolates the ESP32's pins just enough to protect the boot sequence, ensuring you can flash new firmware via USB without having to physically unplug your servos every time! It also acts as a safety buffer to stop the dreaded startup twitch.

The Cross-Wind Launch Trick (Managing Servo Limits)
Your pan servo has 270° of total rotation. When you calibrate the tracker, that calibration point becomes the physical dead-center of the servo, giving you 135° of tracking range to the left, and 135° to the right.

If you are flying a fixed-wing and need to launch cross-wind from a spot that is 90 degrees away from your main flight area, do not point the tripod at the launch pad to calibrate! If you do, you will only have 45° of servo travel remaining in the direction you actually want to fly, and the tracker will hit a physical wall.

The Solution:

Point the front of the tracker dead-center toward the middle of the airspace where you plan to spend the majority of your flight.

Walk your powered plane 20 feet directly in front of the tracker and press the calibrate button.

Pick the plane up and walk it over to your cross-wind launch spot. The tracker will pan to follow you as you walk.

Launch the plane. As you turn out toward your main flight area, the tracker will smoothly pan back toward the center of its travel, giving you maximum tracking room in all directions.

The Live RF Trim Knob
If you installed the optional 10k potentiometer, the tracker averages its position during the calibration sequence to establish "Zero". During flight, turning the knob will sweep the entire tracker array up to 20 degrees left or right. This allows you to manually "dial in" the invisible RF lobe of your patch antennas mid-flight for the absolute best video feed without resetting your calibration!
