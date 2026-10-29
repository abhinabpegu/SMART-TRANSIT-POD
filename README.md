 #Smart Cargo Transit Pod

**Team Ghee refilled** | NIT Silchar | Problem Statement 20: *Remote Asset Geofencing & Shock-Aware Transit Pod*

A small battery-powered box that rides inside a shipping crate and quietly records two things that damage cargo:

1. **Shocks and drops** (hard knocks, free-fall)
2. **Leaving the allowed area** (a "geofence" around a starting point)

It works completely **offline**. Every event is saved in the ESP32's own flash memory with a GPS time stamp, so after delivery you can check exactly what happened to the cargo.

> This is an **offline logger**, not a live cloud tracker. Data is read back later over Bluetooth (BLE) or the Serial Monitor.

---

## Table of contents

1. [The problem](#1-the-problem)
2. [What the pod does](#2-what-the-pod-does)
3. [How it works (in simple words)](#3-how-it-works-in-simple-words)
4. [Hardware](#4-hardware)
5. [Wiring](#5-wiring)
6. [Power system](#6-power-system)
7. [Software setup](#7-software-setup)
8. [How to use the pod](#8-how-to-use-the-pod)
9. [What the OLED shows](#9-what-the-oled-shows)
10. [Reading the logs (Serial and Bluetooth)](#10-reading-the-logs-serial-and-bluetooth)
11. [Settings you can change](#11-settings-you-can-change)
12. [How to test each feature](#12-how-to-test-each-feature)
13. [Safety notes](#13-safety-notes)
14. [Limitations](#14-limitations)
15. [Troubleshooting](#15-troubleshooting)
16. [Cost and future scope](#16-cost-and-future-scope)
17. [Repository contents](#17-repository-contents)
18. [Team](#18-team)

---

## 1. The problem

High-value cargo (medical supplies, art, industrial equipment) often travels through several handlers. Two things go wrong without anyone noticing:

- The package takes a **hard knock or a drop**, and nobody records it.
- The package **leaves its route or yard**, and nobody records that either.

Afterwards there is damage but no proof of when or where it happened. That leads to disputes and spoiled goods.

**Our constraints**

- No SD card, so logs must go into the ESP32's flash.
- No real-time clock, so time comes from GPS.
- The pod needs a clear view of the sky to get a GPS fix.
- We do **not** claim live cloud tracking. This is an offline logger.

---

## 2. What the pod does

| Feature | Detail |
|---|---|
| Shock detection | Logs an event when total g-force is **above 3.0 g** |
| Free-fall detection | Logs an event when total g-force stays **below 0.25 g** (the pod is falling) |
| Geofence | Logs an event when the pod is **more than 100 m** from the stored centre point |
| Offline log | Last **20 events** saved in ESP32 flash, each with GPS date, time and position |
| Local alarm | Buzzer beeps and red LED lights until someone acknowledges |
| Live display | 0.96" OLED shows GPS status, fence status, distance, highest shock and log count |
| Bluetooth (BLE) | Sends a live status line and can send the stored log to a phone |
| Survives power-off | Fence centre, logs and highest shock are kept in flash |

---

## 3. How it works (in simple words)

### 3.1 Measuring a shock

The MPU6050 sensor measures acceleration on three axes (x, y, z). Sitting still on a table it reads about **1 g** (that is just gravity). To get one single number for "how hard is the pod being pushed", we combine the three axes:

```
total g = sqrt( ax² + ay² + az² )
```

- If total g goes **above 3.0 g** → it was a **shock**.
- If total g goes **close to 0** (below 0.25 g) for a few readings in a row → the pod is in **free-fall** (a drop, before the impact).

The sensor is read **200 times per second** so short knocks are not missed. The range is set to ±16 g so a 3 g hit is comfortably inside what the sensor can measure.

To avoid false alarms:
- One impact is logged **once** (there is a 1 second hold-off after each shock).
- Free-fall needs **6 low readings in a row** (about 30 ms), so a single noisy reading cannot trigger it.

### 3.2 Measuring distance from the fence centre

The GPS module gives latitude and longitude. The Earth is a sphere, so we cannot use flat "Pythagoras" on lat/lon. We use the **Haversine formula**, which gives the great-circle distance in metres between two points on a sphere.

1. Press **BTN1** when the pod is at the starting point. Its position is saved as the **fence centre**.
2. Every time the GPS gives a new position, the code calculates the distance from that centre.
3. If the distance is **over 100 m for 3 GPS updates in a row**, a **fence breach** is logged.

Why 3 in a row? GPS positions jitter by a few metres. Requiring 3 readings avoids a false alarm from one bad reading. Fixes with poor accuracy (HDOP above 5) are ignored.

### 3.3 Saving to flash without an SD card

The ESP32 has built-in flash memory. The Arduino `Preferences` library (which uses the ESP32's **NVS**, non-volatile storage) saves small pieces of data that **stay after power-off**.

Each event is stored as a small record:

| Field | Meaning |
|---|---|
| type | 1 = shock, 2 = free-fall, 3 = fence breach |
| value | g-force (shock/free-fall) or distance in metres (breach) |
| date | GPS date (ddmmyy) |
| time | GPS time (hhmmss, UTC) |
| lat, lon | position when it happened |

The log is a **ring buffer of 20 entries**: when full, the newest entry overwrites the oldest. We only write to flash **when an event happens** (not continuously), which keeps flash wear low.

### 3.4 Time without a clock

The ESP32 has no real-time clock. The GPS satellites broadcast very accurate time, so the code reads the **date and time from the GPS** and stamps each log with it. (This needs a GPS fix.)

### 3.5 Program flow

```
setup():  start I2C, OLED, MPU6050, GPS, flash, BLE  →  load saved data from flash

loop() (runs forever):
   1. read GPS bytes                     (feed the GPS parser)
   2. check the two buttons
   3. every 5 ms: read MPU6050 → compute g → shock / free-fall check
   4. when a new GPS fix arrives → distance → fence check
   5. update buzzer + LEDs
   6. every 250 ms: redraw OLED
   7. BLE: send status / log if requested
```

When something is detected: **save to flash → turn on alarm**. The alarm stays on until you press BTN2.

---

## 4. Hardware

| Part | Used for |
|---|---|
| ESP32 DevKit (30-pin) | Brain: processing, flash storage, Bluetooth |
| MPU6050 (GY-521 breakout) | Accelerometer (shock / free-fall) |
| GPS NEO-6M module | Position and time |
| OLED 0.96" 128×64 (I2C, SSD1306) | Live status display |
| Active buzzer | Audible alarm |
| LED red + LED green | Alert and status lights |
| 2 × 220 Ω resistors | One per LED |
| 2 × push buttons | BTN1 set fence, BTN2 acknowledge/clear |
| Li-ion 3.7 V cell | Battery |
| TP4056 charger module **with protection** (B+/B−/OUT+/OUT−) | Charges the cell and protects it |
| 5 V boost converter | Raises 3.7 V to a steady 5 V |
| Slide/toggle switch | Main power switch |
| Breadboard or perfboard, jumper wires | Assembly |

---

## 5. Wiring

![Circuit connections](/wiring_diagram.svg)

### Pin table

| ESP32 pin | Connected to | Notes |
|---|---|---|
| GPIO21 | MPU6050 **SDA** and OLED **SDA** | I2C data (shared bus) |
| GPIO22 | MPU6050 **SCL** and OLED **SCL** | I2C clock (shared bus) |
| GPIO16 (RX2) | GPS **TX** | GPS talks to ESP32 |
| GPIO17 (TX2) | GPS **RX** | ESP32 talks to GPS |
| GPIO25 | Buzzer **+** | other buzzer leg to GND |
| GPIO26 | 220 Ω → red LED → GND | alert LED |
| GPIO27 | 220 Ω → green LED → GND | status LED |
| GPIO32 | BTN1 → GND | internal pull-up used |
| GPIO33 | BTN2 → GND | internal pull-up used |
| 3V3 | VCC of MPU6050, OLED, GPS | sensors run on 3.3 V |
| GND | GND of everything | **all grounds common** |
| VIN | 5 V boost output | main power |

Important wiring rules:

- **TX goes to RX.** GPS TX → ESP32 RX (GPIO16) and ESP32 TX (GPIO17) → GPS RX.
- The MPU6050 (address **0x68**) and OLED (address **0x3C**) share the same two I2C wires. That is normal. The breakout boards already include the pull-up resistors.
- The GPS module's VCC rating varies. Many NEO-6M boards accept 3.3 V to 5 V. **Check your module** before connecting.
- LED direction: current flows GPIO → resistor → LED → GND. The long leg (anode) faces the resistor.
- Buttons connect **between the GPIO and GND**. No external resistor is needed.

---

## 6. Power system

```
Li-ion 3.7 V → TP4056 (protected) → Power switch → 5 V boost → ESP32 VIN
```

- **TP4056:** charges the cell from USB and cuts off the cell if it is over-charged, over-discharged or shorted (use the version with the protection chip, which has separate B+/B− and OUT+/OUT− pads).
- **Boost converter:** set it to **5.0 V with a multimeter before** connecting it to the ESP32.
- **Charge with the power switch OFF.**
- The ESP32 board then makes its own 3.3 V for the sensors.

---

## 7. Software setup

### 7.1 Install the tools

1. Install the **Arduino IDE** (version 2.x).
2. Add ESP32 support: *File → Preferences → Additional boards manager URLs*, add Espressif's ESP32 package URL, then *Tools → Board → Boards Manager* and install **"esp32" by Espressif Systems**.
3. Select the board: *Tools → Board → ESP32 Arduino → **ESP32 Dev Module***.

### 7.2 Install the libraries

*Sketch → Include Library → Manage Libraries*, then search and install:

| Library | Author | Why |
|---|---|---|
| **TinyGPSPlus** | Mikal Hart | Reads the GPS data |
| **Adafruit MPU6050** | Adafruit | Reads the accelerometer (also installs *Adafruit Unified Sensor* and *Adafruit BusIO*) |
| **Adafruit SSD1306** | Adafruit | OLED driver |
| **Adafruit GFX Library** | Adafruit | Text and shapes on the OLED |

Built in with the ESP32 package (nothing to install): `Wire`, `Preferences`, `HardwareSerial`, and the BLE libraries.

### 7.3 Upload the code

1. Put `transit_pod.ino` in a folder named `transit_pod` (the Arduino IDE needs the folder name to match the file name).
2. Open it in the Arduino IDE.
3. Connect the ESP32 by USB. **Switch the pod battery power OFF / disconnect the boost converter while the USB cable is connected** (see [Safety notes](#13-safety-notes)).
4. Choose the right port in *Tools → Port*.
5. Click **Upload**. If it hangs on "Connecting...", hold the **BOOT** button on the ESP32 until it starts.
6. Open *Tools → Serial Monitor* and set **115200 baud**. You should see `Pod ready`.

If BLE gives compile errors on your ESP32 package version, set `#define USE_BLE 0` near the top of the file. Everything else still works.

---

## 8. How to use the pod

1. **Switch on** and wait for the boot beep (one short beep = ready).
2. **Take the pod outside** with a clear sky view. The green LED **blinks** while searching for GPS and turns **steady** when it has a good fix. The first fix can take a few minutes.
3. At the starting point, press **BTN1** to set the **fence centre**. The OLED shows `FENCE SET`. (If it says `NO GPS FIX`, wait for the green LED to go steady and try again.)
4. Put the pod in the crate and ship it. It monitors by itself.
5. If something happens, the **buzzer beeps and the red LED lights**. Press **BTN2 briefly** to silence it. The event stays in the log.
6. At the destination, read the log over **BLE** or the **Serial Monitor**.

### Buttons

| Button | Action | Result |
|---|---|---|
| **BTN1** (GPIO32) | Press | Saves the current GPS position as the fence centre (needs a GPS fix) |
| **BTN2** (GPIO33) | Short press | Acknowledges the alarm (buzzer + red LED off) |
| **BTN2** (GPIO33) | Hold 2 seconds, then release | **Clears all logs** and the highest-shock value |

### LEDs and buzzer

| Indicator | Meaning |
|---|---|
| Green LED steady | GPS has a good fix |
| Green LED blinking | Still searching for GPS |
| Red LED steady + buzzer beeping | Alarm active (shock, free-fall or breach) until acknowledged |

---

## 9. What the OLED shows

```
GPS: LOCKED            ← or "GPS: SEARCHING"
FENCE: OK              ← "FENCE: NOT SET" or "FENCE: BREACH" (shown as a white bar)
DIST: 42m / 100m       ← distance from the centre / allowed radius
MAX SHOCK: 1.2g OK     ← highest g seen; FAIL if it reached 3.0 g or more
LOGS: 03 IN FLASH      ← number of events stored
NOW: 1.01g             ← live g-force (or a short message like "ACK", "FENCE SET")
```

At rest the live g reading should be close to **1.0 g**.

---

## 10. Reading the logs (Serial and Bluetooth)

### 10.1 Serial Monitor (115200 baud)

Every time an event is saved, the pod prints a line like:

```
EVENT type=1 value=3.20 time=142859 flash_write=1234 us
```

- `type`: 1 = shock, 2 = free-fall, 3 = fence breach
- `value`: g-force, or distance in metres for a breach
- `time`: GPS time (UTC) as hhmmss
- `flash_write`: how long the flash write took, in microseconds (1000 µs = 1 ms). Use this to fill in your own flash-write-delay measurement.

### 10.2 Bluetooth (BLE)

Use a free phone app such as **nRF Connect** (Nordic) or **LightBlue**.

| Item | Value |
|---|---|
| Device name | `GheeRefilled-Pod` |
| Service UUID | `7a1c0001-5e2b-4c3a-9d1e-0f6a8b2c4d10` |
| Characteristic UUID | `7a1c0002-5e2b-4c3a-9d1e-0f6a8b2c4d10` (Read + Write + Notify) |

Steps:

1. Open the app, scan, and connect to `GheeRefilled-Pod`.
2. Open the characteristic above and **enable notifications**.
3. Once per second you receive a **status line**, for example `D142 G3.2 L3 F1`:

| Part | Meaning |
|---|---|
| `D142` | distance from fence centre in metres |
| `G3.2` | highest g-force seen |
| `L3` | number of logs stored |
| `F1` | 1 = currently outside the fence, 0 = inside |

4. To download the stored log, **write the text `DUMP`** to the characteristic. The pod sends two lines per event, oldest first, then `END`:

```
1,S,3.2,142859        ← event number, type (S shock / F free-fall / B breach), value, time hhmmss UTC
24.8333,92.7789       ← latitude, longitude
```

(The lines are short because the default BLE message size is 20 bytes.)

---

## 11. Settings you can change

All settings are constants near the top of `transit_pod.ino`.

| Constant | Default | What it does |
|---|---|---|
| `SHOCK_G` | 3.0 | Shock threshold in g |
| `FREEFALL_G` | 0.25 | Free-fall threshold in g |
| `FENCE_M` | 100 | Geofence radius in metres |
| `BREACH_CONFIRM` | 3 | GPS updates in a row outside the fence before logging a breach |
| `FREEFALL_SAMPLES` | 6 | Low readings in a row before free-fall is logged (~30 ms) |
| `MAX_HDOP` | 5.0 | GPS accuracy limit (lower = stricter) |
| `IMU_PERIOD_MS` | 5 | Sensor read interval (200 Hz) |
| `OLED_PERIOD_MS` | 250 | Screen refresh interval |
| `SHOCK_HOLDOFF` | 1000 | Minimum ms between two logged shocks |
| `MAX_LOGS` | 20 | Number of events kept in flash |
| `USE_BLE` | 1 | Set to 0 to build without Bluetooth |

Why these defaults: 3.0 g is a common fragile-cargo damage level, below 0.25 g means the object is falling, and 100 m is a typical size for a loading or transit yard. Change them to suit your cargo.

---

## 12. How to test each feature

Test on **soft foam or a bed**, never on a hard floor.

| Test | How | Expected |
|---|---|---|
| Boot | Power on | One short beep, OLED shows status, Serial says `Pod ready` |
| GPS fix | Go outside | Green LED steady, `GPS: LOCKED` |
| Set fence | Press BTN1 outside with a fix | `FENCE SET`, then `FENCE: OK` |
| Geofence | Walk or drive more than 100 m away | `FENCE: BREACH` bar, buzzer, red LED, log entry. Walk back: `FENCE: OK` |
| Shock | Give the pod a firm tap or drop it onto foam | `MAX SHOCK` rises; at 3.0 g or more an event is logged and the alarm sounds |
| Free-fall | Drop the pod from a short height onto foam | A free-fall event is logged |
| Flash write time | Cause an event with the Serial Monitor open | Note the `flash_write` value |
| Power-off memory | Cut power and restart | Fence centre, log count and highest shock are still there |
| Acknowledge | Press BTN2 briefly | Buzzer and red LED stop |
| Clear | Hold BTN2 for 2 s | `LOGS: 00 IN FLASH`, `MAX SHOCK` back to 0 |

### Measured results (fill in after your own testing)

| Test | Target | Measured |
|---|---|---|
| Shock detect | ≥ 3.0 g | |
| Geofence trigger distance | 100 m | |
| Flash write time | < 50 ms | |

---

## 13. Safety notes

- **Battery:** use a Li-ion cell with the **protected** TP4056 module. Check polarity before connecting. Never short the cell.
- **Boost converter:** set it to **5.0 V** with a multimeter *before* connecting the ESP32.
- **USB and VIN together:** do not power the ESP32 from USB and from the boost converter at the same time unless you added a diode. When uploading code, keep the pod's own battery power OFF.
- **Charging:** charge with the pod's power switch OFF. Do not leave a charging cell unattended.
- **Buzzer current:** an ESP32 pin should supply only about 12 mA. If your buzzer draws more, drive it through a small NPN transistor (for example BC547 with a 1 kΩ resistor on the base).
- **Testing:** do drop tests on soft foam. No open flames.

---

## 14. Limitations

- **Offline only.** There is no live cloud tracking. Logs are read after the trip (Bluetooth or USB).
- **GPS needs open sky.** The first fix can take minutes, especially indoors. Inside a closed crate or metal container the GPS may not work.
- **A very short spike can be missed** if it happens during the ~25 ms the OLED takes to redraw.
- **No compass.** The MPU6050 measures acceleration and tilt only, not direction.
- **20-event log.** After 20 events, the oldest are overwritten.
- Accuracy of the g-force and distance depends on sensor quality and GPS signal; they are good for flagging events, not for lab-grade measurement.

---

## 15. Troubleshooting

| Problem | Likely cause and fix |
|---|---|
| OLED blank / "OLED not found" | Check SDA→GPIO21, SCL→GPIO22, power, and that the address is 0x3C |
| "MPU6050 NOT FOUND" on screen | Check I2C wires and 3.3 V; address should be 0x68 |
| Green LED keeps blinking | No GPS fix yet. Go outside, wait a few minutes; check GPS TX→GPIO16 and RX→GPIO17 |
| `NO GPS FIX` when pressing BTN1 | Wait until the green LED is steady, then press again |
| Upload fails | Hold the BOOT button while "Connecting...", check the USB cable and port, keep battery power off |
| Buzzer always on after a shock | Press BTN2 briefly to acknowledge |
| No Bluetooth device visible | Wait a few seconds after boot; or set `USE_BLE` to 0 if the BLE library failed to compile |
| Random shocks logged | Mount the sensor firmly; very loose wiring can also cause false readings |
| Library errors when compiling | Install every library in [section 7.2](#72-install-the-libraries) |

---

## 16. Cost and future scope

**Estimated cost:** about ₹1,300 (~$16) for the prototype with off-the-shelf parts; about ₹650 (~$8) is the target for a custom single-board design in volume.

**Future scope**

- Cellular connectivity (NB-IoT / LTE-M) for continuous cloud updates
- Custom PCB and a rugged IP67 waterproof enclosure
- Mobile app to graph Bluetooth data and show events on a map
- Better impact capture using the MPU6050's motion interrupt

---

## 17. Repository contents

```
SMART-TRANSIT-POD/
├── transit_pod.ino        ← complete firmware (heavily commented)
├── README.md              ← this file
└── images/
    └── circuit_connections.png   ← wiring diagram
```

---

## 18. Team

**Team Ghee refilled**, NIT Silchar (First year)

- **Abhinab Pegu**: Team leader, product design and testing
- **Satyam Kesari**: Software integration lead
- **Syed Taha**: Hardware assembly
