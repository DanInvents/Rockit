# Rockit Firmware 3.0

Rockit Firmware 3.0 is a major update over the previous Arduino-based firmware. It keeps the same core flight modes while improving launch detection, recovery reliability, configuration, logging, usability, and firmware updates.

**Firmware version:** 3.0  
**Current build:** `55d0209`  
**Runtime:** MicroPython 1.29.0 / `RPI_PICO`\n\n[Download the Rockit Rev.1 User Manual - Firmware 3.0 (PDF)](Rockit_User_Manual_Firmware_3.0.pdf)

---

## Main improvements over the previous firmware

### 1. More reliable launch detection

Firmware 3.0 improves launch detection by combining altitude and acceleration events instead of allowing them to be satisfied independently over a long period.

Improvements include:

- acceleration measured relative to a pre-launch baseline;
- altitude measured relative to the armed baseline;
- acceleration and altitude launch events must occur within a defined time window;
- separate real-flight and bench-test thresholds.

Current real-flight settings use:

- **10 m** altitude change;
- **2 g** acceleration change;
- **10 consecutive acceleration samples**;
- a **2 s correlation window**.

This reduces the risk of false launch detection caused by handling, vibration, or unrelated sensor events.

---

### 2. Robust automatic apogee detection

Automatic mode **A** has been substantially redesigned.

Instead of reacting to a very small altitude decrease, Firmware 3.0 now:

- tracks the **filtered altitude peak**;
- applies a post-launch lockout;
- requires a minimum peak altitude;
- requires a meaningful drop from the filtered peak;
- requires the drop to persist before apogee is accepted.

Current real-flight parameters are:

- **1.5 s** post-launch lockout;
- **10 m** minimum filtered peak;
- **2 m** drop from peak;
- **300 ms** persistence before apogee confirmation.

This makes automatic deployment much more resistant to pressure noise and temporary altitude dips during ascent.

---

### 3. Improved timer mode

Timer mode **B** continues to deploy the recovery system after a user-programmed delay following launch detection.

- Servo 1 deploys after the delay configured with rotary position **D**.
- Servo 2 deploys after the additional delay configured with rotary position **2**.
- A small additional separation is used so both servos do not start at exactly the same instant.

---

### 4. More reliable recovery-servo control

Servo handling has been reworked to improve reliability and reduce unnecessary power consumption.

Improvements include:

- explicit deployment-state tracking;
- reliable detach timing after deployment;
- improved sequencing between servo 1 and servo 2;
- reduced chance of simultaneous current spikes;
- better PWM resource handling;
- servos are detached after movement rather than remaining powered unnecessarily.

---

### 5. Improved servo startup behavior

At startup, each servo is given sufficient time to reach its programmed initial position before being detached.

The servos are handled sequentially, which reduces current spikes and makes startup positioning more dependable.

---

### 6. Dedicated position-3 servo test

Rotary position **3** is now a self-contained recovery-system test.

When Rockit is powered on with the switch at position 3:

1. Servo 1 moves to its programmed initial position.
2. Servo 2 moves to its programmed initial position.
3. Rockit waits approximately 5 seconds.
4. Servo 1 moves to its programmed final position.
5. Servo 2 moves to its programmed final position.
6. The flight computer remains inactive.

This allows the recovery mechanism to be checked easily after the flight computer has been installed inside the rocket.

---

### 7. Shorter arming delay

The startup arming delay has been reduced to approximately **3 seconds**.

This makes launch preparation faster while keeping a clear startup sequence before the system becomes armed.

---

### 8. Human-readable persistent settings

The previous firmware stored configuration values directly in EEPROM addresses.

Firmware 3.0 uses a human-readable settings file:

`rockit_settings.json`

Stored values include:

- automatic-mode deployment delay;
- timer-mode deployment delay;
- servo 1 initial position;
- servo 1 final position;
- servo 2 initial position;
- servo 2 final position;
- delay between servo 1 and servo 2;
- rotary-switch hardware profile.

The firmware also avoids unnecessary flash writes when a value has not changed.

---

### 9. Improved configuration workflow

The rotary-switch configuration process has been redesigned to reduce accidental or unclear configuration changes.

Improvements include:

- settings are not saved immediately when entering a configuration position;
- the selected value must remain stable before saving;
- clearer visual and audible confirmation;
- more time to select servo positions;
- improved persistence and save verification.

---

### 10. Support for both rotary-switch hardware variants

Some Rockit units use a standard-coded rotary switch, while others use a complementary-coded switch.

Firmware 3.0 supports **both variants using the same firmware**.

The standalone updater can:

- let the user choose the switch type manually; or
- detect it automatically by asking the user to place the rotary switch at position **A**.

The selected hardware profile is stored in `rockit_settings.json`.

The standard-switch path has been verified on physical hardware. The complementary-coded path is implemented from the known complementary switch encoding but has not yet been verified on a physical complementary-coded Rockit.

---

### 11. Better pre-launch data handling

Firmware 3.0 keeps a rolling **100-sample pre-launch history**.

When launch is detected, those samples are written to the CSV so the log contains data from immediately before takeoff as well as the complete post-launch flight.

The pre-launch acceleration calculation is also aligned with the main flight calculation for a more consistent transition into flight data.

---

### 12. Improved SD-card flight logging

Flight data continues to be recorded to microSD, including:

- time;
- altitude;
- filtered altitude;
- longitudinal acceleration;
- perpendicular acceleration;
- temperature.

Periodic flushing is used to reduce the risk of losing flight data.

---

### 13. Improved maximum-altitude measurement

Maximum altitude is tracked internally as a floating-point value throughout the flight and rounded only when the final altitude report is generated.

This preserves more resolution during peak-altitude tracking.

---

### 14. Improved audible altitude reporting

The maximum-altitude beep sequence is clearer and easier to interpret after landing.

The report now includes:

- **two short beeps** as a start marker;
- the altitude digits;
- **two short beeps** as an end marker;
- a long beep representation for the digit **0**;
- repeated reporting after the flight.

For example, an altitude of **67 m** is reported as:

- two short marker beeps;
- six short beeps;
- seven short beeps;
- two short marker beeps.

---

### 15. Faster post-flight altitude report

The previous firmware waited approximately five minutes before entering altitude-reporting mode.

Firmware 3.0 uses a **60-second logging timeout**, so the maximum-altitude report begins roughly one minute after takeoff.

---

### 16. Improved buzzer and PWM handling

The RP2040 can place the buzzer and one servo output on related PWM resources.

Firmware 3.0 explicitly manages PWM creation, shutdown, and pin state so buzzer activity does not interfere with servo operation.

The buzzer is also silenced once at launch rather than being repeatedly manipulated while recovery servos may be active.

---

### 17. Correct LED flash counts

The previous implementation could produce one more flash than requested in some routines.

Firmware 3.0 now produces the requested number of flashes exactly.

---

### 18. Better standalone operation

Normal customer firmware runs with serial debugging disabled.

This reduces unnecessary USB/serial activity and makes normal operation independent of a connected terminal.

Debug output can still be enabled during development or diagnosis.

---

### 19. Standalone Windows firmware updater

Firmware 3.0 introduces the **Triskel Instruments Rockit Firmware Updater** for Windows.

Customers no longer need:

- Python;
- `mpremote`;
- Arduino IDE;
- VS Code;
- manual copying of firmware files.

The updater can:

- detect an existing MicroPython Rockit;
- verify the firmware package;
- back up `rockit_settings.json`;
- install firmware modules;
- restore saved settings;
- install `main.py` last;
- verify core modules and the SD driver;
- reset Rockit automatically;
- migrate an older Arduino-based or blank RP2040 through BOOTSEL;
- install the validated MicroPython 1.29.0 `RPI_PICO` runtime;
- support both rotary-switch hardware variants.

---

### 20. Settings backup and restore during updates

Before updating an existing MicroPython Rockit, the updater backs up the current settings to the computer.

Servo positions, recovery delays, and the rotary-switch profile are restored after the firmware modules are installed.

This allows firmware updates without requiring the customer to reconfigure the flight computer.

---

### 21. Arduino-to-MicroPython migration

Older Rockit units running the previous Arduino firmware can be migrated to Firmware 3.0.

The updater guides the customer through RP2040 **BOOTSEL** mode and installs:

`RPI_PICO-20260824-v1.29.0.uf2`

After MicroPython starts, the updater installs Rockit Firmware 3.0 automatically.

Legacy Arduino EEPROM settings are not automatically transferred, so a migrated unit must be configured again after the upgrade.

---

### 22. Improved USB re-enumeration handling

Windows can take some time to detect the RP2040 after a UF2 is flashed.

The updater now handles this more gracefully and, if necessary, asks the customer to disconnect and reconnect Rockit normally and retry detection instead of immediately treating the update as failed.

---

### 23. Firmware package integrity checking

The updater verifies the packaged firmware files before installation.

A manifest stores hashes for the firmware modules so installation is stopped if the bundled package has been unexpectedly modified or corrupted.

---

### 24. Modular firmware architecture

The previous firmware was organized as Arduino `.ino` files sharing global state.

Firmware 3.0 is split into dedicated MicroPython modules for:

- flight-state detection;
- recovery control;
- sensor handling;
- servo control;
- SD logging;
- settings storage;
- rotary-switch handling;
- battery monitoring;
- Kalman filtering;
- buzzer/LED behavior;
- pre-launch buffering.

This makes the firmware easier to test, maintain, and extend without changing unrelated flight logic.

---

## Flight validation

Firmware 3.0 has been tested in real flights using both primary recovery modes:

- **A — Automatic/apogee mode**
- **B — Timer mode**

Both modes have operated successfully in actual flight testing.

The current standard rotary-switch hardware profile has also been verified on physical hardware.

---

## Compatibility

Firmware 3.0 targets Rockit hardware based on the **RP2040** and uses:

- **MicroPython 1.29.0**;
- target build: **RPI_PICO**;
- microSD storage;
- MS5637 barometric sensor;
- ADXL343 accelerometer;
- two recovery servos;
- 16-position rotary switch.

Both standard-coded and complementary-coded rotary-switch variants are supported.

---

## Customer update

For customers, the recommended update method is:

`Rockit_Firmware_Updater_v3.0.exe`

No separate Python installation is required.

Follow the instructions displayed by the updater. Older Arduino-based units will be guided through BOOTSEL migration automatically.

---

## Version

**Rockit Firmware 3.0**  
**Build:** `55d0209`  
**Triskel Instruments**
