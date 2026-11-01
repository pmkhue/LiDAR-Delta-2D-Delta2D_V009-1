# Delta2D Radar V7.1 - compile fix

This release fixes `SectorResult does not name a type` in Arduino IDE by declaring the type in `SectorResult.h`, included before Arduino automatically generates function prototypes. The LiDAR protocol, UART D7 and radar algorithms are unchanged.

## Open and upload

1. Extract the ZIP; keep both files in the **same** directory `Delta2D_Radar_V7_1`.
2. Open `Delta2D_Radar_V7_1.ino` in Arduino IDE. Do not copy only the `.ino` without `SectorResult.h`.
3. Select `LOLIN(WEMOS) D1 R2 & mini` (ESP8266).
4. Install/update `kaiaai/LDS` with `lds_all_models.h` and `LDS_DELTA_2C_115200` if not installed.
5. Upload, reconnect LiDAR TX to D7/GPIO13, and visit `http://192.168.4.1` on Wi-Fi `Delta2D-Radar`, password `12345678`.

## Hardware precautions

LiDAR TX must be 3.3V-compatible on D7. LiDAR sensor supply was tested at 5V and motor at 3V in the user's setup; verify regulator currents and common ground. Do not drive the motor from an ESP8266 GPIO.

## Validation

Corrected the Arduino preprocessor type ordering and inspected reference occurrences. Not compiled with a full ESP8266 Arduino toolchain in this environment or tested on hardware.
