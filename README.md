# Fall Detection and Alert Device for the Elderly

ESP32-C3 based smart fall detection device with WiFi connectivity and remote web dashboard control.

## Overview

| Feature | Description |
|---------|-------------|
| **Fall Detection** | State machine algorithm detecting freefall → impact → lying position |
| **Local Alert** | Buzzer (20s) + LED blinking on fall detection |
| **Web Dashboard** | Real-time sensor monitoring via HTTP |
| **Remote Control** | Stop alert / reset system via REST API |
| **Telegram Bot** | Push notifications to mobile device |
| **Auto Calibration** | 5-second MPU6050 calibration on startup |

## Hardware

### Components
| Component | Spec |
|-----------|------|
| ESP32-C3 | Main microcontroller |
| MPU6050 | 6-axis IMU (Accelerometer + Gyroscope) |
| Buzzer | Active buzzer, 3.3V |
| LED | Status indicator |
| 2x Push buttons | Cancel / SOS controls |

### Pinout

**MPU6050 (I2C)**
| ESP32-C3 | MPU6050 |
|----------|---------|
| GPIO 8 | SDA |
| GPIO 9 | SCL |
| 3.3V | VCC |
| GND | GND |

**GPIO Configuration**
| GPIO | Direction | Function |
|------|-----------|----------|
| 0 | Output | Buzzer |
| 1 | Output | LED |
| 5 | Input (Pull-up) | Cancel button |
| 6 | Input (Pull-up) | SOS button (hold 3s) |

### Sensor Placement
- **Position**: Worn on belt
- **X-axis**: Pointing up
- **Y-axis**: Pointing forward
- **Z-axis**: Horizontal

## Fall Detection Algorithm

```
┌───────┐  accel<0.5g   ┌───────────┐  accel≥1.5g  ┌────────┐  wait 1s  ┌──────────────┐  tilt≥70°+stable  ┌─────────────┐
│ IDLE  │ ───────────►  │ FREEFALL  │ ───────────►  │ IMPACT │ ─────────►  │ WAIT_LIE_DOWN│ ────────────────►  │    SOS      │
└───────┘               └───────────┘               └────────┘            └──────────────┘                    └─────────────┘
     ▲                        │                        │                     │
     │                        │                        │                     └──── tilt<70° (got up)
     └────────────────────────┴────────────────────────┘  (reset)
```

**Thresholds**
| Parameter | Value | Description |
|-----------|-------|-------------|
| Freefall | ≤ 0.5g | Weightlessness detected |
| Impact | ≥ 1.5g | Hard collision after fall |
| Lying angle | ≥ 70° | Horizontal position |
| Freefall timeout | 150ms | Max freefall duration |
| Impact check delay | 1000ms | Wait before angle check |
| Lie confirm time | 3500ms | Stable lying confirmation |
| Low-pass filter α | 0.5 | 50% old + 50% new value |

## Project Structure

```
├── main/
│   └── CODE.c              # Main application, GPIO, tasks
├── components/
│   ├── mpu6050/
│   │   ├── mpu6050.c       # I2C driver, raw data reading
│   │   ├── mpu6050.h
│   │   ├── roll_pitch.c    # Complementary filter for angle
│   │   ├── roll_pitch.h
│   │   └── mpu6050_constants.h
│   ├── fall_detection/
│   │   ├── fall_detection.c # State machine algorithm
│   │   └── fall_detection.h
│   ├── wifi/
│   │   ├── wifi.c          # Station mode + NTP sync
│   │   └── wifi.h
│   ├── webserver/
│   │   ├── webserver.c     # HTTP server + dashboard
│   │   └── webserver.h
│   └── telegram/
│       ├── telegram.c      # Async bot notifications
│       └── telegram.h
├── CMakeLists.txt
└── sdkconfig
```

## Build & Flash

```bash
# Build project
idf.py build

# Flash to device
idf.py flash

# Serial monitor (115200 baud)
idf.py monitor
```

## Web Dashboard

Access `http://<ESP32_IP>` after WiFi connection.

### REST API

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/api/data` | Sensor data JSON |
| GET | `/api/status` | Device status JSON |
| POST | `/api/alert/stop` | Cancel active alert |
| POST | `/api/reset` | Full system reset |

### GET /api/data Response
```json
{
  "accel": 9.81,
  "accel_g": 1.00,
  "gyro": 2.50,
  "roll": 5.2,
  "pitch": -3.1,
  "wifi_connected": 1,
  "alert_active": 0,
  "fall_state": 0,
  "max_tilt": 5.2,
  "uptime": 3600
}
```

## Configuration

**WiFi** (`components/wifi/wifi.h`)
```c
#define WIFI_SSID     "Your_SSID"
#define WIFI_PASS     "Your_Password"
```

**Telegram** (`main/CODE.c`)
```c
#define TELEGRAM_BOT_TOKEN  "your_bot_token"
#define TELEGRAM_CHAT_ID    "your_chat_id"
```

**Fall Detection** (`components/fall_detection/fall_detection.c`)
```c
static fall_detection_config_t default_config = {
    .filter_alpha = 0.5f,
    .accel_freefall_abs = 0.5f,
    .accel_impact_abs = 1.5f,
    .lying_angle_threshold = 70.0f,
    .timeout_freefall = 150,
    .timeout_impact_check = 1000,
    .wait_lie_down_time = 3500,
};
```

## System States

| State | Description |
|-------|-------------|
| 0 - IDLE | Normal monitoring |
| 1 - FREEFALL | Detected weightlessness |
| 2 - IMPACT | Detected collision |
| 3 - WAIT_LIE_DOWN | Waiting for stable lying |
| 4 - SOS | Fall confirmed, alert active |

## Operation Flow

1. **Startup**: 5-second calibration (LED blinks)
2. **Normal**: Continuous sensor monitoring at 100Hz
3. **Fall Detected**:
   - Buzzer activates for 20 seconds
   - LED blinks at 1Hz
   - Telegram notification sent
4. **Cancel**: Press button or call `/api/alert/stop`

## Debugging

```bash
# View logs
idf.py monitor

# Filter by tag
idf.py monitor | grep "FALL_DETECT"
idf.py monitor | grep "MPU6050"
idf.py monitor | grep "WIFI"
```

## License

MIT License
