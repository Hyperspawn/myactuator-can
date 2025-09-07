# MyActuator RMD Actuators Integration Guide for Dropbear

tried to put together a guide documenting my exp with myactuator servos while building **Dropbear**. I've put some docs and softwares I used along the way with my firmware code etc.

---

## 🚀 Quick Start


**Ready to build?** Start by sourcing the actuators from [MyActuator](#-myactuator-official) and then follow the [Hardware Setup](#-hardware-setup) and [Motor Configuration](#-motor-configuration).

**Need help?** Contact **Arics Tang** (MyActuator Global Sales Director):  
📱 **+86 17775378663** (WhatsApp/WeChat) | 📧 **tangxc@myactuator.com**

#### MyActuator Official
🌐 **Website**: [www.myactuator.com](https://www.myactuator.com/)  
📧 **Support**: tangxc@myactuator.com  
📱 **Direct Contact**: +86 17775378663 (WhatsApp/WeChat)

## Actuators to get for dropbear build
1. RMD-X8-PRO-1:9 x 12 qty
2. RMD-X10-1:7-V3 x 4 qty
3. EPS-CEM-60 (with ethercat) x 6 qty

---

## System Architecture

### Distributed Control

**Distributed control architecture**:

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Left Arm      │    │   Right Arm     │    │     Pelvis      │    │   Right Leg     │    │   Left Leg      │
│   ESP32         │    │   ESP32         │    │   ESP32         │    │   ESP32         │    │   ESP32         │
│  4x RMD X8 Pro  │    │  4x RMD X8 Pro  │    │  4x RMD X10 S2  │    │  4x RMD X8 Pro  │    │  4x RMD X8 Pro  │
│  1x RMD X10 S2  │    │  1x RMD X10 S2  │    │  1x RMD X10 S2  │    │  2x CEM-60      │    │  2x CEM-60      │
└─────────────────┘    └─────────────────┘    └─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │                       │                       │
         └───────────────────────┼───────────────────────┼───────────────────────┼───────────────────────┘
                                                         |          
                                               ┌─────────────────┐
                                               │   Main Brain    │
                                               │   (ROS/PC)      │
                                               └─────────────────┘
```

### Design Principles
- **Modular**: Each sub-assembly is independent
- **Scalable**: Easy to add/remove actuators
- **Robust**: CAN bus provides fault tolerance
- **Safe**: Per-motor fusing and watchdog timers

---

## Hardware Setup

### Bill of Materials (Per Sub-Assembly)

| Component | Quantity | Notes |
|-----------|----------|-------|
| **ESP32 Dev Board** | 1 | ESP32-WROOM-32 recommended |
| **MCP2515 CAN Board** | 1 | **Must have 8 MHz crystal** |
| **RMD X8 Pro Actuators** | 12 | RMD-X8-PRO series|
| **RMD X10 S2 Actuators** | 4 | RMD-X10-1:7-V3 series |
| **CEM-60 Actuators** | 6 | EPS-CEM-60 series |
| **Twisted Pair Cable** | 2m+ | For CANH/CANL signals |
| **120Ω Resistors** | 2 | Bus termination (one per end) |
| **Inline Fuses** | 5 | Per motor branch protection |
| **Power Supply** | 1 | 24-48V, 100A+ peak capacity |
| **Connectors** | As needed | JST, XT60, or similar |

### ESP32 ↔ MCP2515 Wiring

| MCP2515 Pin | ESP32 GPIO | Function |
|-------------|------------|----------|
| **CS** | **GPIO 5** | Chip Select (matches code) |
| **SCK** | GPIO 18 | SPI Clock |
| **MOSI** | GPIO 23 | Master Out, Slave In |
| **MISO** | GPIO 19 | Master In, Slave Out |
| **INT** | GPIO 4 | Interrupt (optional) |
| **VCC** | 3.3V/5V | Power (check board specs) |
| **GND** | GND | Common Ground |
| **CANH** | - | To actuator harness |
| **CANL** | - | To actuator harness |


### Power Distribution
```
24-48V Supply → Fuse → Bulk Cap → Motor
     ↓
   Common GND
```

---

## 📡 CAN Bus Configuration

### topology notes
- **linear topology**, no star configurations
- **120Ω termination** at both ends only (remove any middle terminators)
- **short stubs**, keep motor connections < 15cm (just in case)
- **common ground**, tie logic and power grounds together
- **1 Mbps baud rate**, stable for robot-scale wiring

### Bus Layout
```
[ESP32] ←→ [Motor1] ←→ [Motor2] ←→ [Motor3] ←→ [Motor4] ←→ [Motor5]
   ↑                                                              ↑
120Ω Terminator                                           120Ω Terminator
```

---

## ⚙️ Motor Configuration

My rough motor limits and assigned IDs:

```cpp
// Right Arm (IDs 21-25)
21: Right Hand Wrist    (0° to 50°)
22: Right Hand Elbow    (-31° to 31°)
23: Right Hand Rotate   (5° to 40°)
24: Right Shoulder Out  (-31° to 90°)
25: Right Shoulder Rot  (15° to 45°)

// Left Arm (IDs 31-35)
31: Left Hand Wrist     (0° to 50°)
32: Left Hand Elbow     (-40° to 31°)
33: Left Hand Rotate    (-140° to 140°)
34: Left Shoulder Out   (-90° to 0°)
35: Left Shoulder Rot   (0° to 90°)
```

### Initial Setup Checklist
1. ✅ **Power up one motor at a time** (prevents ID conflicts)
2. ✅ **Set unique CAN ID** using opcode `0x79`
3. ✅ **Power cycle** motor to save settings
4. ✅ **Verify ID** by sending status command `0x9A`
5. ✅ **Repeat** for all motors

### Critical Settings
- **Zero Position** (`0x64`) - Set at mechanical reference
- **Acceleration** (`0x43`) - Start conservative, protect gearboxes
- **Watchdog** (`0xB3`) - 200-500ms timeout for safety
- **PID Tuning** (`0x30/0x31/0x32`) - Test in RAM, store in ROM

---

## 📖 Essential CAN Commands

### Frame Format
- **TX ID**: `0x140 + MotorID`
- **RX ID**: `0x240 + MotorID`
- **Endianness**: Little-endian for numeric payloads
- **Angle Units**: `int32 = degrees × 100` (0.01°/LSB)

### Most Important Commands

| Command | Opcode | Description | Usage |
|---------|--------|-------------|-------|
| **Absolute Position** | `0xA4` | Move to specific angle | Primary control method |
| **Set Zero** | `0x64` | Store current as zero | Critical for calibration |
| **Set Acceleration** | `0x43` | Control motion smoothness | Protect gearboxes |
| **Set Watchdog** | `0xB3` | Safety timeout | Prevent runaway |
| **Read Status** | `0x9A` | Temperature, voltage, errors | Health monitoring |
| **Set CAN ID** | `0x79` | Assign unique ID | One-time setup |
| **Soft Stop** | `0x81` | Controlled halt | Safety |
| **Hard Shutdown** | `0x80` | Emergency stop | Safety |

### Complete Command Reference

<details>
<summary>Click to expand full command list</summary>

| Category | Opcode | Description | Key Parameters |
|----------|--------|-------------|----------------|
| **Position Control** | | | |
| | `0xA4` | Absolute position | `[2-3]`: maxSpeed, `[4-7]`: angle |
| | `0xA8` | Incremental position | `[4-7]`: delta angle |
| | `0xA6` | Single-turn position | 0-360° bounded |
| **Control Modes** | | | |
| | `0xA2` | Speed control | Velocity setpoint |
| | `0xA1` | Torque control | Force/torque setpoint |
| **Status & Feedback** | | | |
| | `0x9A` | Status 1 | Temperature, voltage, errors |
| | `0x9C` | Status 2 | Current, speed |
| | `0x9D` | Status 3 | Position feedback |
| | `0x60` | Multi-turn position | Absolute position |
| | `0x92` | Angle feedback | Current angle |
| **Configuration** | | | |
| | `0x79` | Set CAN ID | Unique ID per motor |
| | `0xB4` | Set baud rate | Keep 1 Mbps |
| | `0x64` | Set zero (current) | Store current as zero |
| | `0x63` | Set zero (value) | Store given value |
| | `0x43` | Set acceleration | Smooth motion |
| | `0xB3` | Set watchdog | Safety timeout |
| **PID Tuning** | | | |
| | `0x30` | Read PID | Current parameters |
| | `0x31` | Write PID (RAM) | Test parameters |
| | `0x32` | Write PID (ROM) | Store parameters |
| **Safety** | | | |
| | `0x81` | Soft stop | Controlled halt |
| | `0x80` | Hard shutdown | Emergency stop |
| | `0x77` | Brake release | If supported |
| | `0x78` | Brake lock | If supported |
| **Utility** | | | |
| | `0x76` | Reset | Soft reboot |
| | `0x70` | Mode control | Debug modes |
| | `0x71` | Power control | Power management |

</details>

---

## My rough working firmware
```bash
# Pair Bluetooth device "Dropbear"
# Send commands via Bluetooth or USB serial:

31 20    # Move motor 31 to 20 degrees
home     # Safe home position
hi       # Wave gesture demo
```

### Complete ESP32 Sketch

```cpp
/*
 * Dropbear Humanoid - MyActuator RMD Control
 * ESP32 + MCP2515 + CAN Bus
 * 
 * Features:
 * - Bluetooth control via "Dropbear" pairing
 * - USB serial interface for debugging
 * - Per-motor safety limits
 * - Watchdog timers for safety
 * - Demo poses and gestures
 */

#include <SPI.h>
#include "mcp_can.h"
#include "BluetoothSerial.h"

// Hardware configuration
#define CAN_CS 5                    // MCP2515 Chip Select pin
MCP_CAN CAN0(CAN_CS);              // CAN controller instance
BluetoothSerial SerialBT;          // Bluetooth interface

// Motor configuration
struct AngleLimits { 
  float min_angle; 
  float max_angle; 
};

const int NUM_MOTORS = 10;
const byte motorIDs[NUM_MOTORS] = {21,22,23,24,25,31,32,33,34,35};

// Per-motor safety limits (degrees) - Based on our humanoid design
AngleLimits motorLimits[NUM_MOTORS] = {
  {0.0, 50.0},     // Motor 21: Right Hand Wrist
  {-31.0, 31.0},   // Motor 22: Right Hand Elbow
  {5.0, 40.0},     // Motor 23: Right Hand Rotate
  {-31.0, 90.0},   // Motor 24: Right Shoulder Out
  {15.0, 45.0},    // Motor 25: Right Shoulder Rotate
  {0.0, 50.0},     // Motor 31: Left Hand Wrist
  {-40.0, 31.0},   // Motor 32: Left Hand Elbow
  {-140.0,140.0},  // Motor 33: Left Hand Rotate
  {-90.0, 0.0},    // Motor 34: Left Shoulder Out
  {0.0, 90.0}      // Motor 35: Left Shoulder Rotate
};

// Input handling
String inputString = "";
bool stringComplete = false;

/**
 * Convert degrees to CAN frame format (degrees × 100)
 */
int32_t angleToA4(float deg) { 
  return (int32_t)(deg * 100.0f); 
}

/**
 * Handle incoming serial data
 */
void serialEvent() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      stringComplete = true;
    } else {
      inputString += c;
    }
  }
}

/**
 * Find motor index by ID
 */
int getMotorIndex(byte id) { 
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motorIDs[i] == id) return i;
  }
  return -1; 
}

/**
 * Move motor to absolute position with safety limits
 */
void moveAbsolutePosition(byte id, float deg, uint16_t maxSpeed = 100) {
  int idx = getMotorIndex(id);
  if (idx == -1) { 
    Serial.printf("❌ Invalid motor ID: %d\n", id); 
    return; 
  }
  
  // Apply safety limits
  float lo = motorLimits[idx].min_angle;
  float hi = motorLimits[idx].max_angle;
  if (deg < lo) { 
    Serial.printf("⚠ ID %d: clamp to %.2f°\n", id, lo); 
    deg = lo; 
  }
  if (deg > hi) { 
    Serial.printf("⚠ ID %d: clamp to %.2f°\n", id, hi); 
    deg = hi; 
  }

  // Build CAN frame for 0xA4 (absolute position)
  int32_t a = angleToA4(deg);
  byte d[8] = {
    0xA4, 0x00,                                    // Opcode
    (byte)(maxSpeed & 0xFF), (byte)(maxSpeed >> 8), // Max speed
    (byte)(a & 0xFF), (byte)((a >> 8) & 0xFF),      // Angle (little-endian)
    (byte)((a >> 16) & 0xFF), (byte)((a >> 24) & 0xFF)
  };
  
  CAN0.sendMsgBuf(0x140 + id, 0, 8, d);
  Serial.printf("✅ ID %d → %.2f° (A4)\n", id, deg);
}

/**
 * Set watchdog timer for motor safety
 */
void setWatchdog(byte id, uint16_t ms) {
  byte d[8] = {
    0xB3, 0,                                    // Opcode
    (byte)(ms & 0xFF), (byte)(ms >> 8),         // Timeout (little-endian)
    0, 0, 0, 0
  };
  CAN0.sendMsgBuf(0x140 + id, 0, 8, d);
}

/**
 * Read motor status (temperature, voltage, errors)
 */
void readStatus1(byte id) {
  byte d[8] = {0x9A, 0, 0, 0, 0, 0, 0, 0};
  CAN0.sendMsgBuf(0x140 + id, 0, 8, d);
  
  long rx;
  byte len, buf[8];
  if (CAN0.checkReceive() == CAN_MSGAVAIL && 
      CAN0.readMsgBuf(&rx, &len, buf) == CAN_OK) {
    if (rx == (0x240 + id) && buf[0] == 0x9A) {
      uint8_t err = buf[1];
      uint8_t temp = buf[2];
      uint16_t v_dV = buf[3] | (buf[4] << 8);
      Serial.printf("ID %d: err=0x%02X, T=%d°C, V=%.1fV\n", 
                   id, err, temp, v_dV / 10.0f);
    }
  }
}

/**
 * Setup function - initialize all systems
 */
void setup() {
  Serial.begin(115200);
  SerialBT.begin("Dropbear");
  inputString.reserve(30);

  // Initialize CAN bus
  if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("❌ CAN INIT FAILED");
    while (1);
  }
  CAN0.setMode(MCP_NORMAL);
  Serial.println("✅ CAN BUS Ready");
  
  // Set up watchdog timers for all motors
  for (byte id : motorIDs) {
    setWatchdog(id, 300); // 300ms timeout
  }
  
  Serial.println("🔧 Type: <motor_id> <angle> | hi | home");
}

/**
 * Demo pose - wave gesture
 */
void hiPose() {
  moveAbsolutePosition(31, 20);   delay(1200);
  moveAbsolutePosition(32, -20);  delay(1200);
  moveAbsolutePosition(33, -80);  delay(1200);
  moveAbsolutePosition(34, -90);  delay(1200);
  moveAbsolutePosition(35, 10);   delay(1200);
  
  // Wave gesture
  moveAbsolutePosition(32, -20, 30); delay(600);
  moveAbsolutePosition(32, -25, 30); delay(600);
  moveAbsolutePosition(32, -20, 30); delay(600);
  moveAbsolutePosition(32, -25, 30); delay(600);
  
  // Return to home
  moveAbsolutePosition(31, 20);   delay(1200);
  moveAbsolutePosition(32, -40);  delay(1200);
  moveAbsolutePosition(33, 0);    delay(1200);
  moveAbsolutePosition(34, 0);    delay(1200);
  moveAbsolutePosition(35, 0);
}

/**
 * Safe home position
 */
void homing() {
  moveAbsolutePosition(31, 20);   delay(200);
  moveAbsolutePosition(32, -40);  delay(200);
  moveAbsolutePosition(33, 0);    delay(200);
  moveAbsolutePosition(34, 0);    delay(200);
  moveAbsolutePosition(35, 0);
}

/**
 * Main loop - handle commands and monitoring
 */
void loop() {
  // Handle Bluetooth input
  while (SerialBT.available()) {
    char c = (char)SerialBT.read();
    if (c == '\n') {
      stringComplete = true;
    } else {
      inputString += c;
    }
  }
  
  // Process completed commands
  if (stringComplete) {
    inputString.trim();
    
    if (inputString == "hi") {
      hiPose();
    } else if (inputString == "home") {
      homing();
    } else {
      // Parse motor command: "ID angle"
      int sp = inputString.indexOf(' ');
      if (sp > 0) {
        byte id = inputString.substring(0, sp).toInt();
        float ang = inputString.substring(sp + 1).toFloat();
        moveAbsolutePosition(id, ang);
      } else {
        SerialBT.println("⚠ Use: <motor_id> <angle> | hi | home");
      }
    }
    
    inputString = "";
    stringComplete = false;
  }

  // Periodic status monitoring
  static uint32_t lastStatus = 0;
  if (millis() - lastStatus > 200) {
    lastStatus = millis();
    readStatus1(31); // Monitor motor 31
  }
}
```

---

## 🛡️ Safety Guidelines

### ⚠️ Break stuff at your own risk

### E-Stop Strategy
1. **Soft Stop** (`0x81`) - Controlled halt for all motors
2. **Hard Shutdown** (`0x80`) - Emergency power cutoff
3. **Watchdog Timers** (`0xB3`) - Automatic stop on communication loss
4. **Thermal Protection** - Monitor temperatures via `0x9A`


---

## 🔧 PID Tuning Guide

### My rough tuning process

Based on my experience with Dropbear, here's my rough tuning methodology:

### 1. Initial Setup
```cpp
// Start with conservative settings
setAcceleration(motorID, 50);  // Low acceleration
setWatchdog(motorID, 300);     // 300ms timeout
```

### 2. PID Tuning Order
1. **Torque Loop** - Start with basic force control
2. **Speed Loop** - Add velocity control  
3. **Position Loop** - Fine-tune positioning

### 3. Tuning Commands
```cpp
// Read current PID parameters
readPID(motorID);           // 0x30

// Test new parameters in RAM (temporary)
writePID_RAM(motorID, kp, ki, kd);  // 0x31

// Store working parameters in ROM (permanent)
writePID_ROM(motorID, kp, ki, kd);  // 0x32
```

### 4. Potential Starting Values

| Motor Type | Kp | Ki | Kd | Notes |
|------------|----|----|----|----|
| **Wrist** | 50 | 10 | 5 | Light load, fast response |
| **Elbow** | 80 | 15 | 8 | Medium load, smooth motion |
| **Shoulder** | 100 | 20 | 10 | Heavy load, stable control |

### 5. Tuning Tips from my Experience
- **Start low** - Begin with 50% of recommended values
- **Test incrementally** - Change one parameter at a time
- **Monitor temperature** - Watch for overheating during tuning
- **Use RAM first** - Test in RAM before storing in ROM
- **Log everything** - Record what works for future reference

---

## 🚨 Troubleshooting

### Common Issues I Encountered

| Problem | Cause | Solution |
|---------|-------|----------|
| **Nothing moves** | Wrong crystal frequency | Check MCP2515 crystal (8MHz vs 16MHz) |
| **Random faults** | Extra terminators | Remove middle terminators, keep only ends |
| **Wrong angles** | Not zeroed properly | Use `0x64` at mechanical zero |
| **Settings lost** | RAM vs ROM | Use `0x32` to store in ROM |
| **Harsh motion** | High acceleration | Reduce `0x43` values |
| **Communication errors** | Bus issues | Check wiring, terminators, grounding |
| **Overheating** | Aggressive PID | Reduce Kp, Ki values |
| **Oscillation** | High Kp | Reduce proportional gain |

### Debug Commands
```bash
# Check motor status
0x9A  # Temperature, voltage, errors
0x9C  # Current, speed
0x9D  # Position feedback

# Verify settings
0x30  # Read PID parameters
0x42  # Read acceleration
```

### Debugging checklist
1. **Check CAN bus** - Verify wiring and terminators
2. **Test communication** - Send status commands
3. **Verify motor IDs** - Ensure unique IDs assigned
4. **Check power** - Monitor voltage and current
5. **Review settings** - Confirm acceleration and PID values

---

## Advanced 

### Multi-Motor Control
- **Time-sliced commands** - Avoid CAN bus overload
- **Synchronous moves** - Queue targets, dispatch together
- **Incremental control** - Use `0xA8` for smooth gaits

### Helper Functions
```cpp
// Zero current position
void zeroHere(byte id) {
  byte d[8] = {0x64,0,0,0,0,0,0,0};
  CAN0.sendMsgBuf(0x140+id,0,8,d);
}

// Set acceleration
void writeAccel(byte id, uint16_t accel) {
  byte d[8] = {0x43,0,(byte)(accel&0xFF),(byte)(accel>>8),0,0,0,0};
  CAN0.sendMsgBuf(0x140+id,0,8,d);
}

// Emergency stops
void softStopAll() {
  for(byte id: motorIDs) {
    byte d[8] = {0x81,0,0,0,0,0,0,0};
    CAN0.sendMsgBuf(0x140+id,0,8,d);
  }
}

void shutdownAll() {
  for(byte id: motorIDs) {
    byte d[8] = {0x80,0,0,0,0,0,0,0};
    CAN0.sendMsgBuf(0x140+id,0,8,d);
  }
}
```
---

## My trip to MyActuator in Shanghai

Robit had already ordered the actuators from MyActuator long ago, during the research phasse, then it was called gyems actuator. It was before sckientific made a video about myactuator. Those were the old ones and now I wanted new set of motors for my next prototype. And I wanted to explore china.

I was in touch with Arics, Cofounder and Global Sales Director of Myactuator. He's always super helpful. I met him directly at their factory in Huaqiao, near Shanghai.

The factory tour was amazing - seeing their assembly lines, testing processes, and meeting the engineering team. We had great technical discussions with their CTO about their new line of cycloidal motors and their experimennts with an inhouse humaoid robobt. The whole team was genuinely interested in our project and super welcoming.

After the business stuff out of the way during the day, we went out for dinner and I'm only saying this on a readme page cuz it was so liberating to talk to Arics and Giri (giridhar) it was like I was hanging out with my own friends. We spoke about tech and life in china, etc. 

Check out my factory visit video: https://photos.app.goo.gl/Zsanz9FSKmc7ZVjC8

If you're considering MyActuator servos, definitely reach out to Arics directly. He knows robotics and will take care of you.

---e