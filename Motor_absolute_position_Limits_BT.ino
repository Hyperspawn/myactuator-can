#include <SPI.h>
#include "mcp_can.h"
#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

#define CAN_CS 5
MCP_CAN CAN0(CAN_CS);

// Angle limits for each motor (in degrees)
struct AngleLimits {
  float min_angle;
  float max_angle;
};


const int NUM_MOTORS = 10;
const byte motorIDs[NUM_MOTORS] = {21, 22, 23, 24, 25, 31, 32, 33, 34, 35};
AngleLimits motorLimits[NUM_MOTORS] = {
  {0.0, 50.0},   // Motor Right Hand wrist
  {-31.0, 31.0},  // Motor Right Hand Elbow
  {5.0, 40.0},   // Motor Right Hand rotate
  {-31.0, 90.0},   // Motor Right Hand Shoulder out in
  {15.0, 45.0},   // Motor Right Hand Shoulder rotate
  {0.0, 50.0},   // Motor Left Hand wrist
  {-40.0, 31.0},  // Motor Left Hand Elbow
  {-140.0, 140.0},   // Motor Left Hand rotate
  {-90.0, 0.0},   // Motor Left Hand Shoulder out in
  {0.0, 90.0}   // Motor Left Hand Shoulder rotate
};

// Converts angle in degrees to int32_t where 1 LSB = 0.01°
int32_t angleToA4(float angle_deg) {
  return (int32_t)(angle_deg * 100.0);
}

String inputString = "";
bool stringComplete = false;

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

// Find index of motor ID in the motorIDs array
int getMotorIndex(byte motor_id) {
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motorIDs[i] == motor_id) return i;
  }
  return -1;  // Not found
}

void moveAbsolutePosition(byte motor_id, float angle_deg, uint16_t maxSpeed = 100) {
  int motorIndex = getMotorIndex(motor_id);
  if (motorIndex == -1) {
    Serial.print("❌ Invalid motor ID: ");
    Serial.println(motor_id);
    return;
  }

  // Get limits for this motor
  float min_limit = motorLimits[motorIndex].min_angle;
  float max_limit = motorLimits[motorIndex].max_angle;

  // Clamp angle
  if (angle_deg < min_limit) {
    Serial.print("⚠ Motor ");
    Serial.print(motor_id);
    Serial.print(": Angle too low, clamping to ");
    Serial.println(min_limit);
    angle_deg = min_limit;
  }
  if (angle_deg > max_limit) {
    Serial.print("⚠ Motor ");
    Serial.print(motor_id);
    Serial.print(": Angle too high, clamping to ");
    Serial.println(max_limit);
    angle_deg = max_limit;
  }

  int32_t angle_val = angleToA4(angle_deg);
  long can_id = 0x140 + motor_id;
  byte data[8];

  data[0] = 0xA4;
  data[1] = 0x00;
  data[2] = (uint8_t)(maxSpeed & 0xFF);
  data[3] = (uint8_t)(maxSpeed >> 8);
  data[4] = (uint8_t)(angle_val & 0xFF);
  data[5] = (uint8_t)((angle_val >> 8) & 0xFF);
  data[6] = (uint8_t)((angle_val >> 16) & 0xFF);
  data[7] = (uint8_t)((angle_val >> 24) & 0xFF);

  CAN0.sendMsgBuf(can_id, 0, 8, data);

  Serial.print("✅ Motor ");
  Serial.print(motor_id);
  Serial.print(" moving to ");
  Serial.print(angle_deg);
  Serial.println("° (Absolute Position A4)");
}

void setup() {
  Serial.begin(115200);
  SerialBT.begin("Dropbear");
  inputString.reserve(30);

  if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) != CAN_OK) {
    Serial.println("❌ CAN INIT FAILED");
    while (1);
  }

  CAN0.setMode(MCP_NORMAL);
  Serial.println("✅ CAN BUS Ready");
  Serial.println("🔧 Type: <motor_id> <angle>. Example: 11 30");
  Serial.println("ℹ️ Limits are set individually per motor.");
}
void hiPose() {
  moveAbsolutePosition(31, 20);
  delay(1200);
  moveAbsolutePosition(32, -20);
  delay(1200);
  moveAbsolutePosition(33, -80);
  delay(1200);
  moveAbsolutePosition(34, -90);
  delay(1200);
  moveAbsolutePosition(35, 10);
  delay(1200);
  moveAbsolutePosition(32, -20, 30);
  delay(600);
  moveAbsolutePosition(32, -25, 30);
  delay(600);
  moveAbsolutePosition(32, -20, 30);
  delay(600);
  moveAbsolutePosition(32, -25, 30);
  delay(600);
  //homing again
  moveAbsolutePosition(31, 20);
  delay(1200);
  moveAbsolutePosition(32, -40);
  delay(1200);
  moveAbsolutePosition(33, 0);
  delay(1200);
  moveAbsolutePosition(34, 0);
  delay(1200);
  moveAbsolutePosition(35, 0);
}
void homing() {
  moveAbsolutePosition(31, 20);
  delay(200);
  moveAbsolutePosition(32, -40);
  delay(200);
  moveAbsolutePosition(33, 0);
  delay(200);
  moveAbsolutePosition(34, 0);
  delay(200);
  moveAbsolutePosition(35, 0);
}
void loop() {
  // Read from Bluetooth
  while (SerialBT.available()) {
    char inChar = (char)SerialBT.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }

  if (stringComplete) {
    inputString.trim(); // remove any \r or trailing spaces

    if (inputString == "hi") {
      hiPose();
    } else if (inputString == "home") {
      homing();
    } else {
      int spaceIndex = inputString.indexOf(' ');
      if (spaceIndex > 0) {
        byte motorID = inputString.substring(0, spaceIndex).toInt();
        float angle = inputString.substring(spaceIndex + 1).toFloat();
        moveAbsolutePosition(motorID, angle);
      } else {
        SerialBT.println("⚠ Invalid input. Use: <motor_id> <angle> or hi");
      }
    }

    inputString = "";
    stringComplete = false;
  }
}