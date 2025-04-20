//Created by Event_Horizon
#include <BluetoothSerial.h>
#include <EEPROM.h>
#include <math.h>

BluetoothSerial SerialBT;
HardwareSerial ArduinoSerial(1);  // Use UART1

#define IN1  13
#define IN2  12
#define IN3  14
#define IN4  27
#define EA   32
#define EB   33

// Speed levels
#define SPEED_LOW     0
#define SPEED_MEDIUM  1
#define SPEED_HIGH    2

int currentSpeed = SPEED_MEDIUM;
unsigned long lastCommandTime = 0;
const unsigned long disconnectTimeout = 3000; // 3 seconds

#define EEPROM_ADDR 0

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32_BT_MOTOR");
  ArduinoSerial.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EA, OUTPUT);
  pinMode(EB, OUTPUT);

  digitalWrite(EA, HIGH);
  digitalWrite(EB, HIGH);

  EEPROM.begin(4);
  currentSpeed = EEPROM.read(EEPROM_ADDR);
  if (currentSpeed > 2) currentSpeed = SPEED_MEDIUM;

  stopMotors();
  Serial.println("Bluetooth Started. Waiting for commands...");
}

void loop() {
  if (SerialBT.available()) {
    String command = SerialBT.readStringUntil('\n');
    command.trim();

    Serial.print("Received: ");
    Serial.println(command);
    lastCommandTime = millis();

    executeCommand(command);
  }

  if (millis() - lastCommandTime > disconnectTimeout) {
    stopAll();
    lastCommandTime = millis();
  }
}

void stopAll() {
  stopMotors();
  ArduinoSerial.println("Stop_Arm");
  SerialBT.println("All motors and servos stopped");
}

void executeCommand(String command) {
  if (command == "F") {
    runSmooth(moveForward, getForwardBackwardDuration());
    SerialBT.println("Moving forward");
  } else if (command == "B") {
    runSmooth(moveBackward, getForwardBackwardDuration());
    SerialBT.println("Moving backward");
  } else if (command == "L") {
    runSmooth(turnLeft, 250);  // 0.25 sec fixed
    SerialBT.println("Turning left");
  } else if (command == "R") {
    runSmooth(turnRight, 250); // 0.25 sec fixed
    SerialBT.println("Turning right");
  } else if (command == "STOP") {
    stopMotors();
    SerialBT.println("Motors stopped");
  } else if (command == "SPEED_LOW") {
    currentSpeed = SPEED_LOW;
    EEPROM.write(EEPROM_ADDR, currentSpeed);
    EEPROM.commit();
    SerialBT.println("Speed set to LOW");
  } else if (command == "SPEED_MEDIUM") {
    currentSpeed = SPEED_MEDIUM;
    EEPROM.write(EEPROM_ADDR, currentSpeed);
    EEPROM.commit();
    SerialBT.println("Speed set to MEDIUM");
  } else if (command == "SPEED_HIGH") {
    currentSpeed = SPEED_HIGH;
    EEPROM.write(EEPROM_ADDR, currentSpeed);
    EEPROM.commit();
    SerialBT.println("Speed set to HIGH");
  } else if (
    command == "STEP_CW" || command == "STEP_CCW" ||
    command == "STEP2_CW" || command == "STEP2_CCW" ||
    command == "SERVO2_CW" || command == "SERVO2_CCW" ||
    command == "SERVO2_CCW_90" || command == "Stop_Arm"
  ) {
    ArduinoSerial.println(command);
    SerialBT.println("Command sent to Arduino: " + command);
  } else {
    SerialBT.println("Invalid Command");
  }
}

// Smooth sinusoidal run
void runSmooth(void (*motorFunc)(), int durationMs) {
  int totalSteps = 300;
  float stepDuration = (float)durationMs * 1000.0 / totalSteps; // in microseconds

  for (int i = 0; i < totalSteps; i++) {
    float angle = PI * i / totalSteps; // 0 to PI
    float intensity = sin(angle); // smooth ramp
    int pulseWidth = (int)(intensity * stepDuration);

    if (pulseWidth > 0) {
      motorFunc();
      delayMicroseconds(pulseWidth);
      stopMotors();
      delayMicroseconds((int)(stepDuration - pulseWidth));
    } else {
      delayMicroseconds((int)stepDuration);
    }
  }
}

int getForwardBackwardDuration() {
  switch (currentSpeed) {
    case SPEED_LOW: return 500;
    case SPEED_MEDIUM: return 1500;
    case SPEED_HIGH: return 3000;
    default: return 1500;
  }
}

void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
