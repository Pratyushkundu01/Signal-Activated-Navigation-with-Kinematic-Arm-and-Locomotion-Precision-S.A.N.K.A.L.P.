//Created by Event_Horizon
#include <Servo.h>
#include <EEPROM.h>

// Custom Stepper Class
class Stepper {
  public:
    int pin[4] = {0, 0, 0, 0};
    int lpx = 1;
    int bitl = 0; // 0 = stop, 1 = CW, 2 = CCW
    byte spd = 6;

    Stepper(int pn1, int pn2, int pn3, int pn4) {
      pin[0] = pn1;
      pin[1] = pn2;
      pin[2] = pn3;
      pin[3] = pn4;
    }

    void begin() {
      for (int i = 0; i < 4; i++) {
        pinMode(pin[i], OUTPUT);
        digitalWrite(pin[i], LOW);
      }
    }

    void stop() {
      for (int i = 0; i < 4; i++) {
        digitalWrite(pin[i], LOW);
      }
      bitl = 0;
    }

    void run() {
      if (bitl == 0) return;

      switch (bitl) {
        case 1: // CW
          switch (lpx) {
            case 4: digitalWrite(pin[3], LOW); digitalWrite(pin[0], HIGH); lpx = 1; break;
            case 1: digitalWrite(pin[0], LOW); digitalWrite(pin[1], HIGH); lpx++; break;
            case 2: digitalWrite(pin[1], LOW); digitalWrite(pin[2], HIGH); lpx++; break;
            case 3: digitalWrite(pin[2], LOW); digitalWrite(pin[3], HIGH); lpx++; break;
          }
          delay(spd);
          break;

        case 2: // CCW
          switch (lpx) {
            case 1: digitalWrite(pin[3], HIGH); digitalWrite(pin[0], LOW); lpx++; break;
            case 2: digitalWrite(pin[3], LOW); digitalWrite(pin[2], HIGH); lpx++; break;
            case 3: digitalWrite(pin[2], LOW); digitalWrite(pin[1], HIGH); lpx++; break;
            case 4: digitalWrite(pin[1], LOW); digitalWrite(pin[0], HIGH); lpx = 1; break;
          }
          delay(spd);
          break;
      }
    }
};

// Instantiate steppers
Stepper stepper1(8, 9, 10, 11);
Stepper stepper2(4, 5, 6, 7);

// Servo
Servo servo2;
int servo2Angle = 90;
const int EEPROM_SERVO2_ADDR = 0;

// Timer variables
unsigned long stepper1StartTime = 0;
unsigned long stepper2StartTime = 0;
bool stepper1Running = false;
bool stepper2Running = false;

// Run durations
const unsigned long stepper1Duration = 300; // 0.3s
const unsigned long stepper2Duration = 300;  // 0.3s

void setup() {
  Serial.begin(9600);
  stepper1.begin();
  stepper2.begin();
  servo2.attach(3);

  // Restore servo2 position from EEPROM
  int storedAngle = EEPROM.read(EEPROM_SERVO2_ADDR);
  if (storedAngle >= 0 && storedAngle <= 90) {
    servo2Angle = storedAngle;
  } else {
    servo2Angle = 90;
  }

  servo2.write(servo2Angle);
  Serial.print("Arduino Ready. Servo at angle: ");
  Serial.println(servo2Angle);
}

void loop() {
  unsigned long currentTime = millis();

  // Run Stepper 1
  if (stepper1Running) {
    stepper1.run();
    if (currentTime - stepper1StartTime >= stepper1Duration) {
      stepper1.stop();
      stepper1Running = false;
      Serial.println("Stepper 1 auto-stopped after 1.5s");
    }
  }

  // Run Stepper 2
  if (stepper2Running) {
    stepper2.run();
    if (currentTime - stepper2StartTime >= stepper2Duration) {
      stepper2.stop();
      stepper2Running = false;
      Serial.println("Stepper 2 auto-stopped after 0.3s");
    }
  }

  // Handle serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "STEP_CW") {
      stepper1.bitl = 1;
      stepper1StartTime = millis();
      stepper1Running = true;

      stepper2.stop();
      stepper2Running = false;
      Serial.println("Stepper 1 CW for 1.5s");

    } else if (command == "STEP_CCW") {
      stepper1.bitl = 2;
      stepper1StartTime = millis();
      stepper1Running = true;

      stepper2.stop();
      stepper2Running = false;
      Serial.println("Stepper 1 CCW for 1.5s");

    } else if (command == "STEP2_CW") {
      stepper2.bitl = 1;
      stepper2StartTime = millis();
      stepper2Running = true;

      stepper1.stop();
      stepper1Running = false;
      Serial.println("Stepper 2 CW for 0.3s");

    } else if (command == "STEP2_CCW") {
      stepper2.bitl = 2;
      stepper2StartTime = millis();
      stepper2Running = true;

      stepper1.stop();
      stepper1Running = false;
      Serial.println("Stepper 2 CCW for 0.3s");

    } else if (command == "Stop_Arm") {
      stepper1.stop();
      stepper2.stop();
      stepper1Running = false;
      stepper2Running = false;
      Serial.println("All Arm Stop");

    } else if (command == "SERVO2_CW") {
      int target = min(servo2Angle + 15, 90);
      for (int pos = servo2Angle + 1; pos <= target; pos++) {
        servo2.write(pos);
        delay(20);
      }
      servo2Angle = target;
      if (EEPROM.read(EEPROM_SERVO2_ADDR) != servo2Angle) {
        EEPROM.write(EEPROM_SERVO2_ADDR, servo2Angle);
      }
      Serial.print("Servo 2 moved CW to ");
      Serial.println(servo2Angle);

    } else if (command == "SERVO2_CCW") {
      int target = max(servo2Angle - 15, 0);
      for (int pos = servo2Angle - 1; pos >= target; pos--) {
        servo2.write(pos);
        delay(20);
      }
      servo2Angle = target;
      if (EEPROM.read(EEPROM_SERVO2_ADDR) != servo2Angle) {
        EEPROM.write(EEPROM_SERVO2_ADDR, servo2Angle);
      }
      Serial.print("Servo 2 moved CCW to ");
      Serial.println(servo2Angle);

    } else if (command == "SERVO2_CCW_90") {
      Serial.println("Servo 2 rotating full CCW to 0°");
      for (int pos = servo2Angle - 1; pos >= 0; pos--) {
        servo2.write(pos);
        delay(20);
      }
      servo2Angle = 0;
      if (EEPROM.read(EEPROM_SERVO2_ADDR) != servo2Angle) {
        EEPROM.write(EEPROM_SERVO2_ADDR, servo2Angle);
      }

    } else {
      Serial.println("Unknown command");
    }
  }
}
