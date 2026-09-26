#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Target 1 = CH0
// Target 2 = CH1
// ...
// Target 7 = CH6

const int SERVO_DOWN = 350;
const int SERVO_UP   = 150;

int currentPos[7] = {350, 350, 350, 350, 350, 350, 350};

void moveServoSmooth(int channel, int targetPos, int stepDelay = 3) {
  int startPos = currentPos[channel];
  if (startPos == targetPos) return;

  int step = (targetPos > startPos) ? 2 : -2;
  
  if (step > 0) {
    for (int p = startPos; p <= targetPos; p += step) {
      pwm.setPWM(channel, 0, p);
      delay(stepDelay);
    }
  } else {
    for (int p = startPos; p >= targetPos; p += step) {
      pwm.setPWM(channel, 0, p);
      delay(stepDelay);
    }
  }
  pwm.setPWM(channel, 0, targetPos);
  currentPos[channel] = targetPos;
}

void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);

  pwm.begin();
  pwm.setPWMFreq(50);

  delay(200);

  // Make sure all 7 start DOWN smoothly
  for (int channel = 0; channel < 7; channel++) {
    pwm.setPWM(channel, 0, SERVO_DOWN);
    currentPos[channel] = SERVO_DOWN;
  }

  Serial.println("STATUS: READY");
}

void loop() {
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim(); // Clean whitespace and newlines

    if (line.startsWith("UP,")) {
      String idStr = line.substring(3);
      int targetId = idStr.toInt(); // 1 to 7
      if (targetId >= 1 && targetId <= 7) {
        int channel = targetId - 1;
        moveServoSmooth(channel, SERVO_UP, 3); // Smooth raise
        Serial.print("UP_CONFIRMED,");
        Serial.println(targetId);
      }
    }
    else if (line.startsWith("DOWN,")) {
      String idStr = line.substring(5);
      if (idStr == "ALL") {
        for (int channel = 0; channel < 7; channel++) {
          moveServoSmooth(channel, SERVO_DOWN, 3); // Smooth drop
        }
        Serial.println("DOWN_CONFIRMED,ALL");
      } else {
        int targetId = idStr.toInt(); // 1 to 7
        if (targetId >= 1 && targetId <= 7) {
          int channel = targetId - 1;
          moveServoSmooth(channel, SERVO_DOWN, 3); // Smooth drop
          Serial.print("DOWN_CONFIRMED,");
          Serial.println(targetId);
        }
      }
    }
    else if (line == "PING") {
      Serial.println("OK: PING_ACK (ESP32 RUNNING 115200 BAUD)");
    }
    else if (line == "STATUS") {
      Serial.println("STATUS: READY");
    }
  }
}