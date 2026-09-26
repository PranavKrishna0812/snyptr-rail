// ==============================================================================
// ESP32 NEMA-23 RAIL TARGET POSITIONING & TRACKING SYSTEM
// ==============================================================================
// Hardware Setup:
// - Motor: NEMA-23 Stepper Motor
// - Pulley: GT2 20-Tooth (2mm pitch -> 40mm = 4cm travel per revolution)
// - Driver: TB6600 / DM542 / A4988 / TMC2209 or equivalent
// - Calculations:
//     200 full steps/rev / 4 cm/rev = 50 steps/cm (at 1x full-stepping)
//     Microstepping multiplier adjusts STEPS_PER_CM automatically.
// ==============================================================================

// --- PIN CONFIGURATION ---
// Connected to TB6600: PUL+ -> GPIO 25, DIR+ -> GPIO 32 (ENA left disconnected)
#define STEP_PIN 25
#define DIR_PIN  32

// Set to true to reverse motor direction (swaps HIGH/LOW for forward/backward)
#define INVERT_DIR true

// --- MECHANICAL CONFIGURATION ---
// 30-Tooth GT2 Pulley (2mm pitch):
// 30 teeth × 2 mm = 60 mm = 6.0 cm per revolution
const float CM_PER_REV    = 6.0;

// Microstepping: Set to 1, 2, 4, 8, 16 depending on TB6600 DIP switches
// (TIP: Setting TB6600 to 1/4 or 1/8 microstepping makes motion whisper-quiet!)
#define MICROSTEPPING 1
const float STEPS_PER_REV = 200.0 * MICROSTEPPING;
const float STEPS_PER_CM  = STEPS_PER_REV / CM_PER_REV; // 33.3333 steps/cm at 1x

// Rail Limits (in cm) - 1.74m total viable rail length
const float MIN_POSITION_CM = 0.0;
const float MAX_POSITION_CM = 174.0;

// --- MOTION PROFILE & DYNAMIC SPEED CONTROL ---
// Base reference (100% speed): 1230us = ~813 steps/sec = ~24.4 cm/sec
const unsigned long BASE_CRUISE_DELAY_US = 1230;
int currentSpeedPercent                  = 100;   // 15% to 150% (Default: 100%)
unsigned long maxSpeedDelayUs            = BASE_CRUISE_DELAY_US;
unsigned long startSpeedDelayUs          = 2800;  // Soft-start speed delay
unsigned long accelSteps                 = 80;    // Steps spent ramping up

void setSpeedPercent(int pct) {
  if (pct < 15) pct = 15;
  if (pct > 150) pct = 150;
  currentSpeedPercent = pct;

  // Delay is inversely proportional to speed:
  // At 100%: 1230 us (~24.4 cm/s)
  // At 50%:  2460 us (~12.2 cm/s)
  // At 130%: 946 us  (~31.7 cm/s)
  maxSpeedDelayUs = (unsigned long)((float)BASE_CRUISE_DELAY_US * 100.0 / (float)pct);
  startSpeedDelayUs = maxSpeedDelayUs * 2.2;
  if (startSpeedDelayUs < maxSpeedDelayUs + 400) {
    startSpeedDelayUs = maxSpeedDelayUs + 400;
  }
  accelSteps = map(pct, 15, 150, 30, 90);

  float stepsPerSec = 1000000.0 / maxSpeedDelayUs;
  float cmPerSec = stepsPerSec / STEPS_PER_CM;
  Serial.printf("[SPEED] Set to %d%% -> Delay: %lu us (~%.1f cm/sec)\n",
                currentSpeedPercent, maxSpeedDelayUs, cmPerSec);
}

// Continuous Manual Drive (No end parameters, user determines stop)
enum ContinuousDriveState {
  DRIVE_IDLE,
  DRIVE_RIGHT,
  DRIVE_LEFT
};
ContinuousDriveState driveState = DRIVE_IDLE;
unsigned long continuousSteps = 0;
unsigned long driveStartTime = 0;

// --- POSITION TRACKING ---
// Internal tracking uses integer steps to eliminate cumulative floating-point drift
long currentStepPosition = 0;        // Discrete position relative to 0
long totalStepsTravelled = 0;        // Total cumulative odometer steps

// Converts step counts to centimeters
float getDistanceCm() {
  return (float)currentStepPosition / STEPS_PER_CM;
}

// Converts total odometer steps to centimeters
float getTotalDistanceTravelledCm() {
  return (float)totalStepsTravelled / STEPS_PER_CM;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5); // 5ms timeout ensures readStringUntil never pauses motor stepping!

  // Explicitly initialize TB6600 driver pins to LOW
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);

  // Guarantee motor drive state is completely IDLE on startup
  driveState = DRIVE_IDLE;
  continuousSteps = 0;

  // Flush any startup electrical noise or bootloader garbage from serial RX
  delay(100);
  while (Serial.available()) {
    Serial.read();
  }

  Serial.println();
  Serial.println("==================================================");
  Serial.println("       ESP32 NEMA-23 RAIL POSITIONING SYSTEM      ");
  Serial.println("==================================================");
  Serial.printf("Configuration: %.2f steps/cm (Microstepping: %dx)\n", STEPS_PER_CM, MICROSTEPPING);
  Serial.println("Zero reference set at: 0.00 cm");
  Serial.println("MOTOR STATE: STOPPED [IDLE] - Waiting for commands.");
  Serial.println("Commands:");
  Serial.println("  RIGHT / FWD  - Continuous drive to right (user controls stop)");
  Serial.println("  LEFT / REV   - Continuous drive to left (user controls stop)");
  Serial.println("  STOP         - Instantly halt motor");
  Serial.println("  SPEED,<pct>  - Set speed percentage (15% to 150%, e.g., SPEED,60)");
  Serial.println("  TARGET,<cm>  - Move to absolute position (e.g., TARGET,150)");
  Serial.println("  REL,<cm>     - Move relative +/- distance (e.g., REL,-20)");
  Serial.println("  ZERO         - Reset current position as 0.00 cm");
  Serial.println("  STATUS       - Display current position, speed & odometer");
  Serial.println("  PING         - Connectivity handshake");
  Serial.println("==================================================");
}

// Moves the stepper motor by exact step count with smooth trapezoidal acceleration
void stepMotor(long steps, bool forward) {
  if (steps <= 0) return;

  // Set direction (inverted according to INVERT_DIR)
  bool dirSignal = forward ? (INVERT_DIR ? LOW : HIGH) : (INVERT_DIR ? HIGH : LOW);
  digitalWrite(DIR_PIN, dirSignal);
  delayMicroseconds(20); // Direction setup time

  // Determine ramp length
  unsigned long rampSteps = accelSteps;
  if (steps < (long)(2 * rampSteps)) {
    rampSteps = steps / 2;
  }
  unsigned long decelStartStep = steps - rampSteps;

  for (long i = 0; i < steps; i++) {
    // Calculate step delay with smooth linear ramp
    unsigned long stepDelay = maxSpeedDelayUs;

    if (rampSteps > 0) {
      if (i < rampSteps) {
        // Acceleration phase
        stepDelay = startSpeedDelayUs - 
                    ((startSpeedDelayUs - maxSpeedDelayUs) * i) / rampSteps;
      } else if (i >= decelStartStep) {
        // Deceleration phase
        unsigned long decelProgress = i - decelStartStep;
        stepDelay = maxSpeedDelayUs + 
                    ((startSpeedDelayUs - maxSpeedDelayUs) * decelProgress) / rampSteps;
      }
    }

    // Step pulse
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(10); // Standard pulse width
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay > 10 ? stepDelay - 10 : 10);

    // Update real-time position tracking step-by-step
    if (forward) {
      currentStepPosition++;
    } else {
      currentStepPosition--;
    }
    totalStepsTravelled++;

    // Allow user to send STOP command during long travel
    if (Serial.available()) {
      char c = Serial.peek();
      if (c == 'S' || c == 's') {
        String peekCmd = Serial.readStringUntil('\n');
        peekCmd.trim();
        if (peekCmd.equalsIgnoreCase("STOP")) {
          Serial.println("\n[ALERT] Emergency Stop triggered!");
          break;
        }
      }
    }
  }
}

// Move to absolute target position from zero distance
void moveTo(float targetCm) {
  long targetSteps = lround(targetCm * STEPS_PER_CM);
  long stepsToMove = targetSteps - currentStepPosition;

  if (stepsToMove == 0) {
    Serial.println("Target already reached.");
    return;
  }

  bool forward = (stepsToMove > 0);
  long absSteps = labs(stepsToMove);

  Serial.printf("Moving from %.2f cm -> %.2f cm | Delta: %.2f cm (%ld steps)\n",
                getDistanceCm(), targetCm, (float)stepsToMove / STEPS_PER_CM, absSteps);

  stepMotor(absSteps, forward);

  Serial.printf("Movement complete. Current position: %.2f cm (Steps: %ld)\n",
                getDistanceCm(), currentStepPosition);
}

// Move relative distance from current location
void moveRelative(float deltaCm) {
  float target = getDistanceCm() + deltaCm;
  moveTo(target);
}

void printStatus() {
  Serial.println("----------- SYSTEM STATUS -----------");
  Serial.printf("Current Position:   %.2f cm (%ld steps)\n", getDistanceCm(), currentStepPosition);
  Serial.printf("Total Distance Run: %.2f cm (%ld steps)\n", getTotalDistanceTravelledCm(), totalStepsTravelled);
  Serial.printf("Motor Speed Level:  %d%% (Cruise Delay: %lu us, ~%.1f cm/sec)\n", 
                currentSpeedPercent, maxSpeedDelayUs, (1000000.0 / maxSpeedDelayUs) / STEPS_PER_CM);
  Serial.println("End Parameters:     DISABLED (User determines stop)");
  Serial.println("-------------------------------------");
}

void loop() {
  // Check for incoming serial commands
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command.length() > 0) {
      if (command.equalsIgnoreCase("FWD") || command.equalsIgnoreCase("RIGHT") || command.equalsIgnoreCase("DRIVE_FWD")) {
        driveState = DRIVE_RIGHT;
        continuousSteps = 0;
        bool dirSignal = INVERT_DIR ? LOW : HIGH;
        digitalWrite(DIR_PIN, dirSignal);
        delayMicroseconds(20);
        Serial.println("[DRIVE] Moving RIGHT continuously. User determines stop.");
      }
      else if (command.equalsIgnoreCase("REV") || command.equalsIgnoreCase("LEFT") || command.equalsIgnoreCase("DRIVE_REV")) {
        driveState = DRIVE_LEFT;
        continuousSteps = 0;
        bool dirSignal = INVERT_DIR ? HIGH : LOW;
        digitalWrite(DIR_PIN, dirSignal);
        delayMicroseconds(20);
        Serial.println("[DRIVE] Moving LEFT continuously. User determines stop.");
      }
      else if (command.equalsIgnoreCase("STOP")) {
        driveState = DRIVE_IDLE;
        Serial.printf("[STOP] Motion halted by user at position: %.2f cm (Steps: %ld)\n",
                      getDistanceCm(), currentStepPosition);
      }
      else if (command.startsWith("SPEED,")) {
        int pct = command.substring(6).toInt();
        setSpeedPercent(pct);
      }
      else if (command.startsWith("TARGET,")) {
        driveState = DRIVE_IDLE;
        float target = command.substring(7).toFloat();
        moveTo(target);
      }
      else if (command.startsWith("REL,")) {
        driveState = DRIVE_IDLE;
        float delta = command.substring(4).toFloat();
        moveRelative(delta);
      }
      else if (command.equalsIgnoreCase("ZERO")) {
        driveState = DRIVE_IDLE;
        currentStepPosition = 0;
        Serial.println("Zero reference reset. Current position is now 0.00 cm.");
      }
      else if (command.equalsIgnoreCase("STATUS")) {
        printStatus();
      }
      else if (command.startsWith("UP,")) {
        String idStr = command.substring(3);
        int targetId = idStr.toInt();
        Serial.printf("UP_CONFIRMED,%d\n", targetId);
      }
      else if (command.startsWith("DOWN,")) {
        String idStr = command.substring(5);
        if (idStr.equalsIgnoreCase("ALL")) {
          Serial.println("DOWN_CONFIRMED,ALL");
        } else {
          int targetId = idStr.toInt();
          Serial.printf("DOWN_CONFIRMED,%d\n", targetId);
        }
      }
      else if (command.equalsIgnoreCase("PING")) {
        Serial.println("OK: PING_ACK (MOTOR ESP32 RUNNING 115200 BAUD - IDLE)");
      }
      else {
        Serial.println("Commands: FWD (or RIGHT), REV (or LEFT), STOP, SPEED,<15-150>, UP,<1-7>, DOWN,<1-7|ALL>, TARGET,<cm>, REL,<cm>, ZERO, STATUS, PING");
      }
    }
  }

  // Execute continuous stepping if driveState is active (NO end parameters)
  if (driveState != DRIVE_IDLE) {
    // Safety watchdog: prevent motor runaway if dashboard or connection is lost
    if (continuousSteps == 0) {
      driveStartTime = millis();
    } else if (millis() - driveStartTime > 60000) {
      driveState = DRIVE_IDLE;
      Serial.println("\n[SAFETY WATCHDOG] Continuous drive reached 60s limit. Motor stopped.");
      return;
    }

    unsigned long stepDelay = maxSpeedDelayUs;
    if (continuousSteps < accelSteps) {
      stepDelay = startSpeedDelayUs - 
                  ((startSpeedDelayUs - maxSpeedDelayUs) * continuousSteps) / accelSteps;
    }

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay > 10 ? stepDelay - 10 : 10);

    if (driveState == DRIVE_RIGHT) {
      currentStepPosition++;
    } else {
      currentStepPosition--;
    }
    totalStepsTravelled++;
    continuousSteps++;
  }
}