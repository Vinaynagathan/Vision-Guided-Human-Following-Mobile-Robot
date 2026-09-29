/*
 * Vision-Guided Human-Following Mobile Robot
 * Arduino Mega 2560 controller
 *
 * Based on the project paper:
 * - USB serial from laptop at 9600 bps
 * - Commands: L, F, R, B, S
 * - HC-SR04 obstacle safety override
 * - HC-05 on Serial1 for manual override
 * - 10 cm stop threshold, 15 cm resume threshold
 * - ~100 Hz main control loop
 *
 * NOTE:
 * The paper does not specify exact Arduino pin numbers or relay
 * active-high/active-low behavior. The pin map below is therefore an
 * implementation choice and should be changed to match the actual wiring.
 */

const uint8_t LEFT_FWD_PIN  = 22;
const uint8_t LEFT_REV_PIN  = 23;
const uint8_t RIGHT_FWD_PIN = 24;
const uint8_t RIGHT_REV_PIN = 25;

const uint8_t TRIG_PIN = 30;
const uint8_t ECHO_PIN = 31;

// Change to false if your relay board is active LOW.
const bool RELAY_ACTIVE_HIGH = true;

const unsigned long SERIAL_BAUD = 9600;
const unsigned long CONTROL_PERIOD_MS = 10;     // 100 Hz
const unsigned long ULTRASONIC_PERIOD_MS = 100; // 10 Hz
const unsigned long SERIAL_TIMEOUT_MS = 5000;  // stop after 5 s without laptop command

const float STOP_DISTANCE_CM = 10.0;
const float RESUME_DISTANCE_CM = 15.0;

char visionCommand = 'S';
char manualCommand = 'S';

unsigned long lastVisionCommandMs = 0;
unsigned long lastControlMs = 0;
unsigned long lastUltrasonicMs = 0;

float obstacleDistanceCm = 400.0;
bool obstacleLock = false;

enum RobotState {
  STATE_STOP,
  STATE_FORWARD,
  STATE_LEFT,
  STATE_RIGHT,
  STATE_BACKWARD
};

RobotState currentState = STATE_STOP;

void relayWrite(uint8_t pin, bool on) {
  bool output = RELAY_ACTIVE_HIGH ? on : !on;
  digitalWrite(pin, output ? HIGH : LOW);
}

void allRelaysOff() {
  relayWrite(LEFT_FWD_PIN, false);
  relayWrite(LEFT_REV_PIN, false);
  relayWrite(RIGHT_FWD_PIN, false);
  relayWrite(RIGHT_REV_PIN, false);
}

void stopMotors() {
  allRelaysOff();
  currentState = STATE_STOP;
}

/*
 * Differential-drive relay control.
 * Exact relay polarity/wiring is hardware dependent.
 */
void moveForward() {
  allRelaysOff();
  relayWrite(LEFT_FWD_PIN, true);
  relayWrite(RIGHT_FWD_PIN, true);
  currentState = STATE_FORWARD;
}

void rotateLeft() {
  allRelaysOff();
  relayWrite(LEFT_REV_PIN, true);
  relayWrite(RIGHT_FWD_PIN, true);
  currentState = STATE_LEFT;
}

void rotateRight() {
  allRelaysOff();
  relayWrite(LEFT_FWD_PIN, true);
  relayWrite(RIGHT_REV_PIN, true);
  currentState = STATE_RIGHT;
}

void moveBackward() {
  allRelaysOff();
  relayWrite(LEFT_REV_PIN, true);
  relayWrite(RIGHT_REV_PIN, true);
  currentState = STATE_BACKWARD;
}

void executeCommand(char cmd) {
  switch (cmd) {
    case 'F': moveForward();  break;
    case 'L': rotateLeft();   break;
    case 'R': rotateRight();  break;
    case 'B': moveBackward(); break;
    case 'S':
    default:  stopMotors();   break;
  }
}

void readVisionSerial() {
  while (Serial.available() > 0) {
    char c = toupper(Serial.read());

    if (c == 'L' || c == 'F' || c == 'R' || c == 'S' || c == 'B') {
      visionCommand = c;
      lastVisionCommandMs = millis();
    }
  }
}

void readBluetooth() {
  while (Serial1.available() > 0) {
    char c = toupper(Serial1.read());

    /*
     * The paper specifies Bluetooth manual override primarily for
     * backward motion during obstacle recovery. Supporting L/F/R/S
     * here as well is useful for testing, while B is the key override.
     */
    if (c == 'L' || c == 'F' || c == 'R' || c == 'S' || c == 'B') {
      manualCommand = c;
    }
  }
}

float readUltrasonicCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);

  if (duration == 0) {
    return 400.0; // Treat timeout as "no nearby obstacle".
  }

  return (duration * 0.0343f) / 2.0f;
}

void updateObstacleState() {
  obstacleDistanceCm = readUltrasonicCm();

  if (!obstacleLock && obstacleDistanceCm < STOP_DISTANCE_CM) {
    obstacleLock = true;
  }

  if (obstacleLock && obstacleDistanceCm > RESUME_DISTANCE_CM) {
    obstacleLock = false;
  }
}

void setup() {
  pinMode(LEFT_FWD_PIN, OUTPUT);
  pinMode(LEFT_REV_PIN, OUTPUT);
  pinMode(RIGHT_FWD_PIN, OUTPUT);
  pinMode(RIGHT_REV_PIN, OUTPUT);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  allRelaysOff();

  Serial.begin(SERIAL_BAUD);
  Serial1.begin(SERIAL_BAUD);

  lastVisionCommandMs = millis();

  Serial.println("Human-following robot controller ready.");
}

void loop() {
  unsigned long now = millis();

  // Laptop -> Arduino command input
  readVisionSerial();

  // Bluetooth manual override input
  readBluetooth();

  // Ultrasonic sensing at approximately 10 Hz
  if (now - lastUltrasonicMs >= ULTRASONIC_PERIOD_MS) {
    lastUltrasonicMs = now;
    updateObstacleState();
  }

  // Main control loop at approximately 100 Hz
  if (now - lastControlMs >= CONTROL_PERIOD_MS) {
    lastControlMs = now;

    /*
     * Safety override:
     * If obstacle < 10 cm -> immediate STOP.
     * Vision commands are ignored until obstacle > 15 cm.
     */
    if (obstacleLock) {
      stopMotors();

      /*
       * Manual backward command is allowed for obstacle recovery,
       * matching the paper's Bluetooth override concept.
       */
      if (manualCommand == 'B' && obstacleDistanceCm >= STOP_DISTANCE_CM) {
        moveBackward();
      }
    }
    else {
      // Manual override is given priority when a Bluetooth command exists.
      if (manualCommand != 'S') {
        executeCommand(manualCommand);

        // Manual command is one-shot unless another command is received.
        manualCommand = 'S';
      }
      else if (now - lastVisionCommandMs > SERIAL_TIMEOUT_MS) {
        // Communication-loss safety behavior described in the paper.
        stopMotors();
      }
      else {
        executeCommand(visionCommand);
      }
    }
  }
}
