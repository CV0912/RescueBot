// ============================================================
//         SERIAL PROTOCOL (matches MecanumInterface.cpp)
// ============================================================
// Incoming (PC -> Arduino), NO newline, 4 tokens back-to-back:
//   fr<p|n><val>,fl<p|n><val>,br<p|n><val>,bl<p|n><val>,
//   e.g. frp12.34,fln05.67,brp00.00,blp00.00,
// These are VELOCITY COMMANDS (rad/s) coming from ros2_control.
//
// Outgoing (Arduino -> PC), newline-terminated (PC uses ReadLine):
//   same token format, built from MEASURED wheel velocities.
//
// Wheel order fixed to match the C++ side: FR, FL, BR, BL.


// ============================================================
//                         MOTOR PINS
// ============================================================

// Front Right
const int FRONT_RIGHT_DIR_PIN     = 22;
const int FRONT_RIGHT_PWM_CHANNEL = 6;

// Front Left
const int FRONT_LEFT_DIR_PIN      = 24;
const int FRONT_LEFT_PWM_CHANNEL  = 5;

// Back Right
const int BACK_RIGHT_DIR_PIN      = 26;
const int BACK_RIGHT_PWM_CHANNEL  = 4;

// Back Left
const int BACK_LEFT_DIR_PIN       = 28;
const int BACK_LEFT_PWM_CHANNEL   = 7;


// ============================================================
//                        ENCODER PINS
// ============================================================

const int FRONT_RIGHT_ENCODER_A = 3;
const int FRONT_RIGHT_ENCODER_B = 38;

const int FRONT_LEFT_ENCODER_A = 18;
const int FRONT_LEFT_ENCODER_B = 34;

const int BACK_RIGHT_ENCODER_A = 19;
const int BACK_RIGHT_ENCODER_B = 32;

const int BACK_LEFT_ENCODER_A = 2;
const int BACK_LEFT_ENCODER_B = 36;


// ============================================================
//                      ENCODER COUNTERS
// ============================================================

volatile long front_right_encoder_counter = 0;
volatile long front_left_encoder_counter  = 0;
volatile long back_right_encoder_counter  = 0;
volatile long back_left_encoder_counter   = 0;


// ============================================================
//                MEASURED / COMMANDED VELOCITIES
// ============================================================
// Index order: 0=FR, 1=FL, 2=BR, 3=BL  (matches C++ NUM_WHEELS order)

double measured_velocity[4]  = { 0.0, 0.0, 0.0, 0.0 };  // rad/s, from encoders
double commanded_velocity[4] = { 0.0, 0.0, 0.0, 0.0 };  // rad/s, from ROS2

const char* WHEEL_PREFIX[4] = { "fr", "fl", "br", "bl" };


// ============================================================
//              OPEN-LOOP VELOCITY -> PWM (placeholder)
// ============================================================
// TODO: replace with a real per-wheel velocity PID.
// For now: |commanded velocity| is linearly mapped to 0-100 PWM
// using this assumed max speed. Tune to your motor/gearbox.

const double MAX_WHEEL_VELOCITY_RADPS = 10.0;


// ============================================================
//                    SERIAL INPUT PARSING
// ============================================================

String serial_input_buffer = "";

int wheelIndexFromPrefix(const String &prefix)
{
  if (prefix == "fr") return 0;
  if (prefix == "fl") return 1;
  if (prefix == "br") return 2;
  if (prefix == "bl") return 3;
  return -1;
}

void processToken(const String &token)
{
  if (token.length() < 4) return;  // too short to contain prefix+sign+value

  String prefix = token.substring(0, 2);
  char sign_char = token.charAt(2);
  int wheel = wheelIndexFromPrefix(prefix);

  if (wheel < 0) return;  // unrecognised wheel id, ignore

  double value = token.substring(3).toFloat();
  commanded_velocity[wheel] = (sign_char == 'n') ? -value : value;
}

// Tokens arrive back-to-back with NO newline, each ending in ',' -
// so we split on ',' rather than waiting for a line terminator.
void readSerialVelocityCommands()
{
  while (Serial.available() > 0)
  {
    char incoming_char = (char)Serial.read();

    if (incoming_char == ',')
    {
      processToken(serial_input_buffer);
      serial_input_buffer = "";
    }
    else if (incoming_char == '\n' || incoming_char == '\r')
    {
      // Ignore stray newlines/carriage returns if any show up.
    }
    else
    {
      serial_input_buffer += incoming_char;
    }
  }
}


// ============================================================
//                   SERIAL OUTPUT FORMATTING
// ============================================================
// Builds one token: <prefix><p|n><zero-padded value with 2 decimals>,

String formatWheelToken(const char *prefix, double value)
{
  char sign = (value >= 0.0) ? 'p' : 'n';
  double magnitude = fabs(value);

  char mag_buf[10];
  dtostrf(magnitude, 0, 2, mag_buf);  // e.g. "5.67" or "12.34"

  String zero_pad = (magnitude < 10.0) ? "0" : "";

  String token = String(prefix) + sign + zero_pad + mag_buf + ",";
  return token;
}

void sendMeasuredVelocities()
{
  String message = "";
  for (int i = 0; i < 4; i++)
  {
    message += formatWheelToken(WHEEL_PREFIX[i], measured_velocity[i]);
  }
  Serial.println(message);  // newline required: PC side uses ReadLine()
}


// ============================================================
//                          SETUP
// ============================================================

void setup()
{
  // ---------------- MOTOR PINS ----------------
  pinMode(FRONT_RIGHT_DIR_PIN, OUTPUT);
  pinMode(FRONT_LEFT_DIR_PIN, OUTPUT);
  pinMode(BACK_RIGHT_DIR_PIN, OUTPUT);
  pinMode(BACK_LEFT_DIR_PIN, OUTPUT);

  pinMode(FRONT_RIGHT_PWM_CHANNEL, OUTPUT);
  pinMode(FRONT_LEFT_PWM_CHANNEL, OUTPUT);
  pinMode(BACK_RIGHT_PWM_CHANNEL, OUTPUT);
  pinMode(BACK_LEFT_PWM_CHANNEL, OUTPUT);

  // ---------------- ENCODER PINS ----------------
  pinMode(FRONT_RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(FRONT_RIGHT_ENCODER_B, INPUT_PULLUP);

  pinMode(FRONT_LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(FRONT_LEFT_ENCODER_B, INPUT_PULLUP);

  pinMode(BACK_RIGHT_ENCODER_A, INPUT_PULLUP);
  pinMode(BACK_RIGHT_ENCODER_B, INPUT_PULLUP);

  pinMode(BACK_LEFT_ENCODER_A, INPUT_PULLUP);
  pinMode(BACK_LEFT_ENCODER_B, INPUT_PULLUP);

  // ---------------- INTERRUPTS ----------------
  attachInterrupt(digitalPinToInterrupt(FRONT_RIGHT_ENCODER_A), frontRightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(FRONT_LEFT_ENCODER_A),  frontLeftEncoderCallback,  RISING);
  attachInterrupt(digitalPinToInterrupt(BACK_RIGHT_ENCODER_A),  backRightEncoderCallback,  RISING);
  attachInterrupt(digitalPinToInterrupt(BACK_LEFT_ENCODER_A),   backLeftEncoderCallback,   RISING);

  // ---------------- SERIAL ----------------
  Serial.begin(115200);
  serial_input_buffer.reserve(16);
}


// ============================================================
//                           LOOP
// ============================================================

void loop()
{
  // ==========================================================
  //           READ VELOCITY COMMANDS FROM ROS2
  // ==========================================================
  readSerialVelocityCommands();

  // ==========================================================
  //     OPEN-LOOP: COMMANDED VELOCITY -> DIRECTION + PWM
  //     (placeholder until a real velocity PID is added)
  // ==========================================================

  applyWheelCommand(FRONT_RIGHT_DIR_PIN, FRONT_RIGHT_PWM_CHANNEL, commanded_velocity[0]);
  applyWheelCommand(FRONT_LEFT_DIR_PIN,  FRONT_LEFT_PWM_CHANNEL,  commanded_velocity[1]);
  applyWheelCommand(BACK_RIGHT_DIR_PIN,  BACK_RIGHT_PWM_CHANNEL,  commanded_velocity[2]);
  applyWheelCommand(BACK_LEFT_DIR_PIN,   BACK_LEFT_PWM_CHANNEL,   commanded_velocity[3]);

  // ==========================================================
  //                   VELOCITY CALCULATION
  // ==========================================================
  // counts *10 (100ms window -> Hz) * (60/385 rev/min per count) * 0.10472 (rpm->rad/s)

  measured_velocity[0] = 10 * front_right_encoder_counter * (60.0 / 385.0) * 0.10472;
  measured_velocity[1] = 10 * front_left_encoder_counter  * (60.0 / 385.0) * 0.10472;
  measured_velocity[2] = 10 * back_right_encoder_counter  * (60.0 / 385.0) * 0.10472;
  measured_velocity[3] = 10 * back_left_encoder_counter   * (60.0 / 385.0) * 0.10472;

  // ==========================================================
  //                    SERIAL OUTPUT
  // ==========================================================
  sendMeasuredVelocities();

  // ==========================================================
  //                 RESET ENCODER COUNTERS
  // ==========================================================
  front_right_encoder_counter = 0;
  front_left_encoder_counter  = 0;
  back_right_encoder_counter  = 0;
  back_left_encoder_counter   = 0;

  // ==========================================================
  //                    100 ms INTERVAL
  // ==========================================================
  delay(100);
}


// ============================================================
//   Sets direction pin + PWM magnitude for one wheel, open-loop
// ============================================================

void applyWheelCommand(int dir_pin, int pwm_channel, double velocity_command)
{
  digitalWrite(dir_pin, (velocity_command >= 0.0) ? HIGH : LOW);

  double fraction = fabs(velocity_command) / MAX_WHEEL_VELOCITY_RADPS;
  int pwm_value = (int)round(fraction * 100.0);
  pwm_value = constrain(pwm_value, 0, 100);

  analogWrite(pwm_channel, pwm_value);
}


// ============================================================
//                     ENCODER CALLBACKS
// ============================================================

void frontRightEncoderCallback()
{
  if (digitalRead(FRONT_RIGHT_ENCODER_B)) front_right_encoder_counter++;
  else front_right_encoder_counter--;
}

void frontLeftEncoderCallback()
{
  if (digitalRead(FRONT_LEFT_ENCODER_B)) front_left_encoder_counter++;
  else front_left_encoder_counter--;
}

void backRightEncoderCallback()
{
  if (digitalRead(BACK_RIGHT_ENCODER_B)) back_right_encoder_counter++;
  else back_right_encoder_counter--;
}

void backLeftEncoderCallback()
{
  if (digitalRead(BACK_LEFT_ENCODER_B)) back_left_encoder_counter++;
  else back_left_encoder_counter--;
}