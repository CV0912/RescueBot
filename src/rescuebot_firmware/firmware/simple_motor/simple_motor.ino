// Motor Pins 
const int FRONT_RIGHT_DIR_PIN     = 7;
const int FRONT_RIGHT_PWM_CHANNEL = 9;   
const int FRONT_LEFT_DIR_PIN      = 8;
const int FRONT_LEFT_PWM_CHANNEL  = 10;  
const int BACK_RIGHT_DIR_PIN      = 11;
const int BACK_RIGHT_PWM_CHANNEL  = 5;   
const int BACK_LEFT_DIR_PIN       = 12;
const int BACK_LEFT_PWM_CHANNEL   = 6;

double cmd = 0.0;

void setup() {
  pinMode(FRONT_RIGHT_DIR_PIN, OUTPUT);
  pinMode(FRONT_LEFT_DIR_PIN, OUTPUT);
  pinMode(BACK_RIGHT_DIR_PIN, OUTPUT);
  pinMode(BACK_LEFT_DIR_PIN, OUTPUT);
  pinMode(FRONT_RIGHT_PWM_CHANNEL, OUTPUT);
  pinMode(FRONT_LEFT_PWM_CHANNEL, OUTPUT);
  pinMode(BACK_RIGHT_PWM_CHANNEL, OUTPUT);
  pinMode(BACK_LEFT_PWM_CHANNEL, OUTPUT);

  digitalWrite(FRONT_RIGHT_DIR_PIN, HIGH);
  digitalWrite(FRONT_LEFT_DIR_PIN, HIGH);
  digitalWrite(BACK_RIGHT_DIR_PIN, HIGH);
  digitalWrite(BACK_LEFT_DIR_PIN, HIGH);

  Serial.begin(115200);

}

void loop() {
  if(Serial.available()){
    cmd = Serial.readString().toDouble();
  }

  analogWrite(FRONT_RIGHT_PWM_CHANNEL, cmd*100);
  analogWrite(FRONT_LEFT_PWM_CHANNEL, cmd*100);
  analogWrite(BACK_RIGHT_PWM_CHANNEL, cmd*100);
  analogWrite(BACK_LEFT_PWM_CHANNEL, cmd*100);
}