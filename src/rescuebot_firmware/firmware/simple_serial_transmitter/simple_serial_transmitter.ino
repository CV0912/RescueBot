int x = 0;

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.println(x);
  x++;
  delay(1000);
}