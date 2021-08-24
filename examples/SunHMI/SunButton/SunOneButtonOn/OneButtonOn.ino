void setup() {
  Serial.begin(115200);
  pinMode(48,INPUT_PULLUP);
  while(digitalRead(48))
  {
    Serial.print("isRobotOn:");
    Serial.println(!digitalRead(48));
    }
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Sunnybot On");
  delay(1000);

}
