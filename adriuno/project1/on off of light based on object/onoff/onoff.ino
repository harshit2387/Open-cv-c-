void setup() {
  pinMode(13, OUTPUT); // Use built-in LED for demonstration
  Serial.begin(9600);  // Start serial communication
}

void loop() {
  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == '1') {
      digitalWrite(13, HIGH); // Turn light on
    } else if (command == '0') {
      digitalWrite(13, LOW);  // Turn light off
    }
  }
}
