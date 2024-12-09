void setup() {
  Serial.begin(9600);  // Ensure baud rate matches HC-05
  Serial.println("Waiting for data...");
}

void loop() {
  if (Serial.available()) {
    String received = Serial.readString();  // Read incoming character
    Serial.print("Received: ");
    Serial.println(received);       // Print received character
  }
}