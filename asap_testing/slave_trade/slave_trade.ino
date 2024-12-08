void setup() {
  Serial.begin(9600);  // Start communication with HC-05 on pins 0 (RX) and 1 (TX)
  Serial.println("Slave Ready");
}

void loop() {
  if (Serial.available()) {                 // Check if data is available from HC-05
    String received = Serial.readString();  // Read the incoming data
    received = received.toInt();
    Serial.print("Received: ");
    Serial.println(received);  // Print the received data to Serial Monitor
  } else {
    Serial.println("failed");
  }
}
