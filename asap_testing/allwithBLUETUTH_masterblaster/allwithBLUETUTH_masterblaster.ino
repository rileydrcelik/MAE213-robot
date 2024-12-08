void setup() {
  Serial.begin(9600); // Start communication with HC-05 on pins 0 (RX) and 1 (TX)
}

void loop() {
  Serial.println("Hello from your daddy"); // Send data to the Slave
  delay(1000); // Send every second
}
