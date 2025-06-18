void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() >= 16) {
    byte mottattBytes[16];

    // Les alle innkommende bytes på en gang inn i en buffer ('mottattBytes')
    Serial.readBytes(mottattBytes, 16);

    // Send hele bufferet tilbake til Jetson Nano samtidig
    Serial.write(mottattBytes, 16);
  }
}