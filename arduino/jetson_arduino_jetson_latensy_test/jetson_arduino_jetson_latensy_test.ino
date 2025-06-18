void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // Vent til det er minst 4 bytes tilgjengelig
  if (Serial.available() >= 4) {
    unsigned long tall_mottatt = 0;

    // Les inn bytes og rekonstruer tallet
    for (int i = 0; i < 4; i++) {
      tall_mottatt = (tall_mottatt << 8) | Serial.read();
    }

    // Send det samme tallet tilbake
    for (int i = 3; i >= 0; i--) {
      Serial.write((tall_mottatt >> (i * 8)) & 0xFF);
    }
  }
}
