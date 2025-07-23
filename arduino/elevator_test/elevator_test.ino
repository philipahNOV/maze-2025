#include <Servo.h> // For servo kontroll

// Initialiserer variabler

namespace lift_servo {
    Servo lift; // Servo for heis
    const uint8_t servo_pin = 5; // Pinne for servo
    const uint8_t lift_down = 180; // Lav posisjon for heis
    const uint8_t lift_up = 0; // Høy posisjon for heis
    const uint8_t lift_stop = 90; // Stopp posisjon for heis
}

// Navnerom for globale variabler
namespace {
    // Teller for hvor ofte grensen sjekkes for aktuatorene
    uint8_t elevator_start_time = 0; // Tidspunkt for når heisen startet å kjøre
    bool elevator_running = false; // Variabel for å sjekke om heisen er aktiv
    bool elevator_attached = false; // Variabel for å sjekke om heisen er festet
	uint8_t tid_mellom = 0; // Tidspunkt for når heisen startet å kjøre
	uint8_t teller = 0; // Teller for hvor mange ganger heisen har kjørt
}

// Initialiserer funksjoner
void elevator(int8_t elevator_dir); // Funksjon for å kontrollere heisen for å hente ball

void setup() {
    // Starter seriel kominikasjon
    Serial.begin(115200);

    // Initialiserer heis servo
    lift_servo::lift.attach(lift_servo::servo_pin); // Fester servoen til pinnen
    lift_servo::lift.write(0); // Setter heisen til lav posisjon
    lift_servo::lift.detach(); // Frakobler servoen
}

void loop() {

	if (millis() - tid_mellom >= 1000 && teller == 0) // Sjekker heisen hver 10. loop
	{
		teller = 1; // Setter teller til 1 for å indikere at heisen har kjørt en gang
		tid_mellom = millis(); // Oppdaterer tidspunktet for når heisen startet å kjøre
		elevator(1); // Kjører heisen oppover
	}
	else if (millis() - tid_mellom >= 1000 && teller == 1) // Sjekker heisen hver 10. loop
	{
		teller = 0; // Setter teller til 1 for å indikere at heisen har kjørt en gang
		tid_mellom = millis(); // Oppdaterer tidspunktet for når heisen startet å kjøre
		elevator(-1); // Kjører heisen oppover
	}


	if (elevator_running) // Sjekker om heisen er aktiv
	{
		if (millis() - elevator_start_time >= 200) 
		{
			elevator(0);
		}
	}
}

void elevator(int8_t elevator_dir)
{
    // Hvis en bevegelse er i gang OG den nye kommandoen også er en bevegelse, ignorer den.
    if (elevator_running && elevator_dir != 0) {
        return;
    }

    if (elevator_dir == -1) {  // Move down
        elevator_running = true; // Setter heisen til aktiv
        elevator_start_time = millis(); // Setter tidspunktet for når heisen startet å kjøre
        if (!elevator_attached) {
            lift_servo::lift.attach(lift_servo::servo_pin);
            elevator_attached = true;
        }
        lift_servo::lift.write(lift_servo::lift_down);
    }
    else if (elevator_dir == 1) {  // Move up
        elevator_running = true; // Setter heisen til aktiv
        elevator_start_time = millis(); // Setter tidspunktet for når heisen startet å kjøre
        if (!elevator_attached) {
            lift_servo::lift.attach(lift_servo::servo_pin);
            elevator_attached = true;
        }
        lift_servo::lift.write(lift_servo::lift_up);
    }
    else {  // elevator_dir == 0 → stop and optionally detach
        if (elevator_attached) {
            lift_servo::lift.write(lift_servo::lift_stop);  // optional: neutral hold position
            lift_servo::lift.detach();
            elevator_attached = false; // Frakobler servoen når heisen er stoppet
            elevator_running = false; // Setter heisen til inaktiv
        }
    }
}