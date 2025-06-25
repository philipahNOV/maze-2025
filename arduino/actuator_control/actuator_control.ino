// Inkluderer nødvendige biblioteker
#include <Servo.h> // For servo kontroll

// Initialiserer variabler
// Navnerom for aktuatorene
namespace actuators {
    
    // Struct for pinnenummerene
    struct ActuatorData
    {
        const uint8_t pwm_up; // Pinne for PWM oppover
        const uint8_t pwm_down; // Pinne for PWM nedover
        const uint8_t pot_feedback; // Pinne for potensjometer
        float position; // Posisjon til aktuatoren (mm)
        int8_t distance_status; // Motor status. -1 er under nedre grense, 1 er over øvre grense
    };

    // Aktuator en
    ActuatorData actuator_1 = {9, 10, A4, 0, 0}; // PWM oppover = 9, PWM nedover = 10, Potensjometer = A4

    // Aktuator to
    ActuatorData actuator_2 = {11, 12, A3, 0, 0}; // PWM oppover = 11, PWM nedover = 12, Potensjometer = A3
}

namespace lift_servo {
    Servo lift; // Servo for heis
    const uint8_t servo_pin = 2; // Pinne for servo
    const uint8_t lift_down = 180; // Lav posisjon for heis
    const uint8_t lift_up = 0; // Høy posisjon for heis
    const uint8_t lift_stop = 90; // Stopp posisjon for heis
}

// Navnerom for globale variabler
namespace {
    // Struct for hastigheter til aktuatorene TODO: Flytt til ett felt i ActuatorData
    struct Speeds
    {
        int16_t speed_actuator_1; // Fart for aktuator en
        int16_t speed_actuator_2; // Fart for aktuator to
    };

    Speeds actuator_speeds = {0, 0}; // Initialiserer hastighetene for aktuatorene
    uint8_t limit_check_counter = 0; // Teller hvor ofte grensen sjekkes for aktuatorene
}

// Initialiserer funksjoner
void actuator_position(); // Funksjon for posisjonen til aktuatoren
void actuator_limit_check(); // Funksjon for å sjekke om aktuatoren er over eller under ønsket maks eller min høyde
void move_speed(); // Funksjon for å bevege begge aktuatorene med hastighetene i actuator_speeds
void read_serial(); // Funksjon for å lese innkommende seriedata

void setup() {
    // Starter seriel kominikasjon
    Serial.begin(9600);

    // Setter pin modusene
    // Aktuator en
    pinMode(actuators::actuator_1.pwm_up, OUTPUT); // Setter PWM pinne for aktuator en oppover
    pinMode(actuators::actuator_1.pwm_down, OUTPUT); // Setter PWM pinne for aktuator en nedover

    // Aktuator to
    pinMode(actuators::actuator_2.pwm_up, OUTPUT); // Setter PWM pinne for aktuator to oppover
    pinMode(actuators::actuator_2.pwm_down, OUTPUT); // Setter PWM pinne for aktuator to nedover

    // Setter analog pinnene til null for at motorene ikke bevege seg når programmet starter
    analogWrite(actuators::actuator_1.pwm_up, 0); // Setter PWM pinne for aktuator en oppover til 0
    analogWrite(actuators::actuator_1.pwm_down, 0); // Setter PWM pinne for aktuator en nedover til 0
    analogWrite(actuators::actuator_2.pwm_up, 0); // Setter PWM pinne for aktuator to oppover til 0
    analogWrite(actuators::actuator_2.pwm_down, 0); // Setter PWM pinne for aktuator to nedover til 0

    // Initialiserer heis servo
    lift_servo::lift.attach(lift_servo::servo_pin); // Fester servoen til pinnen
    lift_servo::lift.write(0); // Setter heisen til lav posisjon
    lift_servo::lift.detach(); // Frakobler servoen
}

void loop() {
    actuator_limit_check(); // Sjekker om aktuatorene er over eller under grensen og oppdaterer distance_status
    read_serial(); // Motar data over seriel fra Jetson
    move_speed(); // Setter motor hastighetene
}

// Funksjon for å returnere posisjonen til aktuatoren
void actuator_position() // Tar inn en peker til potensjometer pinnen
{
    const float act_max_stroke{50.0}; // Maks slaglengde for aktuatorene (mm)
    const float adc_max_hight{1018.0}; // ADC verdien for maks lengde
    const float adc_min_hight{2.0}; // ADC verdi for minimum lengde

    // Kalkulerer distansen til aktuator en
    actuators::actuator_1.position = (float(analogRead(actuators::actuator_1.pot_feedback)) - adc_min_hight) * act_max_stroke / (adc_max_hight - adc_min_hight);
    // Kalkulerer distansen til aktuator to
    actuators::actuator_2.position = (float(analogRead(actuators::actuator_2.pot_feedback)) - adc_min_hight) * act_max_stroke / (adc_max_hight - adc_min_hight);
}

// Funksjon for å sjekke om aktuatorene er over eller under grensen
void actuator_limit_check()
{   
    limit_check_counter++; // Øker telleren for hvor ofte grensen sjekkes
    if (limit_check_counter < 10) // Sjekker ikke grensen for ofte
    {
        return; // Ikke sjekk grensen
    }
    else
    {
        limit_check_counter = 0; // Nullstiller telleren
    }

    const float act_max_stroke{50.0}; // Maks slaglengde for aktuatorene (mm)
    const float offset{3.0}; // Distansen fra toppen og bunden hvor aktuatorene ikke skal gå innenfor (mm)

    actuator_position(); // Leser inn posisjonen til aktuatorene

    // Sjekker aktuator en
    if (actuators::actuator_1.position >= act_max_stroke - offset) // Hvis aktuator en
    {
        actuators::actuator_1.distance_status = 1; // Setter status til 1 (for høyt)
    }
    else if (actuators::actuator_1.position <= offset) // Hvis aktuator en er for lavt
    {
        actuators::actuator_1.distance_status = -1; // Setter status til -1 (for lavt)
    }
    else // Hvis aktuator en er innenfor grensen
    {
        actuators::actuator_1.distance_status = 0; // Setter status til 0 (OK)
    }

    // Sjekker aktuator to
    if (actuators::actuator_2.position >= act_max_stroke - offset) // Hvis aktuator to er for høyt
    {
        actuators::actuator_2.distance_status = 1; // Setter status til 1 (for høyt)
    }
    else if (actuators::actuator_2.position <= offset) // Hvis aktuator to er for lavt
    {
        actuators::actuator_2.distance_status = -1; // Setter status til -1 (for lavt)
    }
    else // Hvis aktuator to er innenfor grensen
    {
        actuators::actuator_2.distance_status = 0; // Setter status til 0 (OK)
    }
}

// Funksjon for å sette hastighetene til aktuatorene basert på actuator_speeds
void move_speed()
{
    // Setter hastighetene for aktuatorene fra actuator_speeds
    // Aktuator 1
    if (actuator_speeds.speed_actuator_1 > 0 && actuators::actuator_1.distance_status != 1) // Hvis aktuator en skal bevege seg oppover og ikke er for høyt
    {
        analogWrite(actuators::actuator_1.pwm_up, actuator_speeds.speed_actuator_1);
        analogWrite(actuators::actuator_1.pwm_down, 0);
    } 
    else if (actuator_speeds.speed_actuator_1 < 0 && actuators::actuator_1.distance_status != -1) // Hvis aktuator en skal bevege seg nedover og ikke er for lavt
    {
        analogWrite(actuators::actuator_1.pwm_up, 0);
        analogWrite(actuators::actuator_1.pwm_down, -actuator_speeds.speed_actuator_1); // - for å få riktig retning (-255 er 255 i PWM på negativ pinne)
    }
    else // Hvis aktuator en ikke skal bevege seg
    {
        analogWrite(actuators::actuator_1.pwm_up, 0);
        analogWrite(actuators::actuator_1.pwm_down, 0);
    }
    
    // Aktuator 2
    if (actuator_speeds.speed_actuator_2 > 0 && actuators::actuator_2.distance_status != 1) // Hvis aktuator to skal bevege seg oppover og ikke er for høyt
    {
        analogWrite(actuators::actuator_2.pwm_up, actuator_speeds.speed_actuator_2);
        analogWrite(actuators::actuator_2.pwm_down, 0);
    } 
    else if (actuator_speeds.speed_actuator_2 < 0 && actuators::actuator_2.distance_status != -1) // Hvis aktuator to skal bevege seg nedover og ikke er for lavt
    {
        analogWrite(actuators::actuator_2.pwm_up, 0);
        analogWrite(actuators::actuator_2.pwm_down, -actuator_speeds.speed_actuator_2); // - for å få riktig retning (-255 er 255 i PWM på negativ pinne)
    }
    else // Hvis aktuator to ikke skal bevege seg
    {
        analogWrite(actuators::actuator_2.pwm_up, 0);
        analogWrite(actuators::actuator_2.pwm_down, 0);
    }
}

void get_ball()
{
    // Funksjon for å kontrollere heisen
    lift_servo::lift.attach(lift_servo::servo_pin); // Fester servoen til pinnen
    lift_servo::lift.write(lift_servo::lift_down); // Setter heisen til lav posisjon
    delay(100); // Venter 100 ms for at heisen skal nå ned
    lift_servo::lift.write(lift_servo::lift_stop); // Setter heisen til høy posisjon
    delay(1000); // Venter 1 sekund for at baller kan renne på heisen
    lift_servo::lift.write(lift_servo::lift_up); // Setter heisen
    delay(100); // Venter 100 ms for at heisen skal nå opp
    lift_servo::lift.detach(); // Frakobler servoen
}

// Funksjon for å lese innkommende seriedata
void read_serial()
{
    if (Serial.available() >= 3)
    {
        char buffer[64]; // Buffer for å lese innkommende data
        int len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1); // Leser inn data til linjeskift eller buffer er fullt
        buffer[len] = '\0'; // Null-terminer strengen
        
        int speed1, speed2; // Variabler for å lagre hastigheter og checksum
        
        // Leser inn hastigheter fra buffer
        if (sscanf(buffer, "%d,%d", &speed1, &speed2) == 2) 
        {
            actuator_speeds.speed_actuator_1 = constrain(speed1, -255, 255);
            actuator_speeds.speed_actuator_2 = constrain(speed2, -255, 255);
        }
    }
}
