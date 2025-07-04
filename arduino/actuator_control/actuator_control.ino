// Inkluderer nødvendige biblioteker
#include <Servo.h> // For servo kontroll
#include <Adafruit_NeoPixel.h> // For styring av NeoPixel LED

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

namespace led_strips {
    const uint8_t led_pin = 30; // Pinne for LED strip
    const uint16_t num_leds = 10; // Antall LED i stripen
    Adafruit_NeoPixel strip = Adafruit_NeoPixel(num_leds, led_pin, NEO_GRB + NEO_KHZ800); // Initialiserer LED stripen
}

namespace serial_messages {
    // Enum for tilstander i programmet
    enum class State
    {
        IDLE, // Tilstand for å vente på kommandoer
        GET_BALL, // Tilstand for å hente ball
        CONTROL, // Tilstand for å kontrollere aktuatorene
        SET_COLOR // Tilstand for å sette farge på LED stripen
    };

    State state = State::IDLE; // Initialiserer tilstanden til IDLE

    // Variabler for seriedata
    int16_t value_1 = 0; // Variabel for første verdi i seriedata
    int16_t value_2 = 0; // Variabel for andre verdi i seriedata
    int16_t value_3 = 0; // Variabel for tredje verdi i seriedata
}

// Navnerom for globale variabler
namespace {
    // Teller for hvor ofte grensen sjekkes for aktuatorene
    uint8_t limit_check_counter = 0;
}

// Initialiserer funksjoner
void stop_actuators(); // Funksjon for å stoppe aktuatorene
void actuator_position(); // Funksjon for posisjonen til aktuatoren
void actuator_limit_check(); // Funksjon for å sjekke om aktuatoren er over eller under ønsket maks eller min høyde
void move_speed(int16_t speed_actuator_1, int16_t speed_actuator_2); // Funksjon for å bevege begge aktuatorene med hastighetene i actuator_speeds
void get_ball(); // Funksjon for å kontrollere heisen for å hente ball
void set_led_color(uint8_t r, uint8_t g, uint8_t b); // Funksjon for å sette fargen på LED stripen
void read_serial(); // Funksjon for å lese innkommende seriedata

void setup() {
    // Starter seriel kominikasjon
    Serial.begin(9600);

    // Initialiserer LED stripen
    led_strips::strip.begin(); // Starter LED stripen
    led_strips::strip.setBrightness(255); // Setter lysstyrken til stripen
    // Setter alle LED til hvit farge
    set_led_color(255, 255, 255); // Setter fargen til hvit

    // Setter pin modusene
    // Aktuator en
    pinMode(actuators::actuator_1.pwm_up, OUTPUT); // Setter PWM pinne for aktuator en oppover
    pinMode(actuators::actuator_1.pwm_down, OUTPUT); // Setter PWM pinne for aktuator en nedover

    // Aktuator to
    pinMode(actuators::actuator_2.pwm_up, OUTPUT); // Setter PWM pinne for aktuator to oppover
    pinMode(actuators::actuator_2.pwm_down, OUTPUT); // Setter PWM pinne for aktuator to nedover

    // Setter analog pinnene til null for at motorene ikke bevege seg når programmet starter
    stop_actuators(); // Stopper aktuatorene

    // Initialiserer heis servo
    lift_servo::lift.attach(lift_servo::servo_pin); // Fester servoen til pinnen
    lift_servo::lift.write(0); // Setter heisen til lav posisjon
    lift_servo::lift.detach(); // Frakobler servoen
}

void loop() {
    read_serial(); // Motar data over seriel fra Jetson

    switch (serial_messages::state) // Sjekker hvilken tilstand programmet er i
    {
        case serial_messages::State::GET_BALL: // Tilstand 0: Heis klar til å hente ball
            get_ball(); // Henter ballen
            serial_messages::state = serial_messages::State::IDLE; // Går til neste tilstand
            break;
        case serial_messages::State::CONTROL: // Tilstand 1: Heis klar til å motta kommandoer
            actuator_limit_check(); // Sjekker om aktuatorene er over eller under grensen og oppdaterer distance_status
            move_speed(serial_messages::value_1, serial_messages::value_2); // Setter motor hastighetene
            break;
        case serial_messages::State::SET_COLOR: // Tilstand 2: Setter farge på LED stripen
            set_led_color(serial_messages::value_1, serial_messages::value_2, serial_messages::value_3); // Setter fargen på LED stripen
            serial_messages::state = serial_messages::State::IDLE; // Går til neste tilstand
            break;
        default: // Standard tilstand: Ingen handling
            stop_actuators(); // Stopper aktuatorene
            break;
    }
}

void stop_actuators()
{
    // Funksjon for å stoppe aktuatorene
    // Aktuator en
    analogWrite(actuators::actuator_1.pwm_up, 0); // Setter PWM pinne for aktuator en oppover til 0
    analogWrite(actuators::actuator_1.pwm_down, 0); // Setter PWM pinne for aktuator en nedover til 0
    actuators::actuator_1.position = 0; // Nullstiller posisjonen til aktuator en

    // Aktuator to
    analogWrite(actuators::actuator_2.pwm_up, 0); // Setter PWM pinne for aktuator to oppover til 0
    analogWrite(actuators::actuator_2.pwm_down, 0); // Setter PWM pinne for aktuator to nedover til 0
    actuators::actuator_2.position = 0; // Nullstiller posisjonen til aktuator to
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
    const uint8_t limit_check_interval = 10; // Sjekk grenser kun hver 10. loop

    limit_check_counter++; // Øker telleren for hvor ofte grensen sjekkes
    if (limit_check_counter < limit_check_interval) // Sjekker ikke grensen for ofte
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
void move_speed(int16_t speed_actuator_1, int16_t speed_actuator_2)
{
    // Aktuator 1
    if (speed_actuator_1 > 0 && actuators::actuator_1.distance_status != 1) // Hvis aktuator en skal bevege seg oppover og ikke er for høyt
    {
        analogWrite(actuators::actuator_1.pwm_up, speed_actuator_1);
        analogWrite(actuators::actuator_1.pwm_down, 0);
    } 
    else if (speed_actuator_1 < 0 && actuators::actuator_1.distance_status != -1) // Hvis aktuator en skal bevege seg nedover og ikke er for lavt
    {
        analogWrite(actuators::actuator_1.pwm_up, 0);
        analogWrite(actuators::actuator_1.pwm_down, -speed_actuator_1); // - for å få riktig retning
    }
    else // Hvis aktuator en ikke skal bevege seg
    {
        analogWrite(actuators::actuator_1.pwm_up, 0);
        analogWrite(actuators::actuator_1.pwm_down, 0);
    }
    
    // Aktuator 2
    if (speed_actuator_2 > 0 && actuators::actuator_2.distance_status != 1) // Hvis aktuator to skal bevege seg oppover og ikke er for høyt
    {
        analogWrite(actuators::actuator_2.pwm_up, speed_actuator_2);
        analogWrite(actuators::actuator_2.pwm_down, 0);
    } 
    else if (speed_actuator_2 < 0 && actuators::actuator_2.distance_status != -1) // Hvis aktuator to skal bevege seg nedover og ikke er for lavt
    {
        analogWrite(actuators::actuator_2.pwm_up, 0);
        analogWrite(actuators::actuator_2.pwm_down, -speed_actuator_2); // - for å få riktig retning
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

    // 1. Kjør heisen ned for å hente ballen
    lift_servo::lift.write(lift_servo::lift_down); // Setter heisen til lav posisjon
    delay(100); // Venter 100 ms for at heisen skal nå ned

    // 2. Stoper heisen for å la ballen renne ned
    lift_servo::lift.write(lift_servo::lift_stop); // Setter heisen til høy posisjon
    delay(1000); // Venter 1 sekund for at baller kan renne på heisen

    // 3. Kjør heisen opp med ballen
    lift_servo::lift.write(lift_servo::lift_up); // Setter heisen
    delay(100); // Venter 100 ms for at heisen skal nå opp

    lift_servo::lift.detach(); // Frakobler servoen
}

void set_led_color(uint8_t r, uint8_t g, uint8_t b) {
    // Funksjon for å sette fargen på LED stripen
    for (uint8_t i = 0; i < led_strips::num_leds; i++) {
        led_strips::strip.setPixelColor(i, led_strips::strip.Color(r, g, b)); // Setter fargen på hver LED
    }
    led_strips::strip.show(); // Oppdaterer stripen for å vise endringene
}

// Funksjon for å lese innkommende seriedata
void read_serial()
{
    // Leser innkommende seriedata fra Jetson
    if (Serial.available() > 0)
    {
        char buffer[64]; // Buffer for å lagre innkommende data
        int len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1); // Leser data til linjeskift eller buffer er fullt

        // Legg til sjekk for å sikre at vi faktisk leste noe før vi parser
        if (len > 0) {
            buffer[len] = '\0'; // Null-terminerer strengen for sikkerhet
             
            int v1, v2, v3, incoming_state;
            
            // Sjekk for den vanligste meldingen FØRST: CONTROL (2 hastigheter + state)
            if (sscanf(buffer, "%d,%d,%d", &v1, &v2, &incoming_state) == 3)
            {
                serial_messages::value_1 = v1;
                serial_messages::value_2 = v2;
                if (incoming_state >= 0 && incoming_state <= 3) {
                    serial_messages::state = static_cast<serial_messages::State>(incoming_state);
                }
            }
            // Sjekk deretter for den lengste: SET_COLOR (3 fargeverdier + state)
            else if (sscanf(buffer, "%d,%d,%d,%d", &v1, &v2, &v3, &incoming_state) == 4)
            {
                serial_messages::value_1 = v1;
                serial_messages::value_2 = v2;
                serial_messages::value_3 = v3;
                if (incoming_state >= 0 && incoming_state <= 3) {
                    serial_messages::state = static_cast<serial_messages::State>(incoming_state);
                }
            }
            // Sjekk til slutt for den korteste: IDLE/GET_BALL (kun state)
            else if (sscanf(buffer, "%d", &incoming_state) == 1)
            {
                if (incoming_state >= 0 && incoming_state <= 3) {
                    serial_messages::state = static_cast<serial_messages::State>(incoming_state);
                }
            }
        }
    }
}
