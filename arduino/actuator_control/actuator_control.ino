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
    const uint8_t servo_pin = 5; // Pinne for servo
    const uint8_t lift_down = 180; // Lav posisjon for heis
    const uint8_t lift_up = 0; // Høy posisjon for heis
    const uint8_t lift_stop = 90; // Stopp posisjon for heis
    unsigned long elevator_start_time = 0; // Tidspunkt for når heisen startet å kjøre
    bool elevator_running = false; // Variabel for å sjekke om heisen er aktiv
}

namespace led_strips {
    const uint8_t led_pin = 7; // Pinne for LED strip
    const uint16_t num_leds = 10; // Antall LED i stripen 
    Adafruit_NeoPixel strip = Adafruit_NeoPixel(num_leds, led_pin, NEO_GRB + NEO_KHZ800); // Initialiserer LED stripen
}

namespace serial_messages {
    // Enum for tilstander i programmet
    enum class State
    {
        IDLE, // Tilstand for å vente på kommandoer
        ELEVATOR, // Tilstand for å hente ball
        CONTROL, // Tilstand for å kontrollere aktuatorene
        SET_COLOR // Tilstand for å sette farge på LED stripen
    };

    State state = State::IDLE; // Initialiserer tilstanden til IDLE

    // Variabler for seriedata
    int16_t value_1 = 0; // Variabel for første verdi i seriedata
    int16_t value_2 = 0; // Variabel for andre verdi i seriedata
    int16_t value_3 = 0; // Variabel for tredje verdi i seriedata
    int16_t value_4 = 0; // Variabel for fjerde verdi i seriedata
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
void elevator(int8_t elevator_dir); // Funksjon for å kontrollere heisen for å hente ball
void set_led_color(uint8_t r, uint8_t g, uint8_t b, int8_t led_number); // Funksjon for å sette fargen på LED stripen
void read_serial(); // Funksjon for å lese innkommende seriedata

void setup() {
    // Starter seriel kominikasjon
    Serial.begin(115200);

    // Initialiserer LED stripen
    led_strips::strip.begin(); // Starter LED stripen
    led_strips::strip.setBrightness(255); // Setter lysstyrken til stripen
    // Setter alle LED til hvit farge
    set_led_color(255, 255, 255, -1); // Setter fargen til hvit

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
        case serial_messages::State::ELEVATOR: // Tilstand 0: Heis klar til å hente ball

            // Sjekk om heisen heisen står stille og om det er over 1s siden den ble kjørt sist
            if (lift_servo::elevator_running == false && millis() - lift_servo::elevator_start_time >= 1000)
            {
                elevator(serial_messages::value_1); // Kjører heisen 
            }
            else
            {
                // Gjør ingen ting
            }
            
            serial_messages::state = serial_messages::State::IDLE; // Går til neste tilstand
            break;
        case serial_messages::State::CONTROL: // Tilstand 1: Kontrollerer aktuatorene
            actuator_limit_check(); // Sjekker om aktuatorene er over eller under grensen og oppdaterer distance_status
            move_speed(serial_messages::value_1, serial_messages::value_2); // Setter motor hastighetene
            break;
        case serial_messages::State::SET_COLOR: // Tilstand 2: Setter farge på LED stripen
            set_led_color(serial_messages::value_1, serial_messages::value_2, serial_messages::value_3, serial_messages::value_4); // Setter fargen på LED stripen
            serial_messages::state = serial_messages::State::IDLE; // Går til neste tilstand
            break;
        default: // Standard tilstand: Ingen handling
            stop_actuators(); // Stopper aktuatorene
            break;
    }

    if (lift_servo::elevator_running == true && millis() - lift_servo::elevator_start_time >= 200)
    {
        elevator(0);
        lift_servo::elevator_running = false;
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

void elevator(int8_t elevator_dir)
{
    if (lift_servo::lift.attached() == false && lift_servo::elevator_running == false)
    {
        lift_servo::lift.attach(lift_servo::servo_pin);
    }
    else if (lift_servo::lift.attached() == true && lift_servo::elevator_running == true)
    {
        lift_servo::lift.detach();
        return;
    }
    else
    {
        // Gjør ingenting
    }

    if (elevator_dir == -1)
    {
        lift_servo::lift.write(lift_servo::lift_down);
        lift_servo::elevator_start_time = millis();
        lift_servo::elevator_running = true;
    }
    else if (elevator_dir == 1)
    {
        lift_servo::lift.write(lift_servo::lift_up);
        lift_servo::elevator_start_time = millis();
        lift_servo::elevator_running = true;
    }
    else
    {
        // Gjør ingenting
    }
}


// Funksjon for å sette fargen på LED stripen
void set_led_color(uint8_t r, uint8_t g, uint8_t b, int8_t led_number) {

    // Setter alle LED til en spesifikk farge
    if (led_number == -1)
    {
        for (uint8_t i = 0; i < led_strips::num_leds; i++) {
        led_strips::strip.setPixelColor(i, led_strips::strip.Color(r, g, b)); // Setter fargen på hver LED
        }
    }
    // Setter en spesifikk LED til en spesifikk farge
    else if (led_number >= 0 && led_number < led_strips::num_leds)
    {
        led_strips::strip.setPixelColor(led_number, led_strips::strip.Color(r, g, b)); // Setter fargen på den spesifikke LED
    }
    else
    {
        Serial.println("Invalid LED number!"); // Skriver ut en feilmelding hvis led_number er utenfor gyldig område
        return; // Avslutter funksjonen tidlig
    }
    led_strips::strip.show(); // Oppdaterer stripen for å vise endringene
}

// Funksjon for å lese innkommende seriedata
void read_serial()
{
    // Leser innkommende seriedata fra Jetson
    if (Serial.available() <= 0)
    {
        return; // Hvis det ikke er noen data tilgjengelig, bare avslutt funksjonen.
    }

    // Nullstiller verdiene
    serial_messages::value_1 = 0; 
    serial_messages::value_2 = 0;
    serial_messages::value_3 = 0;
    serial_messages::value_4 = 0;

    char buffer[64]; // Buffer for å lagre innkommende data
    int len = Serial.readBytesUntil('\n', buffer, sizeof(buffer) - 1); // Leser data til linjeskift eller buffer er fullt

    // Legg til sjekk for å sikre at vi faktisk leste noe før vi parser
    if (len > 0)
    {
        buffer[len] = '\0'; // Null-terminerer strengen for sikkerhet
            
        int v1, v2, v3, v4, incoming_state;

        // Tell antall kommaer for å bestemme meldingstype raskt
        uint8_t comma_count = 0;
        for (int i = 0; i < len; i++)
        {
            if (buffer[i] == ',')
            {
                comma_count++;
            }
        }
        
        // Sjekk for den vanligste meldingen FØRST: CONTROL (2 hastigheter + state)
        if (comma_count == 2) // For CONTROL-meldinger (f.eks. "25,200,1")
        {
            if (sscanf(buffer, "%d,%d,%d", &v1, &v2, &incoming_state) != 3)
            {
                return; // Ugyldig format, ignorer meldingen
            }

            if (incoming_state < 0 || incoming_state > 3)
            {
                return; // Ugyldig state, ignorer meldingen
            }

            serial_messages::value_1 = v1;
            serial_messages::value_2 = v2;
            serial_messages::state = static_cast<serial_messages::State>(incoming_state);

            return; // Avslutter funksjonen siden meldingen er lest
        }

        // Sjekker for get_ball-meldinger: GET_BALL
        else if (comma_count == 1) // For GET_BALL-meldinger (f.eks. "1, 0" eller "-1, 1")
        {
            if (sscanf(buffer, "%d,%d", &v1, &incoming_state) != 2)
            {
                return; // Ugyldig format, ignorer meldingen
            } 
                
            if (incoming_state < 0 || incoming_state > 3)
            {
                return; // Ugyldig state, ignorer meldingen
            }

            serial_messages::value_1 = v1;
            serial_messages::state = static_cast<serial_messages::State>(incoming_state);

            return; // Avslutter funksjonen siden meldingen er lest
        }

        // Sjekk deretter for den lengste: SET_COLOR (3 fargeverdier + state)
        else if (comma_count == 4) // For SET_COLOR-meldinger (f.eks. "255,0,0,3")
        {
            if (sscanf(buffer, "%d,%d,%d,%d,%d", &v1, &v2, &v3, &v4, &incoming_state) != 5)
            {
                return; // Ugyldig format, ignorer meldingen
            }

            if (incoming_state < 0 || incoming_state > 3)
            {
                return; // Ugyldig state, ignorer meldingen
            }

            serial_messages::value_1 = v1;
            serial_messages::value_2 = v2;
            serial_messages::value_3 = v3;
            serial_messages::value_4 = v4;
            serial_messages::state = static_cast<serial_messages::State>(incoming_state);

            return; // Avslutter funksjonen siden meldingen er lest
        }

        // Sjekk til slutt for den korteste: IDLE/GET_BALL (kun state)
        else if (comma_count == 0) // For IDLE/GET_BALL-meldinger (f.eks. "0")
        {
            if (sscanf(buffer, "%d", &incoming_state) != 1)
            {
                return; // Hvis sscanf ikke klarer å lese fem verdier, hopp over denne meldingen
            }

            if (incoming_state < 0 || incoming_state > 3)
            {
                return; // Hvis state ikke er gyldig, hopp over denne meldingen
            }

            serial_messages::state = static_cast<serial_messages::State>(incoming_state);
        }
    }
}
