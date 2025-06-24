// Initialiserer variabler
// Navnerom for aktuatorene
namespace actuators {
    
    // Struct for pinnenummerene
    struct ActuatorData
    {
        const uint8_t pwm_up; // Pinne for PWM oppover
        const uint8_t pwm_down; // Pinne for PWM nedover
        const uint8_t pot_feedback; // Pinne for potensjometer
        int8_t distance_status; // Motor status. -1 er under nedre grense, 1 er over øvre grense
    };

    // Aktuator en
    ActuatorData actuator_1 = {9, 10, A4, 0};

    // Aktuator to
    ActuatorData actuator_2 = {11, 12, A3, 0};
}

// Navnerom for globale variabler
namespace {
    struct Speeds
    {
        int16_t speed_actuator_1; // Fart for aktuator en
        int16_t speed_actuator_2; // Fart for aktuator to
    };

    Speeds actuator_speeds = {0, 0}; // Initialiserer hastighetene for aktuatorene

    int teller{0};
}

// Initialiserer funksjoner
void stop_actuator(uint8_t actuator); // Funksjon for å stoppe aktuatoren
float actuator_position(const uint8_t* pPot_pin); // Funksjon for posisjonen til aktuatoren
void actuator_limit_check(); // Funksjon for å sjekke om aktuatoren er over eller under ønsket maks eller min høyde
void actuator_move_distance(float distance, const int16_t speed, const uint8_t actuator); // Funksjon for å bevege en aktuator en distanse
void actuator_move_speed(const int16_t speed, const uint8_t actuator); // Funksjon for å bevege en aktuator med en hastighet
void move_speed(); // Funksjon for å bevege begge aktuatorene med hastighetene i actuator_speeds
void read_serial(); // Funksjon for å lese innkommende seriedata
void clear_serial_buffer(); // Funksjon for å tømme seriebufferen

void setup() {
    // Starter seriel kominikasjon
    Serial.begin(9600);

    // Setter pin modusene
    // Aktuator en
    pinMode(actuators::actuator_1.pwm_up, OUTPUT);
    pinMode(actuators::actuator_1.pwm_down, OUTPUT);

    // Aktuator to
    pinMode(actuators::actuator_2.pwm_up, OUTPUT);
    pinMode(actuators::actuator_2.pwm_down, OUTPUT);

    // Setter analog pinnene til null for at motorene ikke skal bevege seg når programmet starter
    stop_actuator(1); // Stopper aktuator en
    stop_actuator(2); // Stopper aktuator to
}

void loop() {
    actuator_limit_check(); // Sjekker om aktuatorene er over eller under grensen og oppdaterer distance_status
    read_serial(); // Motar data over seriel fra Jetson
    move_speed(); // Setter motor hastighetene
}

// Funksjon for å stoppe aktuatorene
void stop_actuator(uint8_t actuator)
{
    if (actuator == 1)
    {
        analogWrite(actuators::actuator_1.pwm_up, 0);
        analogWrite(actuators::actuator_1.pwm_down, 0);
    }
    else if (actuator == 2)
    {
        analogWrite(actuators::actuator_2.pwm_up, 0);
        analogWrite(actuators::actuator_2.pwm_down, 0);
    }
}

// Funksjon for å returnere posisjonen til aktuatoren
float actuator_position(const uint8_t* pPot_pin) // Tar inn en peker til potensjometer pinnen
{
    const float act_max_stroke{50.0}; // Maks slaglengde for aktuatorene (mm)
    const float adc_max_hight{1018.0}; // ADC verdien for maks lengde
    const float adc_min_hight{2.0}; // ADC verdi for minimum lengde

    float dist = (float(analogRead(*pPot_pin)) - adc_min_hight) * act_max_stroke / (adc_max_hight - adc_min_hight); // Kalkulerer distansen

    return dist; // Returnerer distanse 
}

// Returnerer: 0 = OK å bevege seg, 1 = for høyt (ikke beveg opp), -1 = for lavt (ikke beveg ned)
void actuator_limit_check()
{   
    const float act_max_stroke{50.0}; // Maks slaglengde for aktuatorene (mm)
    const float offset{3.0}; // Distansen fra toppen og bunden hvor aktuatorene ikke skal gå innenfor (mm)

    // Array med pekere til begge aktuatorene
    actuators::ActuatorData* pActuators_list[] = {
        &actuators::actuator_1,
        &actuators::actuator_2
    };

    // Sjekk om aktuatorene er over eller under grensen
    for (int i = 0; i < 2; i++)
    {
        actuators::ActuatorData* pActuator = pActuators_list[i]; // Peker til aktuatoren i listen
        float dist = actuator_position(&pActuator->pot_feedback); // Henter posisjonen til aktuatoren
        
        // Sjekker om aktuatoren er over eller under grensen
        if (dist >= act_max_stroke - offset)
        {
            // For høyt
            pActuator->distance_status = 1;
        } 
        else if (dist <= 0.0 + offset)
        {
            // For lavt
            pActuator->distance_status = -1;
        } 
        else
        {
            // Innenfor grensene
            pActuator->distance_status = 0;
        }
    }
}

void actuator_move_distance(const float distance, const int16_t speed, const uint8_t actuator)
{
    // Initialiserer pekeren til selectedActuatorData strukturen til en nullpeker
    actuators::ActuatorData* pSelectedActuatorData = nullptr; 
    
    // Velger aktuator
    if (actuator == 1) // Hvis det er aktuator en
    {
        pSelectedActuatorData = &actuators::actuator_1; // Peker til selectedActuatorData til aktuator en
    }
    else if (actuator == 2) // Hvis det er aktuator to
    {
        pSelectedActuatorData = &actuators::actuator_2; // Peker til selectedActuatorData til aktuator to
    }
    else // Hvis aktuator valget ikke er gyldig
    {
        Serial.print("Ikke et valg. Velg mellom en eller to aktuator");
        return;
    }
    
    // Initialiserer variabler
    const float targetTolerance{0.3}; // Toleranse for hvor nære ønsket distanse før ok (mm) 

    const uint8_t pot_pin = pSelectedActuatorData->pot_feedback; // Aktuator potensjometer pinne
    const uint8_t pwm_pin = (distance >= 0.0) ? pSelectedActuatorData->pwm_up : pSelectedActuatorData->pwm_down; // PWM pinne som skal brukes
    const float init_position{actuator_position(&pot_pin)}; // Initial aktuatorposisjon

    // Sjekk om aktuatoren allerede er i grenseområdet og prøver å bevege seg feil vei
    if ((distance > 0.0 && pSelectedActuatorData->distance_status == 1) || (distance < 0.0 && pSelectedActuatorData->distance_status == -1))
    {
        return; // Kan ikke bevege seg i den retningen
    }

    analogWrite(pwm_pin, speed); // Sender PWM signalet til motorkontrolleren
    
    // Kjører til aktuatoren har kjørt ønsket distanse eller når en grense
    while (abs(actuator_position(&pot_pin) - init_position) < abs(distance) - targetTolerance)
    {
        // Sjekk kontinuerlig om vi når en grense under bevegelsen
        if ((distance > 0.0 && pSelectedActuatorData->distance_status == 1) || (distance < 0.0 && pSelectedActuatorData->distance_status == -1))
        {
            break; // Stopp hvis vi når en grense
        }
    }
    analogWrite(pwm_pin, 0); // Stopper motoren
}

void actuator_move_speed(const int16_t speed, const uint8_t actuator)
{
    // Initialiserer pekeren til selectedActuatorData strukturen til en nullpeker
    actuators::ActuatorData* pSelectedActuatorData = nullptr;
    
    // Velger aktuator
    if (actuator == 1) // Hvis det er aktuator en
    {
        pSelectedActuatorData = &actuators::actuator_1; // Peker til selectedActuatorData til aktuator en
    }
    else if (actuator == 2) // Hvis det er aktuator to
    {
        pSelectedActuatorData = &actuators::actuator_2; // Peker til selectedActuatorData til aktuator to
    }
    else // Hvis aktuator valget ikke er gyldig
    {
        Serial.print("Ikke et valg. Velg mellom en eller to aktuator");
        return;
    }

    // Initialiserer variabler
    const uint8_t pwm_pin = (speed >= 0) ? pSelectedActuatorData->pwm_up : pSelectedActuatorData->pwm_down; // PWM pinne som skal brukes


    // Sjekker om aktuatoren er innenfor grensen for å bevege seg
    if ((speed > 0 && pSelectedActuatorData->distance_status != 1) || (speed < 0 && pSelectedActuatorData->distance_status != -1)) // Hvis den er innenfor grensen for å bevege seg
    {   
        analogWrite(pwm_pin, abs(speed)); // Sender PWM signalet til motorkontrolleren
    }
    else // Hvis den ikke er innenfor grensen for å bevege seg
    {
        // Stopper motoren
        analogWrite(pSelectedActuatorData->pwm_up, 0);
        analogWrite(pSelectedActuatorData->pwm_down, 0);
    }
}

void move_speed()
{
    // Array med pekere til begge aktuatorene
    actuators::ActuatorData* pActuators_list[] = {
        &actuators::actuator_1,
        &actuators::actuator_2
    };

    int16_t* pSpeed_list[] = {
        &actuator_speeds.speed_actuator_1,
        &actuator_speeds.speed_actuator_2
    };

    // Sjekk om aktuatorene er over eller under grensen
    for (int i = 0; i < 2; i++)
    {
        actuators::ActuatorData* pActuator = pActuators_list[i]; // Peker til aktuatoren i listen
        int16_t speed = *pSpeed_list[i];

        const uint8_t pwm_pin = (speed >= 0) ? pActuator->pwm_up : pActuator->pwm_down; // PWM pinne som skal brukes
        
        // Sjekker om aktuatoren er over eller under grensen
        if ((speed > 0 && pActuator->distance_status != 1) || (speed < 0 && pActuator->distance_status != -1))
        {
            analogWrite(pwm_pin, abs(speed)); // Sender PWM signalet til motorkontrolleren
        }
        else
        {
            // Stopper motoren
            analogWrite(pActuator->pwm_up, 0);
            analogWrite(pActuator->pwm_down, 0);
        }
    }
}


void read_serial()
{
    if (Serial.available() > 0)
    {
        String input = Serial.readStringUntil('\n');
        input.trim();
        
        int speed1, speed2, checksum;
        int parsed = sscanf(input.c_str(), "%d,%d,%d", &speed1, &speed2, &checksum);
        
        if (parsed == 3)
        {
            // Valider checksum
            int expected_checksum = ((speed1 + speed2) % 256 + 256) % 256;
            if (checksum == expected_checksum)
            {
                actuator_speeds.speed_actuator_1 = constrain(speed1, -255, 255);
                actuator_speeds.speed_actuator_2 = constrain(speed2, -255, 255);
            }
            else
            {
                Serial.print("ERROR: Checksum mismatch: Melding = ");
                Serial.println(input);
                clear_serial_buffer();
            }
        }
        else
        {
            Serial.print("ERROR: Invalid format: Melding = ");
            Serial.println(input);
            clear_serial_buffer();
        }
    }
}

void clear_serial_buffer()
{
    while (Serial.available() > 0)
    {
        Serial.read(); // Les og forkast alle bytes i bufferen
    }
}
