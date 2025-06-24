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
    int teller{0};
}

// Initialiserer funksjoner
void stop_actuator(uint8_t actuator); // Funksjon for å stoppe aktuatoren
float actuator_position(uint8_t* pot_pin); // Funksjon for posisjonen til aktuatoren
void actuator_limit_check(); // Funksjon for å sjekke om aktuatoren er over eller under ønsket maks eller min høyde
void actuator_move_distance(float distance, const int16_t speed, const uint8_t actuator); // Funksjon for å bevege en aktuator en distanse
void actuator_move_speed(const int16_t speed, const uint8_t actuator); // Funksjon for å bevege en aktuator med en hastighet

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
    actuators::ActuatorData* actuators_list[] = {
        &actuators::actuator_1,
        &actuators::actuator_2
    };

    // Sjekk om aktuatorene er over eller under grensen
    for (int i = 0; i < 2; i++) {
        actuators::ActuatorData* pActuator = actuators_list[i]; // Peker til aktuatoren i listen
        float dist = actuator_position(&pActuator->pot_feedback); // Henter posisjonen til aktuatoren
        
        // Sjekker om aktuatoren er over eller under grensen
        if (dist >= act_max_stroke - offset) {
            // For høyt
            pActuator->distance_status = 1;
        } 
        else if (dist <= 0.0 + offset) {
            // For lavt
            pActuator->distance_status = -1;
        } 
        else {
            // Innenfor grensene
            pActuator->distance_status = 0;
        }
    }
}

void actuator_move_distance(const float distance, const int16_t speed, const uint8_t actuator)
{
    // Initialiserer pekeren til selectedActuatorData strukturen til en nulponter
    actuators::ActuatorData* pSelectedActuatorData = nullptr; 
    
    // Velger aktuator
    if (actuator == 1) // Hvis det er aktuator en
    {
        pSelectedActuatorData = &actuators::actuator_1; // Peker til selectedActuatorData til aktoator en
    }
    else if (actuator == 2) // Hvis det er aktuator to
    {
        pSelectedActuatorData = &actuators::actuator_2; // Peker til selectedActuatorData til aktoator to
    }
    else // Hvis aktuator valget ikke er gyldig
    {
        Serial.print("Ikke et valg. Velg mellom en eller to aktuator");
        return;
    }
      // Initialserer variabler
    const float targetTolerance{0.3}; // Tolleranse for hvor hvor nære ønsket distanse før ok (mm) 

    const uint8_t pot_pin = pSelectedActuatorData -> pot_feedback; // Aktuator potensjometer pinne
    const uint8_t pwm_pin = (distance >= 0.0) ? pSelectedActuatorData -> pwm_up : pSelectedActuatorData -> pwm_down; // PWM pinne som skal brukes
    const float init_position{actuator_position(&pot_pin)}; // Initial aktuatorposisjon

    // Sjekk om aktuatoren allerede er i grenseområdet og prøver å bevege seg feil vei
    if ((distance > 0.0 && pSelectedActuatorData -> distance_status == 1) || (distance < 0.0 && pSelectedActuatorData -> distance_status == -1))
    {
        return; // Kan ikke bevege seg i den retningen
    }

    analogWrite(pwm_pin, speed); // Sender PWM signalet til motorkontrolleren
    
    // Kjører til aktuatoren har kjørt ønsket distanse eller når en grense
    while (abs(actuator_position(&pot_pin) - init_position) < abs(distance) - targetTolerance)
    {
        // Sjekk kontinuerlig om vi når en grense under bevegelsen
        if ((distance > 0.0 && pSelectedActuatorData -> distance_status == 1) || (distance < 0.0 && pSelectedActuatorData -> distance_status == -1))
        {
            break; // Stopp hvis vi når en grense
        }
    }
    analogWrite(pwm_pin, 0); // Stopper motoren
}

void actuator_move_speed(const int16_t speed, const uint8_t actuator)
{
    // Initialiserer pekeren til selectedActuatorData strukturen til en nulponter
    actuators::ActuatorData* pSelectedActuatorData = nullptr; 
    
    // Velger aktuator
    if (actuator == 1) // Hvis det er aktuator en
    {
        pSelectedActuatorData = &actuators::actuator_1; // Peker til selectedActuatorData til aktoator en
    }
    else if (actuator == 2) // Hvis det er aktuator to
    {
        pSelectedActuatorData = &actuators::actuator_2; // Peker til selectedActuatorData til aktoator to
    }
    else // Hvis aktuator valget ikke er gyldig
    {
        Serial.print("Ikke et valg. Velg mellom en eller to aktuator");
        return;
    }

    // Initialiserer variabler
    const uint8_t pwm_pin = (speed >= 0.0) ? pSelectedActuatorData -> pwm_up : pSelectedActuatorData -> pwm_down; // PWM pinne som skal brukes


    // Sjekker om aktuatoren er innenfor grensen for å bevege seg
    if ((speed > 0.0 && pSelectedActuatorData -> distance_status != 1) || (speed < 0.0 && pSelectedActuatorData -> distance_status != -1)) // Hvis den er innenfor grensen for å bevege seg
    {   
        analogWrite(pwm_pin, abs(speed)); // Sender PWM signalet til motorkontrolleren
    }
    else // Hvis den ikke er innenfor grensen for å bevege seg
    {
        // Hvis hastigheten er 0, så stopper motoren
        analogWrite(pSelectedActuatorData -> pwm_up, 0);
        analogWrite(pSelectedActuatorData -> pwm_down, 0);
    }
}
