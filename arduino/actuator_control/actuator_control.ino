// Initialiserer variabler

// Navnerom for aktuatorene
namespace actuators {
    
    // Struct for pinnenummerene
    struct ActuatorPins
    {
        const uint8_t pwm_up;
        const uint8_t pwm_down;
        const uint8_t pot_feedback;
    };

    // Aktuator en
    ActuatorPins actuator_1 = {9, 10, A4};

    // Aktuator to
    ActuatorPins actuator_2 = {11, 12, A3};
}

// Navnerom for globale variabler
namespace {
    int teller{0};
}

// Initialiserer funksjoner
float actuator_position(uint8_t* pot_pin); // Funksjon for posisjonen til aktuatoren
int8_t actuator_limit_check(const uint8_t actuator); // Funksjon for å sjekke om aktuatoren er over eller under ønsket maks eller min høyde
void actuator_move_distance(float distance, const uint8_t speed, const uint8_t actuator); // Funksjon for å bevege en aktuator en distanse
void actuator_move_speed(const uint8_t speed, const uint8_t actuator); // Funksjon for å bevege en aktuator med en hastighet

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
    analogWrite(actuators::actuator_1.pwm_up, 0);
    analogWrite(actuators::actuator_1.pwm_down, 0);

    analogWrite(actuators::actuator_2.pwm_up, 0);
    analogWrite(actuators::actuator_2.pwm_down, 0);
}

void loop() {
    // put your main code here, to run repeatedly:


    if (teller == 0)
    {
        actuator_move_speed(25, 1);
        delay(0.5);
        teller =+ 1;
    }
    else if (teller == 1)
    {
        actuator_move_speed(-25, 1);
        delay(0.5);
        teller =+ 1;
    }
    else
    {
        analogWrite(actuators::actuator_1.pwm_up, 0);
        analogWrite(actuators::actuator_1.pwm_down, 0);

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
int8_t actuator_limit_check(const uint8_t actuator)
{
    // Initialiserer pekeren til selectedActuatorPins strukturen til en nulponter
    actuators::ActuatorPins* pSelectedActuatorPins = nullptr; 
    
    // Velger aktuator
    if (actuator == 1) // Hvis det er aktuator en
    {
        pSelectedActuatorPins = &actuators::actuator_1; // Peker til selectedActuatorPins til aktoator en
    }
    else if (actuator == 2) // Hvis det er aktuator to
    {
        pSelectedActuatorPins = &actuators::actuator_2; // Peker til selectedActuatorPins til aktoator to
    }
    else // Hvis aktuator valget ikke er gyldig
    {
        Serial.print("Ikke et valg. Velg mellom en eller to aktuator");
        return 0;
    }

    const uint8_t pot_pin = pSelectedActuatorPins -> pot_feedback; // Aktuator potensjometer pinne

    const float act_max_stroke{50.0}; // Maks slaglengde for aktuatorene (mm)
    const float adc_max_hight{1018.0}; // ADC verdien for maks lengde
    const float adc_min_hight{2.0}; // ADC verdi for minimum lengde
    const float offset{3.0}; // Distansen fra toppen og bunden hvor aktuatorene ikke skal gå innenfor (mm)

    float dist = (float(analogRead(pot_pin)) - adc_min_hight) * act_max_stroke / (adc_max_hight - adc_min_hight); // Kalkulerer distansen

    if (dist >= act_max_stroke - offset) // For høyt
    {
        return 1;
    }
    else if (dist <= 0.0 + offset) // For lavt
    {
        return -1;
    }
    else {
        return 0; // OK å bevege seg
    }
}

void actuator_move_distance(const float distance, const uint8_t speed, const uint8_t actuator)
{
    // Initialiserer pekeren til selectedActuatorPins strukturen til en nulponter
    actuators::ActuatorPins* pSelectedActuatorPins = nullptr; 
    
    // Velger aktuator
    if (actuator == 1) // Hvis det er aktuator en
    {
        pSelectedActuatorPins = &actuators::actuator_1; // Peker til selectedActuatorPins til aktoator en
    }
    else if (actuator == 2) // Hvis det er aktuator to
    {
        pSelectedActuatorPins = &actuators::actuator_2; // Peker til selectedActuatorPins til aktoator to
    }
    else // Hvis aktuator valget ikke er gyldig
    {
        Serial.print("Ikke et valg. Velg mellom en eller to aktuator");
        return;
    }
    
    // Initialserer variabler
    const float act_max_stroke{50.0}; // Maks slaglengde for aktuatorene (mm)
    const float act_min_stroke{0.0}; // Minste slaglengde for aktuatorene (mm)
    const float targetTolerance{0.3}; // Tolleranse for hvor hvor nære ønsket distanse før ok (mm) 
    const float max_hight{48}; // Maksimal høyde for å ikke kjøre aktuatoren helt til toppen (mm)
    const float min_hight{2}; // Minimum høyde for å ikke kjøre aktuatoren helt til bunden (mm)

    const uint8_t pot_pin = pSelectedActuatorPins -> pot_feedback; // Aktuator potensjometer pinne
    const uint8_t pwm_pin = (distance >= 0.0) ? pSelectedActuatorPins -> pwm_up : pSelectedActuatorPins -> pwm_down; // PWM pinne som skal brukes
    const float init_position{actuator_position(&pot_pin)}; // Initial aktuatorposisjon
    float distance_adj{distance};

    // Sjekk om distansen er lenger en aktuatoren kan flytte seg
    if ((init_position >= max_hight && distance >= 0.0) || (init_position <= min_hight && distance < 0.0)) // Hvis den er utenfor området så går den ikke lenger
    {
        return; 
    }
    else if (init_position + abs(distance) > act_max_stroke && distance >= 0.0) // Maks lengde
    {
        distance_adj = max_hight - init_position; // Kalkulerer distansen så den ikke går for langt opp
    }
    else if (init_position - abs(distance) < act_min_stroke && distance < 0.0) // Min lengde
    {
        distance_adj = min_hight - init_position; // Kalkulerer distansen så den ikke går for langt ned
    }
    else
    {
        // Do nothing
    }

    analogWrite(pwm_pin, speed); // Sender PWM signalet til motorkontrolleren
    // Kjører til aktuatoren har kjørt så langt den skal
    while (abs(actuator_position(&pot_pin) - init_position) < abs(distance_adj) + targetTolerance)
    {
        // Wait
    }
    analogWrite(pwm_pin, 0); // Stopper motoren
}

void actuator_move_speed(const uint8_t speed, const uint8_t actuator)
{
    // Initialiserer pekeren til selectedActuatorPins strukturen til en nulponter
    actuators::ActuatorPins* pSelectedActuatorPins = nullptr; 
    
    // Velger aktuator
    if (actuator == 1) // Hvis det er aktuator en
    {
        pSelectedActuatorPins = &actuators::actuator_1; // Peker til selectedActuatorPins til aktoator en
    }
    else if (actuator == 2) // Hvis det er aktuator to
    {
        pSelectedActuatorPins = &actuators::actuator_2; // Peker til selectedActuatorPins til aktoator to
    }
    else // Hvis aktuator valget ikke er gyldig
    {
        Serial.print("Ikke et valg. Velg mellom en eller to aktuator");
        return;
    }

    // Initialiserer variabler
    const uint8_t pwm_pin = (speed >= 0.0) ? pSelectedActuatorPins -> pwm_up : pSelectedActuatorPins -> pwm_down; // PWM pinne som skal brukes


    // Sjekker om aktuatoren er innenfor grensen for å bevege seg
    if ((speed > 0.0 && actuator_limit_check(actuator) != 1) || (speed < 0.0 && actuator_limit_check(actuator) != -1)) // Hvis den er innenfor grensen for å bevege seg
    {   
        analogWrite(pwm_pin, speed); // Sender PWM signalet til motorkontrolleren
    }
    else // Hvis den ikke er innenfor grensen for å bevege seg
    {
        // Hvis hastigheten er 0, så stopper motoren
        analogWrite(pSelectedActuatorPins -> pwm_up, 0);
        analogWrite(pSelectedActuatorPins -> pwm_down, 0); 
    }
}


