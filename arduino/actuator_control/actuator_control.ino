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

}

// Initialiserer funksjoner
float actuator_position(uint8_t* pot_pin); // Funksjon for posisjonen til aktuatoren
void actuator_move_distance(float distance, uint8_t speed, uint8_t actuator); // Funksjon for å bevege en aktuator en distanse

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


    actuator_move_distance(5.0, 30.0, 1);
}

void loop() {
    // put your main code here, to run repeatedly:
    
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

void actuator_move_distance(float distance, uint8_t speed, const uint8_t actuator)
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

    // Sjekk om distansen er lenger en aktuatoren kan flytte seg
    if (init_position + abs(distance) > act_max_stroke && distance >= 0.0) // Maks lengde
    {
        distance = max_hight - init_position; // Kalkulerer distansen så den ikke går for langt opp
    }
    else if (init_position - abs(distance) < act_min_stroke && distance < 0.0) // Min lengde
    {
        distance = min_hight - init_position; // Kalkulerer distansen så den ikke går for langt ned
    }
    else
    {
        // Do nothing
    }

    // Kjører til aktuatoren har kjørt så langt den skal
    while (abs(actuator_position(&pot_pin) - init_position) < abs(distance) + targetTolerance)
    {
        analogWrite(pwm_pin, speed); // Sender PWM signalet til motorkontrolleren
    }
    analogWrite(pwm_pin, 0); // Stopper motoren
}
