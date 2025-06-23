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

    // Setter setter analog pinnene til null for at motorene ikke skal bevege seg
    analogWrite(actuators::actuator_1.pwm_up, 0);
    analogWrite(actuators::actuator_1.pwm_down, 0);

    analogWrite(actuators::actuator_2.pwm_up, 0);
    analogWrite(actuators::actuator_2.pwm_down, 0);
}

void loop() {
    // put your main code here, to run repeatedly:

    for (int i = 0; i < 10; i++) // Kjører en loop for å teste aktuatorene
    {
        if (i == 0)
        {
            actuator_move_speed(20, 1); // Beveger aktuator en oppover med hastighet 10
        }
        else if (i == 1)
        {
            actuator_move_speed(30, 1);; // Beveger aktuator to oppover med hastighet 10
        }
        else if (i == 2)
        {
            actuator_move_speed(50, 1);; // Beveger aktuator to oppover med hastighet 10
        }
        else if (i == 3)
        {
            actuator_move_speed(0, 1);; // Beveger aktuator to oppover med hastighet 10
        }
        else if (i == 4)
        {
            actuator_move_speed(-20, 1);; // Beveger aktuator to oppover med hastighet 10
        }
        else if (i == 5)
        {
            actuator_move_speed(-30, 1);; // Beveger aktuator to oppover med hastighet 10
        }
        else if (i == 6)
        {
            actuator_move_speed(-50, 1); // Stopper aktuator en
        }
        else if (i == 7)
        {
            actuator_move_speed(0, 1); // Stopper aktuator to
        }
        else if (i == 8)
        {
            actuator_move_speed(20, 2);
        }
        else if (i == 9)
        {
            actuator_move_speed(-20, 2);
        }
        else
        {
            actuator_move_speed(0, 1);
            actuator_move_speed(0, 2);
        }
        

        delay(500); // Venter i 0.5 sekund
    }

    analogWrite(actuators::actuator_1.pwm_up, 0);
    analogWrite(actuators::actuator_1.pwm_down, 0);

    analogWrite(actuators::actuator_2.pwm_up, 0);
    analogWrite(actuators::actuator_2.pwm_down, 0);
    
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

void actuator_move_distance(const float distance, uint8_t speed, const uint8_t actuator)
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

    const uint8_t pwm_pin = (speed >= 0.0) ? pSelectedActuatorPins -> pwm_up : pSelectedActuatorPins -> pwm_down; // PWM pinne som skal brukes

    if (speed == 0)
    {   
        // Hvis hastigheten er 0, så stopper motoren
        analogWrite(pSelectedActuatorPins -> pwm_up, 0); 
        analogWrite(pSelectedActuatorPins -> pwm_down, 0); 
    }
    else
    {
        analogWrite(pwm_pin, speed); // Sender PWM signalet til motorkontrolleren
    }
}
