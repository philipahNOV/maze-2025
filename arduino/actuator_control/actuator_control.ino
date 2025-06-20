// Initialiserer variabler

// Namespace for aktuatorene
namespace actuators {
    
    // Struct for pinnenummerene
    struct ActuatorPins
    {
        uint8_t pwm_up;
        uint8_t pwm_down;
        uint8_t pot_feedback;
    };

    ActuatorPins actuator_1 = {9, 10, A4};

    // Namespace for aktuator 2
    ActuatorPins actuator_2 = {11, 12, A3};
}

namespace {
    float dist_act_1{0};
    float dist_act_2{0};
}

float actuator_position(uint8_t* pot_pin);
void actuator_mov_dist(float position, float speed, int actuator);

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);

    dist_act_1 = actuator_position(&actuators::actuator_1.pot_feedback);
    dist_act_2 = actuator_position(&actuators::actuator_2.pot_feedback);

    pinMode(actuators::actuator_1.pwm_up, OUTPUT);
    pinMode(actuators::actuator_1.pwm_down, OUTPUT);

    pinMode(actuators::actuator_2.pwm_up, OUTPUT);
    pinMode(actuators::actuator_2.pwm_down, OUTPUT);

    actuator_mov_dist(25.0, 30.0, 1);
}

void loop() {
    // put your main code here, to run repeatedly:

    dist_act_1 = actuator_position(&actuators::actuator_1.pot_feedback);
    dist_act_2 = actuator_position(&actuators::actuator_2.pot_feedback);
    
}

// Funksjon for å returnere posisjonen til aktuatoren
float actuator_position(uint8_t* pPot_pin) 
{
    const float act_max_stroke = 50.0; // mm
    const float adc_max_hight = 1018.0;
    const float adc_min_hight = 2.0;

    float dist = (float(analogRead(*pPot_pin)) - adc_min_hight) * act_max_stroke / (adc_max_hight - adc_min_hight);

    return dist;
}

void actuator_mov_dist(float distance, float speed, int actuator)
{
    float targetTolerance{0.3}; // mm
    float max_hight{48}; // mm
    float min_hight{2}; //mm

    actuators::ActuatorPins* pSelectedActuatorPins = nullptr; // Initialiserer pekeren til selectedActuatorPins strukturen til en nulponter
    
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

    
    uint8_t pot_pin = pSelectedActuatorPins -> pot_feedback;
    uint8_t pwm_pin = (distance >= 0.0) ? pSelectedActuatorPins -> pwm_up : pSelectedActuatorPins -> pwm_down;
    float init_position = actuator_position(&pot_pin);

    if (init_position + distance > max_hight && distance >= 0.0)
    {
        distance = max_hight;
    }
    else if (init_position - abs(distance) && distance < 0.0)
    {
        distance = -max_hight;
    }
    else
    {
        // Do nothing
    }

    Serial.print("Pos: ");
    Serial.print(actuator_position(&pot_pin));
    Serial.print(" Dist: ");
    Serial.println();

    while (abs(actuator_position(&pot_pin) - init_position) < abs(distance) + targetTolerance)
    {
        Serial.print("1: ");
        Serial.print(abs(actuator_position(&pot_pin) - init_position));
        Serial.print(" 2: ");
        Serial.println(abs(distance) + targetTolerance);
        //analogWrite(pwm_pin, constrain(speed, 0, 255));
    }

    Serial.print("Ferdig");
}
