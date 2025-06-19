

// Initialiserer variabler
// Namespace for aktuator 1
namespace actuators {
    namespace actuator_1 {
        const uint8_t pwm_fw{9};
        const uint8_t pwm_bw{10};

        const uint8_t pot_feedback{A4};
    }

    // Namespace for aktuator 2
    namespace actuator_2 {
        const uint8_t pwm_fw{11};
        const uint8_t pwm_bw{12};

        const uint8_t pot_feedback{A3};
    }
}

namespace {
    uint16_t pot_raw_data_act_1{0};
    uint16_t pot_raw_data_act_2{0};

    float dist_act_1{0};
    float dist_act_2{0};
}

float actuator_dist(uint8_t pot_pin);

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);

    pot_raw_data_act_1 = analogRead(actuators::actuator_1::pot_feedback);
    pot_raw_data_act_2 = analogRead(actuators::actuator_2::pot_feedback);

    dist_act_1 = actuator_dist(actuators::actuator_1::pot_feedback);
    dist_act_2 = actuator_dist(actuators::actuator_2::pot_feedback);
}

void loop() {
    // put your main code here, to run repeatedly:

    pot_raw_data_act_1 = analogRead(actuators::actuator_1::pot_feedback);
    pot_raw_data_act_2 = analogRead(actuators::actuator_2::pot_feedback);

    dist_act_1 = actuator_dist(actuators::actuator_1::pot_feedback);
    dist_act_2 = actuator_dist(actuators::actuator_2::pot_feedback);
    
    Serial.print("Aktuator 1: Raw = ");
    Serial.print(pot_raw_data_act_1);
    Serial.print(" Dist = ");
    Serial.print(dist_act_1);

    Serial.print(" | Aktuator 2: Raw = ");
    Serial.print(pot_raw_data_act_2);
    Serial.print(" Dist = ");
    Serial.print(dist_act_2);
    Serial.print("\r");

    delay(1);
}

float actuator_dist(uint8_t pot_pin)
{
    float act_max_stroke = 50.0; // mm
    float adc_max_hight = 1018.0;
    float adc_min_hight = 2.0;

    float dist = (float(analogRead(pot_pin)) - adc_min_hight) * act_max_stroke / (adc_max_hight - adc_min_hight);

    return dist;
}
