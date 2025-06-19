

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
    float pot_raw_data_act_1{0};
}

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);
}

void loop() {
    // put your main code here, to run repeatedly:

    pot_raw_data_act_1 = analogRead(actuators::actuator_1::pot_feedback);
    Serial.print(pot_raw_data_act_1);
    delay(1);
}
