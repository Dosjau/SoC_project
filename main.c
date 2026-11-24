#include <stdio.h>
#include "arm/motor.h"

int main() {
    struct MotorParameters motor_1_parameters = initialize_motor_parameters();
    struct MotorState motor_1 = initialize_motor();
    float voltage_d, voltage_q;
    float load_torque = 0;
    float time_step = 1.0f / 32000.0f;
    uint8_t control_signal = 6; 
    
    int i;
    while (i < 5){
        calculate_voltages(&motor_1, &motor_1_parameters, control_signal, &voltage_d, &voltage_q);
        motor_step(&motor_1, &motor_1_parameters, voltage_d, voltage_q, load_torque, time_step);

        printf("Time step: %d\n", i);
        printf("i_d: %f, i_q: %f, speed: %f, angle: %f\n",
        motor_1.current_d, motor_1.current_q, motor_1.motor_speed, motor_1.motor_angle);
        i += 1;
    }
    
    return 0;
}