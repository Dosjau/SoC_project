#include <stdint.h>
#include <math.h>
#include "motor.h"

struct MotorState initialize_motor(void)
{
    struct MotorState new_motor;

    new_motor.current_d = 0.0f;
    new_motor.current_q = 0.0f;
    new_motor.motor_speed = 0.0f;
    new_motor.motor_angle = 0.0f;

    return new_motor;
}

struct MotorParameters initialize_motor_parameters(void)
{
    struct MotorParameters parameters;

    parameters.stator_resistance = 0.5f;
    parameters.inductance_d      = 0.45e-3f;
    parameters.inductance_q      = 0.45e-3f;
    parameters.flux_linkage      = 0.008667f;
    parameters.pole_pairs        = 7.0f;
    parameters.inertia           = 0.044e-4f;
    parameters.friction          = 0.0f;
    parameters.dc_voltage        = 48.0f;

    return parameters;
}

void calculate_voltages(struct MotorState *state, const struct MotorParameters *parameters, uint8_t control_signal, float *voltage_d, float *voltage_q){
    float switch_a = (control_signal >> 2) & 1;
    float switch_b = (control_signal >> 1) & 1;
    float switch_c =  control_signal       & 1;

    float old_motor_angle = state->motor_angle;
    float dc_voltage = parameters->dc_voltage;

    float voltage_alpha = (2.0f / 3.0f) * dc_voltage * (switch_a - 0.5f * switch_b - 0.5f * switch_c);
    float voltage_beta  = (2.0f / 3.0f) * dc_voltage * 0.8660254f * (switch_b - switch_c);

    float cos_angle = cosf(old_motor_angle);
    float sin_angle = sinf(old_motor_angle);

    *voltage_d =  cos_angle * voltage_alpha + sin_angle * voltage_beta;
    *voltage_q = -sin_angle * voltage_alpha + cos_angle * voltage_beta;
}

void motor_step(struct MotorState *state, const struct MotorParameters *parameters, float voltage_d, float voltage_q, float load_torque, float time_step){
    //Read state
    float old_current_d   = state->current_d;
    float old_current_q   = state->current_q;
    float old_motor_speed = state->motor_speed;
    float old_motor_angle = state->motor_angle;

    //Read parameters
    float stator_resistance = parameters->stator_resistance;
    float inductance_d      = parameters->inductance_d;
    float inductance_q      = parameters->inductance_q;
    float flux_linkage      = parameters->flux_linkage;
    float pole_pairs        = parameters->pole_pairs;
    float inertia           = parameters->inertia;
    float friction          = parameters->friction;

    //Calculate new motor states
    float new_current_d = (1.0f - stator_resistance * time_step / inductance_d) * old_current_d
                    + (inductance_q * time_step / inductance_d) * old_motor_speed * old_current_q
                    + (time_step / inductance_d) * voltage_d;

    float new_current_q = (1.0f - stator_resistance * time_step / inductance_q) * old_current_q
                    - (inductance_d * time_step / inductance_q) * old_motor_speed * old_current_d
                    + (time_step / inductance_q) * voltage_q
                    - (flux_linkage * time_step / inductance_q) * old_motor_speed;

    float electrical_torque = 3.0f/2.0f*pole_pairs*(flux_linkage*old_current_q
                    +(inductance_d-inductance_q)*old_current_d*old_current_q);
    
    float new_motor_speed = (1-friction*time_step/inertia)*old_motor_speed
                    +(pole_pairs*time_step/inertia)*electrical_torque
                    -(pole_pairs*time_step/inertia)*load_torque;

    float new_motor_angle = old_motor_angle + time_step*old_motor_speed;

    //To keep motor angle between 0 and 2*pi
    while (new_motor_angle >= 6.2831853f) new_motor_angle -= 6.2831853f;
    while (new_motor_angle < 0.0f)        new_motor_angle += 6.2831853f;
    
    //Update state
    state->current_d   = new_current_d;
    state->current_q   = new_current_q;
    state->motor_speed = new_motor_speed;
    state->motor_angle = new_motor_angle;
}