#include <stdint.h>

struct MotorState {
    float current_d;
    float current_q;
    float motor_speed;
    float motor_angle;
};

struct MotorParameters {
    float stator_resistance;
    float inductance_d;
    float inductance_q;
    float flux_linkage;
    float pole_pairs;
    float inertia;
    float friction;
    float dc_voltage;
};

struct MotorParameters initialize_motor_parameters(void);
struct MotorState initialize_motor(void);
void calculate_voltages(struct MotorState *state, const struct MotorParameters *parameters, uint8_t control_signal, float *voltage_d, float *voltage_q);
void motor_step(struct MotorState *state, const struct MotorParameters *parameters, float voltage_d, float voltage_q, float load_torque, float time_step);