#include <stdio.h>
#include "arm/motor.h"

int main() {
    int motor_speed = motor(1);
    printf("Motor speed: %d\n", motor_speed);
    return 0;
}