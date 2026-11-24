main: main.c arm/motor.c arm/motor.h
		gcc main.c arm/motor.c -o main -lm