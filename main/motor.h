#pragma once

#include <stdbool.h>
#include <stddef.h>

typedef struct {
    bool active;
    float target_rpm;
    float c1_rpm, c2_rpm;
    float pwm_percent, elapsed;
    int seconds, encoder_sign;
    const char *reason;
    float requested_rpm, position_turns, goal_turns;
    float p_term, i_term, d_term, ff_term, kp, ki, kd, max_pwm;
    bool position_mode, saturated, antiwindup;
    const char *controller;
} motor_status_t;

motor_status_t motor_get_status(void);

// TB6612FNG A channel: PWMA=GPIO4, AIN1=GPIO5, AIN2=GPIO6, STBY=GPIO7.
void motor_init(void);
bool motor_command(const char *line);
void motor_poll(void);
int motor_pi_status(char *buffer, size_t size);
int motor_lab_status(char *buffer, size_t size);
int motor_predictive_status(char *buffer, size_t size);
int motor_constraint_status(char *buffer, size_t size);
int motor_sample_batch(char *buffer, size_t size);
int motor_sample_batch2(char *buffer, size_t size);
