#pragma once
#include <stdbool.h>

#define MPC_HORIZON 10
#define MPC_MAX_SWEEPS 100
#define MPC_CONSTRAINTS (3*MPC_HORIZON)
#define MPC_ACTIVE_ITERATIONS 80

typedef enum {
    PREDICTIVE_OK, PREDICTIVE_INVALID, PREDICTIVE_UNSUPPORTED,
    PREDICTIVE_INFEASIBLE, PREDICTIVE_ITERATION_LIMIT, PREDICTIVE_NUMERICAL
} predictive_result_t;

// RPM[k+1] = a1*RPM[k] + a2*RPM[k-1] + b1*u[k] + b2*u[k-1] + c.
// u is PWM percentage, not a fraction. The fitted output includes the existing RPM filter.
typedef struct {
    float a1, a2, b1, b2, c, sample_s, move_weight;
} predictive_model_t;

typedef struct {
    predictive_model_t model;
    float response[MPC_HORIZON][MPC_HORIZON];
    float hessian[MPC_HORIZON][MPC_HORIZON];
    float plan[MPC_HORIZON];
    int sweeps;
    float residual;
    float inverse[MPC_HORIZON][MPC_HORIZON];
    float inverse_normal[MPC_CONSTRAINTS][MPC_HORIZON];
    float predicted_peak, prediction_limit, constraint_violation;
    bool positive_response;
    predictive_result_t result;
} predictive_solver_t;

bool predictive_valid(const predictive_model_t *model);
bool predictive_init(predictive_solver_t *solver, const predictive_model_t *model);
// Box-constrained convex quadratic MPC, cyclic coordinate descent.
// Residual is the largest projected coordinate correction in PWM percentage points.
bool predictive_step(predictive_solver_t *solver, float speed, float previous_speed,
                     float previous_pwm, float disturbance, float target, float maximum,
                     float *pwm);
// Active-set QP with 0<=u<=maximum and every predicted RPM<=target-margin.
// This first constrained variant requires a nonnegative finite-horizon impulse response.
// An infeasible tightened prediction returns false; it never relaxes the RPM bound.
bool predictive_step_capped(predictive_solver_t *solver, float speed, float previous_speed,
                           float previous_pwm, float disturbance, float target, float maximum,
                           float margin_rpm, float *pwm);
