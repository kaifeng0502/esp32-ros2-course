#include <string.h>
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "motor.h"
#include "encoder.h"
#include "predictive.h"
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <math.h>
#include <ctype.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define MOTOR_PWM GPIO_NUM_4
#define MOTOR_IN1 GPIO_NUM_5
#define MOTOR_IN2 GPIO_NUM_6
#define MOTOR_STBY GPIO_NUM_7

// Change these two numbers in VS Code, then build and flash again.
#define MOTOR_SPEED_PERCENT 80  // 1-100: PWM power percentage.
#define MOTOR_TEST_MS 2000       // Test duration in milliseconds.
#define MOTOR_DUTY (MOTOR_SPEED_PERCENT * 1023 / 100)
#define MOTOR_TEST_US (MOTOR_TEST_MS * 1000)

// -------- DEMO 参数（可在 VS Code 里改，然后重新编译烧录）--------
#define DEMO_RAMP_UP_MS   3000   // 慢 -> 快 的时间
#define DEMO_HOLD_MS       600   // 保持最快的时间
#define DEMO_RAMP_DOWN_MS 2000   // 快 -> 慢 的时间
#define DEMO_SETTLE_MS     800   // 慢速“定住”的时间
#define DEMO_PAUSE_MS      600   // 反转前的停顿（电机停转）
#define DEMO_STOP_MS      1200   // 最后一段：慢 -> 停
#define DEMO_MIN_PCT        20   // “慢速”对应的 PWM 百分比（起步/稳住）
#define DEMO_MAX_PCT       100   // “快速”对应的 PWM 百分比（全速）

#if MOTOR_SPEED_PERCENT < 1 || MOTOR_SPEED_PERCENT > 100
#error MOTOR_SPEED_PERCENT must be between 1 and 100
#endif
#if MOTOR_TEST_MS < 1 || MOTOR_TEST_MS > 3000
#error MOTOR_TEST_MS must be between 1 and 3000 for this brief test
#endif

static const char *TAG = "motor";
static int64_t stop_at_us;
static SemaphoreHandle_t motor_lock;
static void motor_task(void *arg);

// PWM units are percent; Ki is percent / (RPM * second).
#define PI_DEFAULT_RPM 120.0f
#define PI_COUNTS_PER_REV ENCODER_COUNTS_PER_REV
#define PI_KP 0.15f
#define PI_KI 0.30f
#define PI_FEEDFORWARD 50.0f
#define PI_MAX_PWM 80.0f
#define PI_INTEGRAL_LIMIT 25.0f
static float pi_target = PI_DEFAULT_RPM;
static int pi_duration_seconds = 15;
static float pi_feedforward = PI_FEEDFORWARD;
#define PI_SAMPLE_US 50000LL
static bool pi_active, pi_waiting;
static int pi_direction = 1;
static int pi_encoder_sign = 1;
static int64_t pi_wait_started, pi_quiet_since;
static int64_t pi_previous_signed[2];
static float pi_signed_rpm[2];
static int64_t pi_started, pi_sampled, pi_edge_at[2];
static uint32_t pi_previous[2];
static float pi_rpm[2], pi_integral, pi_pwm, pi_elapsed;
static const char *pi_reason = "IDLE";

typedef enum { CONTROL_OPEN, CONTROL_P, CONTROL_PI, CONTROL_PID, CONTROL_ID, CONTROL_MPC, CONTROL_MPC_CAP } controller_t;
typedef struct {
    controller_t controller;
    float turns, kp, ki, kd, max_pwm, step_rpm, step_at;
    bool antiwindup;
} lab_config_t;
static lab_config_t lab = { .controller=CONTROL_PI, .kp=PI_KP, .ki=PI_KI,
                            .kd=0.005f, .max_pwm=PI_MAX_PWM, .antiwindup=true };
static float requested_rpm = PI_DEFAULT_RPM, position_turns, effective_target;
static float p_term, d_term, derivative, previous_speed;
static bool saturated, braking, stepped;
static int64_t position_start[2], brake_started, brake_quiet;
static uint32_t brake_previous[2];
static predictive_model_t predictive_model;
static predictive_solver_t predictive_solver;
static bool model_ready, tracing;
static bool cap_configured;
static float cap_margin_rpm=1;
static unsigned experiment_id, trace_sequence, trace_dropped;
static float model_last_y, model_old_y, model_old_u, model_disturbance;
static int model_samples, solve_sweeps;
static int64_t solve_us;
static float solve_residual;
#define TRACE_CAPACITY 48
typedef struct {
    unsigned sequence;
    int64_t at_us;
    float elapsed, dt, target, c1, c2, held_pwm, next_pwm;
    int64_t solve_us;
    int sweeps;
    float residual, disturbance;
    float prediction_limit, predicted_peak, constraint_violation;
    const char *phase;
} trace_sample_t;
static trace_sample_t trace_samples[TRACE_CAPACITY];
static unsigned trace_read, trace_write;
static void trace_record(float dt, float held_pwm, int64_t now);
static const char *controller_name(void)
{
    static const char *names[] = {"OPEN", "P", "PI", "PID", "ID", "MPC", "MPC_CAP"};
    return names[lab.controller];
}

static float clampf(float value, float low, float high)
{
    return value < low ? low : value > high ? high : value;
}

typedef enum {
    DIR_STOP = 0,
    DIR_FWD,
    DIR_REV,
} motor_dir_t;

typedef struct {
    motor_dir_t dir;
    int start_pct;
    int end_pct;
    int duration_ms;
} motor_segment_t;

// 一次完整 DEMO 的动作序列：正转慢->快->慢->定住 -> 停顿 -> 反转慢->快->慢->停。
static const motor_segment_t demo[] = {
    { DIR_FWD,  DEMO_MIN_PCT, DEMO_MAX_PCT, DEMO_RAMP_UP_MS   },
    { DIR_FWD,  DEMO_MAX_PCT, DEMO_MAX_PCT, DEMO_HOLD_MS      },
    { DIR_FWD,  DEMO_MAX_PCT, DEMO_MIN_PCT, DEMO_RAMP_DOWN_MS },
    { DIR_FWD,  DEMO_MIN_PCT, DEMO_MIN_PCT, DEMO_SETTLE_MS    },
    { DIR_STOP, 0,            0,            DEMO_PAUSE_MS     },
    { DIR_REV,  DEMO_MIN_PCT, DEMO_MAX_PCT, DEMO_RAMP_UP_MS   },
    { DIR_REV,  DEMO_MAX_PCT, DEMO_MAX_PCT, DEMO_HOLD_MS      },
    { DIR_REV,  DEMO_MAX_PCT, DEMO_MIN_PCT, DEMO_RAMP_DOWN_MS },
    { DIR_REV,  DEMO_MIN_PCT, 0,            DEMO_STOP_MS      },
};
#define DEMO_SEGMENTS ((int)(sizeof(demo) / sizeof(demo[0])))

static int demo_segment = -1;              // -1 = 未运行
static int64_t demo_segment_start_us;

static const char *dir_name(motor_dir_t d)
{
    switch (d) {
        case DIR_FWD: return "FWD";
        case DIR_REV: return "REV";
        default:      return "STOP";
    }
}

// 设定方向与 PWM 占空比，并把 STBY 拉高。停止/零占空比时把 STBY 拉低。
static void motor_apply(motor_dir_t dir, int duty)
{
    if (dir == DIR_STOP || duty <= 0) {
        gpio_set_level(MOTOR_STBY, 0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        gpio_set_level(MOTOR_IN1, 0);
        gpio_set_level(MOTOR_IN2, 0);
        return;
    }
    gpio_set_level(MOTOR_IN1, dir == DIR_FWD ? 1 : 0);
    gpio_set_level(MOTOR_IN2, dir == DIR_REV ? 1 : 0);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    gpio_set_level(MOTOR_STBY, 1);
}

static void motor_stop(void)
{
    bool final_sample=pi_active && tracing;
    if (pi_active) {
        if (!pi_waiting) pi_elapsed = (esp_timer_get_time() - pi_started) / 1000000.0f;
        pi_reason = "STOPPED";
    }
    braking = false;
    pi_active = false;
    pi_waiting = false;
    pi_pwm = 0;
    // STBY is lowered first so direction and PWM changes cannot drive the motor.
    gpio_set_level(MOTOR_STBY, 0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    gpio_set_level(MOTOR_IN1, 0);
    gpio_set_level(MOTOR_IN2, 0);
    stop_at_us = 0;
    demo_segment = -1;
    if (final_sample) trace_record(0,0,esp_timer_get_time());
}

static void trace_record(float dt, float held_pwm, int64_t now)
{
    if (!tracing) return;
    if (trace_write-trace_read >= TRACE_CAPACITY) {
        ++trace_dropped;
        ++trace_sequence;
        return;
    }
    bool model_trace=pi_active && (lab.controller==CONTROL_MPC || lab.controller==CONTROL_MPC_CAP);
    bool cap_trace=model_trace && lab.controller==CONTROL_MPC_CAP;
    trace_samples[trace_write++ % TRACE_CAPACITY]=(trace_sample_t){
        .sequence=trace_sequence++, .at_us=now, .elapsed=pi_elapsed, .dt=dt,
        .target=pi_target, .c1=pi_rpm[0], .c2=pi_rpm[1], .held_pwm=held_pwm,
        .next_pwm=pi_pwm, .solve_us=solve_us, .sweeps=solve_sweeps,
        .residual=isfinite(solve_residual) ? solve_residual : -1, .disturbance=model_disturbance,
        .prediction_limit=cap_trace ? predictive_solver.prediction_limit : -1,
        .predicted_peak=model_trace ? predictive_solver.predicted_peak : -1,
        .constraint_violation=cap_trace ? predictive_solver.constraint_violation : -1,
        .phase=pi_active ? pi_reason : "STOPPED",
    };
}

static float identification_pwm(float elapsed)
{
    static const float levels[]={30,45,35,55,40,50,30,55,35};
    int index=(int)(elapsed/2);
    return levels[index<9 ? index : 8];
}

static bool model_controller(void)
{
    return lab.controller==CONTROL_MPC || lab.controller==CONTROL_MPC_CAP;
}

static bool solve_predictive(float speed,float old_speed,float held_pwm)
{
    int64_t before=esp_timer_get_time();
    bool solved;
    if (lab.controller==CONTROL_MPC_CAP)
        solved=predictive_step_capped(&predictive_solver,speed,old_speed,held_pwm,
                    model_disturbance,pi_target,lab.max_pwm,cap_margin_rpm,&pi_pwm);
    else solved=predictive_step(&predictive_solver,speed,old_speed,held_pwm,
                    model_disturbance,pi_target,lab.max_pwm,&pi_pwm);
    solve_us=esp_timer_get_time()-before;
    solve_sweeps=predictive_solver.sweeps; solve_residual=predictive_solver.residual;
    if (!solved || solve_us>10000) {
        const char *reason=solved ? "MPC_OVERRUN" : "MPC_NOT_CONVERGED";
        if (!solved && lab.controller==CONTROL_MPC_CAP) {
            if (predictive_solver.result==PREDICTIVE_INFEASIBLE) reason="MPC_INFEASIBLE";
            else if (predictive_solver.result==PREDICTIVE_UNSUPPORTED) reason="MPC_UNSUPPORTED";
            else if (predictive_solver.result==PREDICTIVE_NUMERICAL) reason="MPC_NUMERICAL";
        }
        motor_stop();pi_reason=reason;return false;
    }
    return true;
}

static void position_brake(const encoder_snapshot_t *enc, int64_t now)
{
    // Toshiba TB6612FNG H-SW table: IN1=IN2=H, STBY=H -> short brake.
    // https://toshiba.semicon-storage.com/info/docget.jsp?did=10660 (page 4)
    gpio_set_level(MOTOR_STBY, 0);
    gpio_set_level(MOTOR_IN1, 1);
    gpio_set_level(MOTOR_IN2, 1);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
    gpio_set_level(MOTOR_STBY, 1);
    pi_pwm = 0;
    braking = true;
    brake_started = brake_quiet = now;
    brake_previous[0] = enc->c1_edges; brake_previous[1] = enc->c2_edges;
    pi_reason = "BRAKING";
}

static void demo_start(void)
{
    demo_segment = 0;
    demo_segment_start_us = esp_timer_get_time();
    ESP_LOGI(TAG, "Demo started: slow -> fast -> slow -> settle -> reverse");
}

void motor_init(void)
{
    motor_lock = xSemaphoreCreateMutex();
    configASSERT(motor_lock);
    // The TB6612 STBY pin has a pull-down. Explicitly keep all controls low.
    const gpio_config_t io = {
        .pin_bit_mask = (1ULL << MOTOR_IN1) | (1ULL << MOTOR_IN2) | (1ULL << MOTOR_STBY),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    gpio_set_level(MOTOR_STBY, 0);
    gpio_set_level(MOTOR_IN1, 0);
    gpio_set_level(MOTOR_IN2, 0);

    const ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));
    const ledc_channel_config_t channel = {
        .gpio_num = MOTOR_PWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
    motor_stop();
    BaseType_t created = xTaskCreate(motor_task, "motor_control", 4096, NULL, 8, NULL);
    configASSERT(created == pdPASS);
    ESP_LOGI(TAG, "Motor ready; commands: MOTOR TEST | MOTOR DEMO | MOTOR STOP");
}

// Parse complete integer tokens, including range checks before narrowing.
static bool parse_integer(const char *text, int low, int high, int *result)
{
    char *end;
    errno = 0;
    long value = strtol(text, &end, 10);
    if (errno || end == text || *end || value < low || value > high) return false;
    *result = (int)value;
    return true;
}

static bool parse_float(const char *text, float low, float high, float *result)
{
    char *end;
    errno = 0;
    float value = strtof(text, &end);
    if (errno || end == text || *end || !isfinite(value) || value < low || value > high) return false;
    *result = value;
    return true;
}

static int tokens(const char *line, char out[][20], int limit)
{
    int n = 0;
    while (*line) {
        while (isspace((unsigned char)*line)) ++line;
        if (!*line) break;
        if (n == limit) return -1;
        int length = 0;
        while (*line && !isspace((unsigned char)*line)) {
            if (length == 19) return -1;
            out[n][length++] = *line++;
        }
        out[n++][length] = 0;
    }
    return n;
}

static void start_run(int target, int seconds, int direction, int encoder_sign, lab_config_t config)
{
    motor_stop();
    tracing=false;
    lab = config;
    pi_target = requested_rpm = effective_target = target;
    pi_duration_seconds = seconds;
    pi_direction = direction;
    pi_encoder_sign = encoder_sign;
    pi_feedforward = PI_FEEDFORWARD * pi_target / PI_DEFAULT_RPM;
    if (lab.controller==CONTROL_ID || model_controller()) pi_feedforward=0;
    encoder_snapshot_t enc = encoder_snapshot();
    pi_previous[0] = enc.c1_edges; pi_previous[1] = enc.c2_edges;
    pi_previous_signed[0] = enc.c1_signed; pi_previous_signed[1] = enc.c2_signed;
    position_start[0] = enc.c1_signed; position_start[1] = enc.c2_signed;
    pi_wait_started = pi_quiet_since = esp_timer_get_time();
    pi_rpm[0] = pi_rpm[1] = pi_signed_rpm[0] = pi_signed_rpm[1] = 0;
    pi_integral = pi_elapsed = pi_pwm = position_turns = 0;
    p_term = d_term = derivative = previous_speed = 0;
    saturated = stepped = false;
    pi_reason = "WAITING";
    pi_active = pi_waiting = true;
}

static bool motor_command_locked(const char *line)
{
    int target = 120, seconds = 15, direction = 1, encoder_sign = 1;
    if (strncmp(line,"MOTOR CAP ",10)==0) {
        if (pi_active || demo_segment>=0 || stop_at_us) return false;
        char field[1][20];float margin;
        if (tokens(line+10,field,1)!=1 || !parse_float(field[0],0,5,&margin)) return false;
        cap_margin_rpm=margin;cap_configured=true;return true;
    }
    if (strncmp(line,"MOTOR MODEL ",12)==0) {
        if (pi_active || demo_segment>=0 || stop_at_us) return false;
        char field[7][20];
        predictive_model_t m;
        if (tokens(line+12,field,7)!=7 ||
            !parse_float(field[0],-1.999f,1.999f,&m.a1) ||
            !parse_float(field[1],-.999f,.999f,&m.a2) ||
            !parse_float(field[2],0,20,&m.b1) ||
            !parse_float(field[3],-20,20,&m.b2) ||
            !parse_float(field[4],-100,100,&m.c) ||
            !parse_float(field[5],.04f,.08f,&m.sample_s) ||
            !parse_float(field[6],.05f,100,&m.move_weight) || !predictive_valid(&m)) return false;
        predictive_model=m; model_ready=true;
        return true; // Volatile configuration only; never starts the drive.
    }
    if (strncmp(line,"MOTOR EXP ",10)==0) {
        char field[10][20]; int maximum;
        lab_config_t config={.antiwindup=true};
        if (pi_active || demo_segment>=0 || stop_at_us || tokens(line+10,field,10)!=10 ||
            !parse_integer(field[1],40,180,&target) ||
            !parse_integer(field[2],1,30,&seconds) ||
            !parse_integer(field[3],-1,1,&encoder_sign) || !encoder_sign ||
            !parse_float(field[4],0,5,&config.kp) ||
            !parse_float(field[5],0,10,&config.ki) ||
            !parse_float(field[6],0,1,&config.kd) ||
            !parse_integer(field[7],55,60,&maximum) ||
            !parse_float(field[8],0,180,&config.step_rpm) ||
            !parse_float(field[9],0,29,&config.step_at)) return false;
        config.max_pwm=maximum;
        if (strcmp(field[0],"ID")==0) {
            if (seconds!=18 || config.step_rpm || config.step_at) return false;
            config.controller=CONTROL_ID;
        } else if (strcmp(field[0],"PID")==0) config.controller=CONTROL_PID;
        else if (strcmp(field[0],"MPC")==0 || strcmp(field[0],"MPC_CAP")==0) {
            if (!model_ready || !predictive_init(&predictive_solver,&predictive_model)) return false;
            if (strcmp(field[0],"MPC_CAP")==0) {
                if (!cap_configured || !predictive_solver.positive_response) return false;
                config.controller=CONTROL_MPC_CAP;
            } else config.controller=CONTROL_MPC;
        } else return false;
        if (config.step_rpm>0) {
            if (config.step_rpm<40 || config.step_at<1 || config.step_at>=seconds) return false;
        } else if (config.step_at!=0) return false;
        start_run(target,seconds,1,encoder_sign,config);
        tracing=true; ++experiment_id;
        trace_read=trace_write=trace_sequence=trace_dropped=0;
        model_last_y=model_old_y=model_old_u=model_disturbance=0;
        model_samples=solve_sweeps=0; solve_us=0; solve_residual=0;
        return true;
    }
    if (strncmp(line, "MOTOR LAB ", 10) == 0) {
        char field[13][20];
        lab_config_t config = {0};
        int aw;
        if (tokens(line + 10, field, 13) != 13 ||
            !parse_integer(field[0], 40, 180, &target) ||
            !parse_integer(field[1], 1, 60, &seconds) ||
            !parse_integer(field[3], -1, 1, &encoder_sign) || !encoder_sign ||
            !parse_float(field[5], 0, 100, &config.turns) ||
            !parse_float(field[6], 0, 5, &config.kp) ||
            !parse_float(field[7], 0, 10, &config.ki) ||
            !parse_float(field[8], 0, 1, &config.kd) ||
            !parse_integer(field[9], 0, 1, &aw) ||
            !parse_float(field[10], 20, 80, &config.max_pwm) ||
            !parse_float(field[11], 0, 180, &config.step_rpm) ||
            !parse_float(field[12], 0, 59, &config.step_at)) return false;
        if (strcmp(field[2], "FWD") == 0) direction = 1;
        else if (strcmp(field[2], "REV") == 0) direction = -1;
        else return false;
        if (strcmp(field[4], "OPEN") == 0) config.controller = CONTROL_OPEN;
        else if (strcmp(field[4], "P") == 0) config.controller = CONTROL_P;
        else if (strcmp(field[4], "PI") == 0) config.controller = CONTROL_PI;
        else if (strcmp(field[4], "PID") == 0) config.controller = CONTROL_PID;
        else return false;
        config.antiwindup = aw;
        if (config.turns > 0 && (config.turns < 5.0f/360 || config.controller != CONTROL_PI)) return false;
        if (config.step_rpm > 0) {
            if (config.step_rpm < 40 || config.step_at < 0.5f || config.step_at >= seconds || config.turns > 0) return false;
        } else if (config.step_at != 0) return false;
        start_run(target, seconds, direction, encoder_sign, config);
        return true;
    }
    bool legacy = strcmp(line, "MOTOR PI120") == 0;
    bool parameterized = strncmp(line, "MOTOR PI ", 9) == 0;
    if (parameterized) {
        char rpm_text[16], seconds_text[16], dir_text[8], sign_text[8], extra;
        int fields = sscanf(line, "MOTOR PI %15s %15s %7s %7s %c",
                            rpm_text, seconds_text, dir_text, sign_text, &extra);
        if ((fields != 2 && fields != 4) ||
            !parse_integer(rpm_text, 40, 180, &target) ||
            !parse_integer(seconds_text, 1, 60, &seconds)) return false;
        if (fields == 4) {
            if (strcmp(dir_text, "FWD") == 0) direction = 1;
            else if (strcmp(dir_text, "REV") == 0) direction = -1;
            else return false;
            if (!parse_integer(sign_text, -1, 1, &encoder_sign) || !encoder_sign) return false;
        }
    }
    if (legacy || parameterized) {
        lab_config_t config = { .controller=CONTROL_PI, .kp=PI_KP, .ki=PI_KI,
                                .kd=0.005f, .max_pwm=PI_MAX_PWM, .antiwindup=true };
        start_run(target, seconds, direction, encoder_sign, config);
        return true;
    }
    if (strcmp(line, "MOTOR STOP") == 0) {
        motor_stop();
        return true;
    }
    if (strcmp(line, "MOTOR TEST") == 0) {
        motor_stop();
        gpio_set_level(MOTOR_IN1, 1);
        gpio_set_level(MOTOR_IN2, 0);
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, MOTOR_DUTY));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
        stop_at_us = esp_timer_get_time() + MOTOR_TEST_US;
        gpio_set_level(MOTOR_STBY, 1);
        ESP_LOGI(TAG, "Motor test started for %d ms at %d%% PWM", MOTOR_TEST_MS, MOTOR_SPEED_PERCENT);
        return true;
    }
    if (strcmp(line, "MOTOR DEMO") == 0) {
        motor_stop();
        demo_start();
        return true;
    }
    return false;
}

static void motor_poll_locked(void)
{
    const int64_t now = esp_timer_get_time();

    if (pi_active && pi_waiting) {
        encoder_snapshot_t enc = encoder_snapshot();
        if (enc.c1_edges != pi_previous[0] || enc.c2_edges != pi_previous[1]) {
            pi_quiet_since = now;
        }
        pi_previous[0] = enc.c1_edges;
        pi_previous[1] = enc.c2_edges;
        pi_previous_signed[0] = enc.c1_signed;
        pi_previous_signed[1] = enc.c2_signed;
        if (now - pi_wait_started >= 3000000) {
            motor_stop();
            pi_reason = "NOT_STOPPED";
        } else if (now - pi_quiet_since >= 500000) {
            position_start[0] = enc.c1_signed; position_start[1] = enc.c2_signed;
            pi_started = pi_sampled = now;
            pi_edge_at[0] = pi_edge_at[1] = now;
            pi_waiting = false;
            pi_reason = "RUNNING";
            if (lab.turns > 0) {
                pi_target = effective_target = clampf(60.0f * sqrtf(8.0f * lab.turns), 25, requested_rpm);
                pi_feedforward = PI_FEEDFORWARD * pi_target / PI_DEFAULT_RPM;
            }
            pi_pwm = clampf(pi_feedforward, 0, lab.max_pwm);
            if (lab.controller==CONTROL_ID) pi_pwm=identification_pwm(0);
            if (model_controller() && !solve_predictive(0,0,0)) return;
            motor_apply(pi_direction > 0 ? DIR_FWD : DIR_REV, (int)(pi_pwm * 1023 / 100));
            trace_record(0,0,now);
        }
        return;
    }
    if (pi_active) {
        pi_elapsed = (now - pi_started) / 1000000.0f;
        encoder_snapshot_t enc = encoder_snapshot();
        position_turns = pi_encoder_sign *
            ((enc.c1_signed - position_start[0]) + (enc.c2_signed - position_start[1])) /
            (2.0f * PI_COUNTS_PER_REV);
        if (now - pi_started >= (int64_t)pi_duration_seconds * 1000000) {
            motor_stop();
            pi_reason = lab.turns > 0 ? "TIMEOUT" : "DONE";
            return;
        }
        if (braking) {
            if (enc.c1_edges != brake_previous[0] || enc.c2_edges != brake_previous[1]) brake_quiet = now;
            brake_previous[0] = enc.c1_edges; brake_previous[1] = enc.c2_edges;
            if (now - brake_quiet >= 150000) {
                motor_stop(); pi_reason = "DONE";
            } else if (now - brake_started >= 700000) {
                motor_stop(); pi_reason = "BRAKE_TIMEOUT";
            }
            return;
        }
        if (lab.turns > 0) {
            float progress = pi_direction * position_turns;
            float remaining = lab.turns - progress;
            if (progress < -0.01f && now - pi_started > 200000) {
                motor_stop(); pi_reason = "DIRECTION_ERROR";
                return;
            }
            // Position is checked at 10 ms; speed control remains at 50 ms.
            if (remaining <= 2.0f / PI_COUNTS_PER_REV) {
                position_brake(&enc, now);
                return;
            }
            // v^2 = 2*a*distance, a=4 output-shaft revolutions/s^2.
            effective_target = clampf(60.0f * sqrtf(8.0f * remaining), 25, requested_rpm);
            pi_reason = effective_target < requested_rpm - 1 ? "DECEL" : "RUNNING";
        } else {
            if (lab.step_rpm > 0 && !stepped && pi_elapsed >= lab.step_at) {
                requested_rpm = lab.step_rpm;
                stepped = true;
            }
            effective_target = requested_rpm;
        }
        pi_target = effective_target;
        if (now - pi_sampled < PI_SAMPLE_US) return;
        float dt = (now - pi_sampled) / 1000000.0f;
        float held_pwm=((int)(pi_pwm*1023/100))*100.0f/1023;
        uint32_t values[2] = { enc.c1_edges, enc.c2_edges };
        int64_t signed_values[2] = { enc.c1_signed, enc.c2_signed };
        for (int i = 0; i < 2; ++i) {
            uint32_t delta = values[i] - pi_previous[i];
            pi_previous[i] = values[i];
            if (delta) pi_edge_at[i] = now;
            float measured = delta * 60.0f / (PI_COUNTS_PER_REV * dt);
            float alpha = dt / (0.10f + dt);
            pi_rpm[i] += alpha * (measured - pi_rpm[i]);
            int64_t signed_delta = signed_values[i] - pi_previous_signed[i];
            pi_previous_signed[i] = signed_values[i];
            float signed_measured = pi_encoder_sign * signed_delta * 60.0f / (PI_COUNTS_PER_REV * dt);
            pi_signed_rpm[i] += alpha * (signed_measured - pi_signed_rpm[i]);
        }
        pi_sampled = now;
        // Permit startup, then stop if either channel loses all edges for 0.5 s.
        if (now - pi_started >= 800000 &&
            (now - pi_edge_at[0] >= 500000 || now - pi_edge_at[1] >= 500000)) {
            motor_stop();
            pi_reason = "NO_FEEDBACK";
            return;
        }
        float actual = (pi_rpm[0] + pi_rpm[1]) * 0.5f;
        if (tracing && pi_elapsed>.25f &&
            (pi_signed_rpm[0]+pi_signed_rpm[1])*.5f < -10) {
            motor_stop(); pi_reason="DIRECTION_ERROR"; return;
        }
        if (lab.controller==CONTROL_ID || model_controller()) {
            if (lab.controller==CONTROL_ID) pi_pwm=identification_pwm(pi_elapsed);
            else {
                if (dt<predictive_model.sample_s*.5f || dt>predictive_model.sample_s*1.5f) {
                    motor_stop(); pi_reason="MPC_TIMING_ERROR"; return;
                }
                if (model_samples>=2) {
                    float prediction=predictive_model.a1*model_last_y+predictive_model.a2*model_old_y+
                        predictive_model.b1*held_pwm+predictive_model.b2*model_old_u+predictive_model.c;
                    model_disturbance=clampf(.8f*model_disturbance+.2f*(actual-prediction),-10,10);
                }
                if (!solve_predictive(actual,model_last_y,held_pwm)) return;
                model_old_y=model_last_y; model_last_y=actual;
                model_old_u=held_pwm; ++model_samples;
            }
            motor_apply(DIR_FWD,(int)(pi_pwm*1023/100));
            trace_record(dt,held_pwm,now);
            return;
        }
        float error = pi_target - actual;
        pi_feedforward = PI_FEEDFORWARD * pi_target / PI_DEFAULT_RPM;
        // D acts on measured speed, filtered to reduce encoder quantization noise.
        float raw_derivative = (actual - previous_speed) / dt;
        derivative += dt / (0.10f + dt) * (raw_derivative - derivative);
        previous_speed = actual;
        p_term = lab.controller == CONTROL_OPEN ? 0 : lab.kp * error;
        d_term = lab.controller == CONTROL_PID ? -lab.kd * derivative : 0;
        if (lab.controller == CONTROL_PI || lab.controller == CONTROL_PID) {
            float candidate = clampf(pi_integral + lab.ki * error * dt,
                                     -PI_INTEGRAL_LIMIT, PI_INTEGRAL_LIMIT);
            float requested = pi_feedforward + p_term + candidate + d_term;
            if (!lab.antiwindup || (requested >= 0 && requested <= lab.max_pwm) ||
                (requested > lab.max_pwm && error < 0) || (requested < 0 && error > 0)) {
                pi_integral = candidate;
            }
        } else pi_integral = 0;
        float requested = pi_feedforward + p_term + pi_integral + d_term;
        saturated = requested < 0 || requested > lab.max_pwm;
        pi_pwm = clampf(requested, 0, lab.max_pwm);
        motor_apply(pi_direction > 0 ? DIR_FWD : DIR_REV, (int)(pi_pwm * 1023 / 100));
        trace_record(dt,held_pwm,now);
        return;
    }

    if (stop_at_us && now >= stop_at_us) {
        motor_stop();
        ESP_LOGI(TAG, "Motor test stopped automatically");
    }

    if (demo_segment < 0) return;

    while (demo_segment < DEMO_SEGMENTS) {
        const motor_segment_t *seg = &demo[demo_segment];
        const int64_t duration_us = (int64_t)seg->duration_ms * 1000;
        const int64_t elapsed = now - demo_segment_start_us;
        if (elapsed < duration_us) {
            const int pct = seg->start_pct
                          + (int)((seg->end_pct - seg->start_pct) * elapsed / duration_us);
            motor_apply(seg->dir, pct * 1023 / 100);
            return;
        }
        // 本段结束，进入下一段；用 += 保持整段总时长精确。
        demo_segment++;
        demo_segment_start_us += duration_us;
        if (demo_segment < DEMO_SEGMENTS) {
            const motor_segment_t *n = &demo[demo_segment];
            ESP_LOGI(TAG, "Demo %d/%d: %s %d%% -> %d%% over %d ms",
                     demo_segment + 1, DEMO_SEGMENTS, dir_name(n->dir),
                     n->start_pct, n->end_pct, n->duration_ms);
        }
    }
    motor_stop();
    ESP_LOGI(TAG, "Demo finished; motor stopped");
}

bool motor_command(const char *line)
{
    xSemaphoreTake(motor_lock, portMAX_DELAY);
    bool handled = motor_command_locked(line);
    xSemaphoreGive(motor_lock);
    return handled;
}

void motor_poll(void)
{
    xSemaphoreTake(motor_lock, portMAX_DELAY);
    motor_poll_locked();
    xSemaphoreGive(motor_lock);
}

static void motor_task(void *arg)
{
    TickType_t wake = xTaskGetTickCount();
    while (true) {
        motor_poll();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(10));
    }
}

motor_status_t motor_get_status(void)
{
    xSemaphoreTake(motor_lock, portMAX_DELAY);
    motor_status_t status = {
        .active = pi_active,
        .target_rpm = pi_target * pi_direction,
        .c1_rpm = pi_signed_rpm[0], .c2_rpm = pi_signed_rpm[1],
        .pwm_percent = pi_pwm, .elapsed = pi_elapsed,
        .seconds = pi_duration_seconds, .encoder_sign = pi_encoder_sign,
        .reason = pi_reason,
        .requested_rpm = requested_rpm * pi_direction,
        .position_turns = position_turns, .goal_turns = lab.turns * pi_direction,
        .p_term = p_term, .i_term = pi_integral, .d_term = d_term,
        .ff_term = pi_feedforward, .kp = lab.kp, .ki = lab.ki, .kd = lab.kd,
        .max_pwm = lab.max_pwm, .position_mode = lab.turns > 0,
        .saturated = saturated, .antiwindup = lab.antiwindup,
        .controller = controller_name(),
    };
    xSemaphoreGive(motor_lock);
    return status;
}

int motor_pi_status(char *buffer, size_t size)
{
    motor_status_t status = motor_get_status();
    return snprintf(buffer, size, "PI3 %d %.1f %.2f %.2f %.2f %.3f %s %d %d\n",
                    status.active, status.target_rpm, status.c1_rpm, status.c2_rpm,
                    status.pwm_percent, status.elapsed, status.reason,
                    status.seconds, status.encoder_sign);
}

int motor_lab_status(char *buffer, size_t size)
{
    motor_status_t s = motor_get_status();
    return snprintf(buffer, size,
        "LAB1 %d %.1f %.2f %.2f %.2f %.2f %.3f %s %d %d %s %d %.6f %.6f %.3f %.3f %.3f %.3f %d %.4f %.4f %.4f %d %.4f\n",
        s.active, s.requested_rpm, s.target_rpm, s.c1_rpm, s.c2_rpm,
        s.pwm_percent, s.elapsed, s.reason, s.seconds, s.encoder_sign, s.controller,
        s.position_mode, s.goal_turns, s.position_turns,
        s.p_term, s.i_term, s.d_term, s.ff_term, s.saturated,
        s.kp, s.ki, s.kd, s.antiwindup, s.max_pwm);
}

int motor_predictive_status(char *buffer, size_t size)
{
    xSemaphoreTake(motor_lock,portMAX_DELAY);
    int n=snprintf(buffer,size,
        "MPC1 %u %d %d %s %s %u %u %u %.8g %.8g %.8g %.8g %.8g %.8g %.8g\n",
        experiment_id,model_ready,pi_active,controller_name(),pi_reason,
        trace_sequence,trace_dropped,trace_write-trace_read,
        predictive_model.a1,predictive_model.a2,predictive_model.b1,predictive_model.b2,
        predictive_model.c,predictive_model.sample_s,predictive_model.move_weight);
    xSemaphoreGive(motor_lock);
    return n;
}

int motor_constraint_status(char *buffer, size_t size)
{
    xSemaphoreTake(motor_lock,portMAX_DELAY);
    int n=snprintf(buffer,size,"MPC2 %d %.4f %d 2 %d\n",cap_configured,cap_margin_rpm,
                   MPC_HORIZON,predictive_solver.result);
    xSemaphoreGive(motor_lock);
    return n;
}

static int sample_batch(char *buffer, size_t size, bool extended)
{
    xSemaphoreTake(motor_lock,portMAX_DELAY);
    size_t used=0;
    for (int i=0; i<8 && trace_read<trace_write; i++) {
        trace_sample_t *s=&trace_samples[trace_read % TRACE_CAPACITY];
        char row[256];
        int n=snprintf(row,sizeof(row),
            "OBS%d %u %u %lld %.6f %.6f %.3f %.4f %.4f %.5f %.5f %lld %d %.6f %.6f %s",
            extended ? 2 : 1,
            experiment_id,s->sequence,(long long)s->at_us,s->elapsed,s->dt,s->target,
            s->c1,s->c2,s->held_pwm,s->next_pwm,(long long)s->solve_us,
            s->sweeps,s->residual,s->disturbance,s->phase);
        if (n<=0 || n>=sizeof(row)) break;
        int extra=extended ? snprintf(row+n,sizeof(row)-n," %.5f %.5f %.6f\n",
                    s->prediction_limit,s->predicted_peak,s->constraint_violation) :
                    snprintf(row+n,sizeof(row)-n,"\n");
        if (extra<0 || n+extra>=sizeof(row)) break;
        n+=extra;
        if (used+n+64>=size) break;
        memcpy(buffer+used,row,n); used+=n; ++trace_read;
    }
    int n=snprintf(buffer+used,size-used,"OBSEND %u %u %u\n",experiment_id,trace_dropped,trace_write-trace_read);
    xSemaphoreGive(motor_lock);
    return n<0 ? n : (int)used+n;
}

int motor_sample_batch(char *buffer,size_t size){return sample_batch(buffer,size,false);}
int motor_sample_batch2(char *buffer,size_t size){return sample_batch(buffer,size,true);}
