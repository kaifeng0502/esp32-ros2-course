#include "motor_display.h"
#include <stdio.h>
#include <string.h>

// Five-column, seven-row glyphs; bit zero is the top pixel.
static const struct { char c; uint8_t columns[5]; } font[] = {
    {'0',{0x3e,0x51,0x49,0x45,0x3e}}, {'1',{0x00,0x42,0x7f,0x40,0x00}},
    {'2',{0x42,0x61,0x51,0x49,0x46}}, {'3',{0x21,0x41,0x45,0x4b,0x31}},
    {'4',{0x18,0x14,0x12,0x7f,0x10}}, {'5',{0x27,0x45,0x45,0x45,0x39}},
    {'6',{0x3c,0x4a,0x49,0x49,0x30}}, {'7',{0x01,0x71,0x09,0x05,0x03}},
    {'8',{0x36,0x49,0x49,0x49,0x36}}, {'9',{0x06,0x49,0x49,0x29,0x1e}},
    {'A',{0x7e,0x11,0x11,0x11,0x7e}}, {'B',{0x7f,0x49,0x49,0x49,0x36}},
    {'C',{0x3e,0x41,0x41,0x41,0x22}}, {'D',{0x7f,0x41,0x41,0x22,0x1c}},
    {'E',{0x7f,0x49,0x49,0x49,0x41}}, {'F',{0x7f,0x09,0x09,0x09,0x01}},
    {'G',{0x3e,0x41,0x49,0x49,0x7a}}, {'H',{0x7f,0x08,0x08,0x08,0x7f}},
    {'I',{0x00,0x41,0x7f,0x41,0x00}}, {'J',{0x20,0x40,0x41,0x3f,0x01}},
    {'K',{0x7f,0x08,0x14,0x22,0x41}}, {'L',{0x7f,0x40,0x40,0x40,0x40}},
    {'M',{0x7f,0x02,0x0c,0x02,0x7f}}, {'N',{0x7f,0x04,0x08,0x10,0x7f}},
    {'O',{0x3e,0x41,0x41,0x41,0x3e}}, {'P',{0x7f,0x09,0x09,0x09,0x06}},
    {'Q',{0x3e,0x41,0x51,0x21,0x5e}}, {'R',{0x7f,0x09,0x19,0x29,0x46}},
    {'S',{0x46,0x49,0x49,0x49,0x31}}, {'T',{0x01,0x01,0x7f,0x01,0x01}},
    {'U',{0x3f,0x40,0x40,0x40,0x3f}}, {'V',{0x1f,0x20,0x40,0x20,0x1f}},
    {'W',{0x3f,0x40,0x38,0x40,0x3f}}, {'X',{0x63,0x14,0x08,0x14,0x63}},
    {'Y',{0x07,0x08,0x70,0x08,0x07}}, {'Z',{0x61,0x51,0x49,0x45,0x43}},
    {'.',{0,0x60,0x60,0,0}}, {'+',{0x08,0x08,0x3e,0x08,0x08}},
    {'-',{0x08,0x08,0x08,0x08,0x08}}, {'/',{0x20,0x10,0x08,0x04,0x02}},
    {'%',{0x63,0x13,0x08,0x64,0x63}},
};

static void text(uint8_t *frame, int x, int y, const char *str, int scale)
{
    for (; *str; ++str, x += 6 * scale) {
        const uint8_t *columns = NULL;
        for (unsigned i = 0; i < sizeof(font) / sizeof(font[0]); ++i) {
            if (font[i].c == *str) { columns = font[i].columns; break; }
        }
        if (!columns) continue;
        for (int col = 0; col < 5; ++col) for (int row = 0; row < 7; ++row) {
            if (!(columns[col] & (1 << row))) continue;
            for (int dx = 0; dx < scale; ++dx) for (int dy = 0; dy < scale; ++dy) {
                int px = x + col * scale + dx, py = y + row * scale + dy;
                if (px >= 0 && px < 128 && py >= 0 && py < 64)
                    frame[(py / 8) * 128 + px] |= 1 << (py % 8);
            }
        }
    }
}

void motor_display_render(uint8_t frame[1024], const motor_status_t *status,
                          const encoder_snapshot_t *enc, int64_t now_us)
{
    // Keep measuring after the output stops, so coasting and hand turning are visible.
    static bool initialized;
    static int64_t sampled, previous[2];
    static float rpm;
    if (initialized && now_us > sampled) {
        float dt = (now_us - sampled) / 1000000.0f;
        float measured = status->encoder_sign *
            ((enc->c1_signed - previous[0]) + (enc->c2_signed - previous[1])) *
            30.0f / (ENCODER_COUNTS_PER_REV * dt);
        float alpha = dt / (0.15f + dt);
        rpm += alpha * (measured - rpm);
    }
    initialized = true;
    previous[0] = enc->c1_signed; previous[1] = enc->c2_signed;
    sampled = now_us;
    // Avoid a displayed negative zero from the decaying filter tail.
    float shown_rpm = (rpm > -0.05f && rpm < 0.05f) ? 0 : rpm;
    const char *reason = status->reason;
    if (strcmp(reason, "NO_FEEDBACK") == 0) reason = "NO ENCODER";
    if (strcmp(reason, "NOT_STOPPED") == 0) reason = "WAIT FAILED";
    if (strcmp(reason, "DIRECTION_ERROR") == 0) reason = "DIR ERROR";
    if (strcmp(reason, "BRAKE_TIMEOUT") == 0) reason = "BRAKE FAIL";
    memset(frame, 0, 1024);
    char line[32];
    snprintf(line, sizeof(line), "%s %s %s", status->target_rpm < 0 ? "REV" : "FWD",
             status->controller, reason);
    text(frame, 0, 0, line, 1);
    if (status->position_mode)
        snprintf(line, sizeof(line), "POS %+.2f/%+.2f", status->position_turns, status->goal_turns);
    else snprintf(line, sizeof(line), "SET %+4.0f RPM", status->target_rpm);
    text(frame, 0, 10, line, 1);
    snprintf(line, sizeof(line), "RPM %+6.1f", shown_rpm);
    text(frame, 0, 20, line, 1);
    snprintf(line, sizeof(line), "PWM %4.1f%%", status->pwm_percent);
    text(frame, 0, 33, line, 2);
    snprintf(line, sizeof(line), "TIME %4.1f / %d S", status->elapsed, status->seconds);
    text(frame, 0, 55, line, 1);
}
