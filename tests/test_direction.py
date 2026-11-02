"""Offline direction/stop tests; never connects to or drives the board.

Run: ./.venv/bin/python -m unittest discover -s tests -v
Native harnesses compile the actual C sources with mocked ESP-IDF hardware.
"""
import contextlib
import io
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'host'))
import pi_control


class FakePort:
    def __init__(self, lines=()):
        self.lines = iter(lines)
        self.writes = []

    def write(self, value):
        self.writes.append(value)

    def readline(self):
        return next(self.lines)


class ProtocolTests(unittest.TestCase):
    def test_both_directions_and_polarity(self):
        for direction, token, sign, target in [('forward', 'FWD', 1, 80),
                                                ('reverse', 'REV', -1, -80)]:
            with self.subTest(direction=direction):
                port = FakePort([
                    b'ACK MOTOR\n',
                    b'PI3 0 120 0 0 0 0 IDLE 15 1\n',
                    f'PI3 1 {target} 0 0 0 0 WAITING 5 {sign}\n'.encode(),
                    f'PI3 1 {target} {target} {target} 34 1 RUNNING 5 {sign}\n'.encode(),
                    f'PI3 0 {target} {target} {target} 0 5 DONE 5 {sign}\n'.encode(),
                ])
                with patch.object(pi_control.time, 'sleep'), contextlib.redirect_stdout(io.StringIO()):
                    pi_control.run_test(port, 80, 5, direction, sign)
                self.assertIn(f'MOTOR PI 80 5 {token} {sign}\n'.encode(), port.writes)
                self.assertEqual(port.writes[-1], b'MOTOR STOP\n')

    def test_stop_on_disconnect_or_interrupt(self):
        for error in [RuntimeError('disconnected'), KeyboardInterrupt()]:
            port = FakePort()
            with patch.object(pi_control, 'read_status', side_effect=[{'active': 0}, error]), \
                 patch.object(pi_control.time, 'sleep'), contextlib.redirect_stdout(io.StringIO()):
                with self.assertRaises(type(error)):
                    pi_control.run_test(port)
            self.assertEqual(port.writes[-1], b'MOTOR STOP\n')

    def test_wrong_direction_stops(self):
        port = FakePort([b'PI3 0 120 0 0 0 0 IDLE 15 1\n',
                         b'PI3 1 120 0 0 0 0 WAITING 15 1\n'])
        with patch.object(pi_control.time, 'sleep'), contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(RuntimeError):
                pi_control.run_test(port, direction='reverse')
        self.assertEqual(port.writes[-1], b'MOTOR STOP\n')

    def test_old_firmware_never_starts(self):
        port = FakePort([b'PI2 0 120 0 0 0 0 IDLE 15\n', b''])
        with patch.object(pi_control.time, 'monotonic', side_effect=[0, 0, 0, 2]):
            with self.assertRaises(RuntimeError):
                pi_control.run_test(port)
        self.assertEqual(port.writes, [b'PI?\n'])


STUBS = r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <math.h>
#include <ctype.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_ERR_INVALID_STATE 1
#define ESP_ERROR_CHECK(x) assert((x)==0)
#define ESP_LOGI(...) ((void)0)
#define GPIO_NUM_4 4
#define GPIO_NUM_5 5
#define GPIO_NUM_6 6
#define GPIO_NUM_7 7
#define GPIO_NUM_10 10
#define GPIO_NUM_11 11
#define GPIO_MODE_OUTPUT 1
#define GPIO_MODE_INPUT 2
#define GPIO_PULLDOWN_ENABLE 1
#define GPIO_PULLUP_ENABLE 1
#define GPIO_INTR_ANYEDGE 1
#define LEDC_LOW_SPEED_MODE 0
#define LEDC_CHANNEL_0 0
#define LEDC_TIMER_0 0
#define LEDC_TIMER_10_BIT 10
#define LEDC_AUTO_CLK 0
#define LEDC_INTR_DISABLE 0
typedef struct {uint64_t pin_bit_mask; int mode,pull_down_en,pull_up_en,intr_type;} gpio_config_t;
typedef struct {int speed_mode,duty_resolution,timer_num,freq_hz,clk_cfg;} ledc_timer_config_t;
typedef struct {int gpio_num,speed_mode,channel,intr_type,timer_sel,duty,hpoint;} ledc_channel_config_t;
static int levels[64], duty;
static int64_t now;
static int gpio_config(const gpio_config_t *p){return 0;}
static int gpio_set_level(int p,int v){levels[p]=v;return 0;}
static int gpio_get_level(int p){return levels[p];}
static int gpio_install_isr_service(int x){return 0;}
static int gpio_isr_handler_add(int p,void(*f)(void*),void*a){return 0;}
static int ledc_timer_config(const ledc_timer_config_t*p){return 0;}
static int ledc_channel_config(const ledc_channel_config_t*p){return 0;}
static int ledc_set_duty(int m,int c,int d){assert(d>=0 && d<=1023);duty=d;return 0;}
static int ledc_update_duty(int m,int c){return 0;}
static int64_t esp_timer_get_time(void){return now;}
typedef int TickType_t;
typedef int BaseType_t;
typedef int SemaphoreHandle_t;
typedef int portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL_ISR(x) ((void)0)
#define portEXIT_CRITICAL_ISR(x) ((void)0)
#define portENTER_CRITICAL(x) ((void)0)
#define portEXIT_CRITICAL(x) ((void)0)
#define portMAX_DELAY 0
#define pdPASS 1
#define pdMS_TO_TICKS(x) (x)
#define configASSERT(x) assert(x)
static int xSemaphoreCreateMutex(void){return 1;}
static void xSemaphoreTake(int a,int b){}
static void xSemaphoreGive(int a){}
static int xTaskCreate(void(*f)(void*),const char*n,int s,void*a,int p,void*b){return 1;}
static int xTaskGetTickCount(void){return 0;}
static void vTaskDelayUntil(int*a,int b){}
'''


class FirmwareTests(unittest.TestCase):
    def compile_run(self, source, harness):
        cc = shutil.which('cc')
        self.assertIsNotNone(cc, 'Native compiler required')
        # Remove platform includes only; compile the full production implementation.
        code = '\n'.join(line for line in (ROOT / source).read_text().splitlines()
                         if not line.startswith('#include'))
        header = (ROOT / 'main/encoder.h').read_text().replace('#pragma once', '').replace('#include <stdint.h>', '')
        header += (ROOT / 'main/motor.h').read_text().replace('#pragma once', '').replace('#include <stdbool.h>', '').replace('#include <stddef.h>', '')
        if source.endswith('motor.c'):
            header += '\n'.join(line for line in (ROOT/'main/predictive.h').read_text().splitlines()
                                if not line.startswith(('#include','#pragma')))
            code = '\n'.join(line for line in (ROOT/'main/predictive.c').read_text().splitlines()
                             if not line.startswith('#include'))+'\n'+code
        fake_snapshot = '' if source.endswith('encoder.c') else '''
static encoder_snapshot_t enc;
encoder_snapshot_t encoder_snapshot(void){return enc;}
'''
        with tempfile.TemporaryDirectory() as tmp:
            src = Path(tmp) / 'test.c'
            exe = Path(tmp) / 'test'
            src.write_text(STUBS + header + fake_snapshot + code + harness)
            result = subprocess.run([cc, '-std=c11', str(src), '-o', str(exe), '-lm'], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)
            return result.stdout

    def test_quadrature_phase_and_per_channel_scale(self):
        self.compile_run('main/encoder.c', r'''
int main(void){
    // 00 -> 01 -> 11 -> 10 -> 00: two positive edges on each channel.
    for(int i=0;i<700;i++){
        levels[10]=1;c2_isr(NULL);levels[11]=1;c1_isr(NULL);
        levels[10]=0;c2_isr(NULL);levels[11]=0;c1_isr(NULL);
    }
    encoder_snapshot_t x=encoder_snapshot();
    assert(x.c1_edges==1400 && x.c2_edges==1400);
    assert(x.c1_signed==1400 && x.c2_signed==1400);
    for(int i=0;i<700;i++){
        levels[11]=1;c1_isr(NULL);levels[10]=1;c2_isr(NULL);
        levels[11]=0;c1_isr(NULL);levels[10]=0;c2_isr(NULL);
    }
    x=encoder_snapshot();
    assert(x.c1_edges==2800 && x.c2_edges==2800);
    assert(x.c1_signed==0 && x.c2_signed==0);
}
''')

    def test_reversal_wait_deadlines_feedback_and_parsing(self):
        self.compile_run('main/motor.c', r'''
int main(void){
    motor_init();
    const char *invalid[]={"MOTOR PI 1205 5 REV 1", "MOTOR PI 120 5 BAD 1",
        "MOTOR PI 120 5 REV 0", "MOTOR PI 120 61 REV 1", "MOTOR PI -120 5 REV 1",
        "MOTOR PI 120 5 REV 1 extra", "MOTOR PI 120 5 REV"};
    for(int i=0;i<7;i++){assert(!motor_command(invalid[i]));assert(!pi_active);}
    assert(motor_command("MOTOR PI 120 1 FWD 1"));
    assert(pi_waiting && duty==0);
    now=500000;motor_poll();assert(!pi_waiting && duty>0 && levels[5]==1 && levels[6]==0);
    for(int i=0;i<19;i++){
        now+=50000;enc.c1_edges+=140;enc.c2_edges+=140;
        enc.c1_signed+=140;enc.c2_signed+=140;motor_poll();
        assert(pi_active && pi_pwm<=80 && pi_pwm>=0);
    }
    assert(pi_signed_rpm[0]>110);
    now+=50000;motor_poll();assert(!pi_active && duty==0 && strcmp(pi_reason,"DONE")==0);
    assert(motor_command("MOTOR PI 120 5 REV -1"));
    // Simulate coast pulses: never apply reverse until both channels quiet for 500 ms.
    now+=300000;enc.c1_edges++;motor_poll();assert(pi_waiting && duty==0);
    now+=490000;motor_poll();assert(pi_waiting && duty==0);
    now+=10000;motor_poll();assert(!pi_waiting && levels[5]==0 && levels[6]==1);
    for(int i=0;i<10;i++){
        now+=50000;enc.c1_edges+=140;enc.c2_edges+=140;
        enc.c1_signed+=140;enc.c2_signed+=140;motor_poll();
    }
    assert(pi_signed_rpm[0]<-110 && pi_signed_rpm[1]<-110);
    char reply[192];motor_pi_status(reply,sizeof(reply));assert(strstr(reply,"PI3 1 -120.0"));
    now+=600000;motor_poll();assert(!pi_active && duty==0 && strcmp(pi_reason,"NO_FEEDBACK")==0);
    assert(motor_command("MOTOR PI 80 5 FWD 1"));
    for(int i=0;i<30;i++){now+=100000;enc.c1_edges++;motor_poll();assert(duty==0);}
    assert(!pi_active && strcmp(pi_reason,"NOT_STOPPED")==0);
    assert(motor_command("MOTOR PI 80 5 REV 1"));
    assert(motor_command("MOTOR STOP"));assert(!pi_active && strcmp(pi_reason,"STOPPED")==0);
    now+=500000;motor_poll();assert(duty==0);
}
''')


if __name__ == '__main__':
    unittest.main()
