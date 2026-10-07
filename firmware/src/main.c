/*
 * Motor-control firmware (FreeRTOS)
 *
 * Tasks (highest priority first):
 *   plant    1 kHz   simulated motor physics (would be the real motor on hardware)
 *   control  100 Hz  encoder speed measurement, PID, watchdog safety stop
 *   command  10 Hz   test scenario that sends speed commands (later: CAN from ROS 2)
 *   logger   -       writes control data to motor_log.csv and prints a status line
 *
 * Tasks communicate only through queues, never through shared global variables.
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "hal.h"
#include "pid.h"

/* ---- Timing ------------------------------------------------------------ */
#define PLANT_PERIOD_MS       1
#define CONTROL_PERIOD_MS     10
#define COMMAND_PERIOD_MS     100
#define WATCHDOG_TIMEOUT_MS   500

/* ---- Controller tuning (output: volts, input: RPM) --------------------- */
#define PID_KP   0.010f
#define PID_KI   0.100f
#define PID_KD   0.0f

/* ---- Priorities -------------------------------------------------------- */
#define PRIO_PLANT    (tskIDLE_PRIORITY + 4)
#define PRIO_CONTROL  (tskIDLE_PRIORITY + 3)
#define PRIO_COMMAND  (tskIDLE_PRIORITY + 2)
#define PRIO_LOGGER   (tskIDLE_PRIORITY + 1)

#define STACK_SIZE    (configMINIMAL_STACK_SIZE * 2)

/* ---- Messages ---------------------------------------------------------- */
typedef struct {
    float setpoint_rpm;
} speed_cmd_t;

typedef struct {
    uint32_t time_ms;
    float setpoint_rpm;
    float measured_rpm;
    float voltage;
    float load_nm;
    uint8_t watchdog_stop;
} log_record_t;

typedef enum { EVT_LOG, EVT_DONE } log_event_type_t;

typedef struct {
    log_event_type_t type;
    log_record_t record;
} log_event_t;

static QueueHandle_t cmd_queue;
static QueueHandle_t log_queue;
static volatile float current_load_nm = 0.0f;   /* for logging only */

static uint32_t now_ms(void)
{
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

/* ------------------------------------------------------------------------ */
static void plant_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        hal_sim_step(PLANT_PERIOD_MS / 1000.0f);
        vTaskDelayUntil(&last, pdMS_TO_TICKS(PLANT_PERIOD_MS));
    }
}

/* ------------------------------------------------------------------------ */
static void control_task(void *arg)
{
    (void)arg;
    pid_t_ctrl pid;
    pid_init(&pid, PID_KP, PID_KI, PID_KD, -HAL_SUPPLY_VOLTAGE, HAL_SUPPLY_VOLTAGE);

    const float dt = CONTROL_PERIOD_MS / 1000.0f;
    float setpoint = 0.0f;
    uint32_t last_cmd_ms = now_ms();
    int watchdog_active = 0;
    int32_t prev_ticks = hal_encoder_read_ticks();
    TickType_t prev_time = xTaskGetTickCount();
    TickType_t last = prev_time;

    for (;;) {
        /* 1. Take the newest command, if any (never block the control loop) */
        speed_cmd_t cmd;
        while (xQueueReceive(cmd_queue, &cmd, 0) == pdPASS) {
            setpoint = cmd.setpoint_rpm;
            last_cmd_ms = now_ms();
        }

        /* 2. Measure speed: encoder tick difference divided by the REAL
         *    elapsed time (the loop period can jitter by a tick or two) */
        int32_t ticks = hal_encoder_read_ticks();
        TickType_t time_now = xTaskGetTickCount();
        float elapsed_s = (float)((time_now - prev_time) * portTICK_PERIOD_MS) / 1000.0f;
        if (elapsed_s <= 0.0f) elapsed_s = dt;
        float measured_rpm = (float)(ticks - prev_ticks) / HAL_TICKS_PER_REV / elapsed_s * 60.0f;
        prev_ticks = ticks;
        prev_time = time_now;

        /* 3. Watchdog: no command for too long -> safe state */
        int timed_out = (now_ms() - last_cmd_ms) > WATCHDOG_TIMEOUT_MS;
        float voltage;
        if (timed_out) {
            if (!watchdog_active) {
                printf("[control] WATCHDOG: no command for %d ms -> motor off\n", WATCHDOG_TIMEOUT_MS);
                watchdog_active = 1;
            }
            pid_reset(&pid);
            voltage = 0.0f;
        } else {
            if (watchdog_active) {
                printf("[control] Commands received again -> control active\n");
                watchdog_active = 0;
            }
            voltage = pid_update(&pid, setpoint, measured_rpm, dt);
        }
        hal_motor_set_voltage(voltage);

        /* 4. Send a log record (dropped if the logger is behind) */
        log_event_t evt = {
            .type = EVT_LOG,
            .record = { now_ms(), timed_out ? 0.0f : setpoint, measured_rpm,
                        voltage, current_load_nm, (uint8_t)timed_out }
        };
        xQueueSend(log_queue, &evt, 0);

        vTaskDelayUntil(&last, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

/* ------------------------------------------------------------------------
 * Test scenario. Later this task is replaced by commands arriving over CAN.
 *   0.5 s  setpoint 1000 RPM
 *   2.0 s  load torque step (someone brakes the wheel)
 *   3.5 s  setpoint 500 RPM
 *   5.0 s  commands stop -> watchdog switches the motor off ~0.5 s later
 *   7.0 s  end of test
 * ------------------------------------------------------------------------ */
static void command_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    int load_applied = 0;

    for (;;) {
        uint32_t t = now_ms();

        if (t >= 2000 && !load_applied) {
            printf("[command] t=%.1f s: load torque step 0.01 Nm\n", t / 1000.0f);
            hal_set_load_torque(0.01f);
            current_load_nm = 0.01f;
            load_applied = 1;
        }

        if (t < 5000) {
            speed_cmd_t cmd = { .setpoint_rpm = 0.0f };
            if (t >= 500)  cmd.setpoint_rpm = 1000.0f;
            if (t >= 3500) cmd.setpoint_rpm = 500.0f;
            xQueueSend(cmd_queue, &cmd, 0);   /* also acts as a heartbeat */
        }

        if (t >= 7000) {
            log_event_t done = { .type = EVT_DONE };
            xQueueSend(log_queue, &done, portMAX_DELAY);
            vTaskDelete(NULL);
        }

        vTaskDelayUntil(&last, pdMS_TO_TICKS(COMMAND_PERIOD_MS));
    }
}

/* ------------------------------------------------------------------------ */
static void logger_task(void *arg)
{
    (void)arg;
    FILE *csv = fopen("motor_log.csv", "w");
    if (csv) fprintf(csv, "time_ms,setpoint_rpm,measured_rpm,voltage,load_nm,watchdog_stop\n");

    uint32_t next_print_ms = 0;
    for (;;) {
        log_event_t evt;
        xQueueReceive(log_queue, &evt, portMAX_DELAY);

        if (evt.type == EVT_DONE) {
            if (csv) fclose(csv);
            printf("[logger] Test finished. Data written to motor_log.csv\n");
            fflush(stdout);
            exit(0);
        }

        log_record_t *r = &evt.record;
        if (csv) {
            fprintf(csv, "%u,%.1f,%.1f,%.3f,%.3f,%u\n", r->time_ms, r->setpoint_rpm,
                    r->measured_rpm, r->voltage, r->load_nm, r->watchdog_stop);
        }
        if (r->time_ms >= next_print_ms) {
            printf("[status] t=%4.1f s  set=%6.1f rpm  meas=%6.1f rpm  U=%5.2f V\n",
                   r->time_ms / 1000.0f, r->setpoint_rpm, r->measured_rpm, r->voltage);
            fflush(stdout);
            next_print_ms += 500;
        }
    }
}

/* ------------------------------------------------------------------------ */
int main(void)
{
    setvbuf(stdout, NULL, _IOLBF, 0);   /* print each line immediately */
    hal_init();

    cmd_queue = xQueueCreate(8, sizeof(speed_cmd_t));
    log_queue = xQueueCreate(256, sizeof(log_event_t));
    configASSERT(cmd_queue && log_queue);

    BaseType_t ok = pdPASS;
    ok &= xTaskCreate(plant_task,   "plant",   STACK_SIZE, NULL, PRIO_PLANT,   NULL);
    ok &= xTaskCreate(control_task, "control", STACK_SIZE, NULL, PRIO_CONTROL, NULL);
    ok &= xTaskCreate(command_task, "command", STACK_SIZE, NULL, PRIO_COMMAND, NULL);
    ok &= xTaskCreate(logger_task,  "logger",  STACK_SIZE, NULL, PRIO_LOGGER,  NULL);
    configASSERT(ok == pdPASS);   /* fails if the heap is too small */

    printf("Motor firmware starting (FreeRTOS POSIX port)\n");
    vTaskStartScheduler();

    return 1;  /* only reached if the scheduler could not start */
}