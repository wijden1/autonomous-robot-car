/*
 * Motor-control firmware (FreeRTOS)
 *
 * Two modes:
 *   ./motor_firmware                         built-in 7 s test scenario
 *   ./motor_firmware --can vcan0 --side left  motor ECU on the CAN bus
 *
 * Tasks (highest priority first):
 *   plant    1 kHz   simulated motor physics (would be the real motor on hardware)
 *   control  100 Hz  encoder speed measurement, PID, watchdog safety stop,
 *                    sends MOTOR_STATUS every 20 ms in CAN mode
 *   command  10 Hz   test scenario (demo mode)  OR
 *   can_rx   200 Hz  receives WHEEL_CMD from the CAN bus (CAN mode)
 *   logger   -       writes control data to a CSV file and prints a status line
 *
 * Tasks communicate only through queues, never through shared global variables.
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "hal.h"
#include "pid.h"
#include "hal_can.h"

/* ---- Timing ------------------------------------------------------------ */
#define PLANT_PERIOD_MS       1
#define CONTROL_PERIOD_MS     10
#define COMMAND_PERIOD_MS     100
#define WATCHDOG_TIMEOUT_MS   500
#define CAN_RX_PERIOD_MS      5
#define CAN_STATUS_EVERY_N    2      /* status every 2nd control cycle = 20 ms */

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

/* ---- Configuration from the command line (set once in main) ----------- */
static int can_mode = 0;
static int side_left = 1;
static const char *side_name = "motor";
static const char *csv_name = "motor_log.csv";

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
    uint32_t cycle = 0;
    uint8_t status_counter = 0;

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
                printf("[%s] WATCHDOG: no command for %d ms -> motor off\n", side_name, WATCHDOG_TIMEOUT_MS);
                watchdog_active = 1;
            }
            pid_reset(&pid);
            voltage = 0.0f;
        } else {
            if (watchdog_active) {
                printf("[%s] Commands received again -> control active\n", side_name);
                watchdog_active = 0;
            }
            voltage = pid_update(&pid, setpoint, measured_rpm, dt);
        }
        hal_motor_set_voltage(voltage);

        /* 4. CAN mode: report speed, voltage and state every 20 ms */
        if (can_mode && (++cycle % CAN_STATUS_EVERY_N) == 0) {
            motor_status_t st = {
                .speed_rpm = (int16_t)lroundf(measured_rpm),
                .voltage = voltage,
                .state = timed_out ? MOTOR_STATE_WATCHDOG : MOTOR_STATE_OK,
                .alive_counter = status_counter++,
            };
            can_frame_t frame;
            can_pack_motor_status(side_left ? CAN_ID_MOTOR_STATUS_LEFT : CAN_ID_MOTOR_STATUS_RIGHT,
                                  &st, &frame);
            hal_can_send(&frame);
        }

        /* 5. Send a log record (dropped if the logger is behind) */
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

/* ------------------------------------------------------------------------
 * CAN mode: receive WHEEL_CMD frames and forward our own setpoint.
 * The alive counter must change, otherwise the sender is frozen and the
 * frame does not count as a heartbeat (the watchdog keeps running).
 * ------------------------------------------------------------------------ */
static void can_rx_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    int have_counter = 0;
    uint8_t prev_counter = 0;

    for (;;) {
        can_frame_t frame;
        while (hal_can_receive(&frame)) {
            wheel_cmd_t cmd;
            if (!can_unpack_wheel_cmd(&frame, &cmd)) continue;   /* not for us */

            if (have_counter && cmd.alive_counter == prev_counter) continue;  /* stale */
            have_counter = 1;
            prev_counter = cmd.alive_counter;

            speed_cmd_t sc = {
                .setpoint_rpm = side_left ? cmd.setpoint_left_rpm : cmd.setpoint_right_rpm
            };
            xQueueSend(cmd_queue, &sc, 0);
        }
        vTaskDelayUntil(&last, pdMS_TO_TICKS(CAN_RX_PERIOD_MS));
    }
}

/* ------------------------------------------------------------------------ */
static void logger_task(void *arg)
{
    (void)arg;
    FILE *csv = fopen(csv_name, "w");
    if (csv) fprintf(csv, "time_ms,setpoint_rpm,measured_rpm,voltage,load_nm,watchdog_stop\n");

    uint32_t next_print_ms = 0;
    for (;;) {
        log_event_t evt;
        xQueueReceive(log_queue, &evt, portMAX_DELAY);

        if (evt.type == EVT_DONE) {
            if (csv) fclose(csv);
            printf("[logger] Test finished. Data written to %s\n", csv_name);
            fflush(stdout);
            exit(0);
        }

        log_record_t *r = &evt.record;
        if (csv) {
            fprintf(csv, "%u,%.1f,%.1f,%.3f,%.3f,%u\n", r->time_ms, r->setpoint_rpm,
                    r->measured_rpm, r->voltage, r->load_nm, r->watchdog_stop);
        }
        if (r->time_ms >= next_print_ms) {
            printf("[%s] t=%5.1f s  set=%6.1f rpm  meas=%6.1f rpm  U=%5.2f V\n", side_name,
                   r->time_ms / 1000.0f, r->setpoint_rpm, r->measured_rpm, r->voltage);
            fflush(stdout);
            next_print_ms += can_mode ? 2000 : 500;
        }
    }
}

/* ------------------------------------------------------------------------ */
static void usage(void)
{
    printf("Usage: motor_firmware                              (built-in test scenario)\n"
           "       motor_firmware --can <iface> --side left|right (motor ECU on CAN)\n");
}

int main(int argc, char **argv)
{
    setvbuf(stdout, NULL, _IOLBF, 0);   /* print each line immediately */

    const char *can_iface = NULL;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--can") == 0 && i + 1 < argc) {
            can_iface = argv[++i];
        } else if (strcmp(argv[i], "--side") == 0 && i + 1 < argc) {
            side_left = strcmp(argv[++i], "right") != 0;
        } else {
            usage();
            return 1;
        }
    }

    if (can_iface) {
        if (hal_can_init(can_iface) != 0) return 1;
        can_mode = 1;
        side_name = side_left ? "left" : "right";
        csv_name = side_left ? "motor_log_left.csv" : "motor_log_right.csv";
    }

    hal_init();

    cmd_queue = xQueueCreate(8, sizeof(speed_cmd_t));
    log_queue = xQueueCreate(256, sizeof(log_event_t));
    configASSERT(cmd_queue && log_queue);

    BaseType_t ok = pdPASS;
    ok &= xTaskCreate(plant_task,   "plant",   STACK_SIZE, NULL, PRIO_PLANT,   NULL);
    ok &= xTaskCreate(control_task, "control", STACK_SIZE, NULL, PRIO_CONTROL, NULL);
    if (can_mode) {
        ok &= xTaskCreate(can_rx_task, "can_rx", STACK_SIZE, NULL, PRIO_COMMAND, NULL);
    } else {
        ok &= xTaskCreate(command_task, "command", STACK_SIZE, NULL, PRIO_COMMAND, NULL);
    }
    ok &= xTaskCreate(logger_task,  "logger",  STACK_SIZE, NULL, PRIO_LOGGER,  NULL);
    configASSERT(ok == pdPASS);   /* fails if the heap is too small */

    if (can_mode) {
        printf("Motor ECU '%s' starting on %s (status ID 0x%03X)\n", side_name, can_iface,
               side_left ? CAN_ID_MOTOR_STATUS_LEFT : CAN_ID_MOTOR_STATUS_RIGHT);
    } else {
        printf("Motor firmware starting (FreeRTOS POSIX port)\n");
    }
    vTaskStartScheduler();

    return 1;  /* only reached if the scheduler could not start */
}