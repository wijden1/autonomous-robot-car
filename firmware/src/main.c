#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

#include "hal.h"

static void plant_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        hal_sim_step(0.001f);                       /* advance physics by 1 ms */
        vTaskDelayUntil(&last, pdMS_TO_TICKS(1));
    }
}

static void test_task(void *arg)
{
    (void)arg;
    printf("Applying 6 V to the motor (open loop, no controller)\n");
    hal_motor_set_voltage(6.0f);

    int32_t prev_ticks = hal_encoder_read_ticks();
    for (int i = 1; i <= 8; i++) {
        vTaskDelay(pdMS_TO_TICKS(100));
        int32_t ticks = hal_encoder_read_ticks();
        float rpm = (float)(ticks - prev_ticks) / HAL_TICKS_PER_REV / 0.1f * 60.0f;
        prev_ticks = ticks;
        printf("t=%.1f s  encoder=%6ld ticks  speed=%6.1f rpm\n", i * 0.1f, (long)ticks, rpm);
    }

    hal_motor_set_voltage(0.0f);
    printf("Done\n");
    exit(0);
}

int main(void)
{
    setvbuf(stdout, NULL, _IOLBF, 0);
    hal_init();

    BaseType_t ok = pdPASS;
    ok &= xTaskCreate(plant_task, "plant", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 4, NULL);
    ok &= xTaskCreate(test_task,  "test",  configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 2, NULL);
    configASSERT(ok == pdPASS);

    vTaskStartScheduler();
    return 1;
}