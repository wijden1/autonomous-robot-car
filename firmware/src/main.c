#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

static void fast_task(void *arg)
{
    (void)arg;
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        printf("[fast] tick %u\n", (unsigned)xTaskGetTickCount());
        vTaskDelayUntil(&last, pdMS_TO_TICKS(250));   /* every 250 ms */
    }
}

static void slow_task(void *arg)
{
    (void)arg;
    int count = 0;
    for (;;) {
        printf("[slow] I run once per second (%d)\n", ++count);
        vTaskDelay(pdMS_TO_TICKS(1000));               /* sleep 1 s */
    }
}

int main(void)
{
    setvbuf(stdout, NULL, _IOLBF, 0);   /* print each line immediately */

    BaseType_t ok = pdPASS;
    ok &= xTaskCreate(fast_task, "fast", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 2, NULL);
    ok &= xTaskCreate(slow_task, "slow", configMINIMAL_STACK_SIZE * 2, NULL, tskIDLE_PRIORITY + 1, NULL);
    configASSERT(ok == pdPASS);

    printf("Starting FreeRTOS scheduler\n");
    vTaskStartScheduler();   /* never returns while tasks run */
    return 1;
}