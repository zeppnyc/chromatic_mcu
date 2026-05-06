#include "pwrmgr.h"

#include "board.h"
#include "button.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "fpga_common.h"
#include "fpga_rx.h"
#include "fpga_tx.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "osd.h"

#include <stdbool.h>
#include <stdint.h>

enum {
    kIdleTime_ms = 100,
};

typedef enum LowPowerMode {
    kLowPowerMode_Inactive,
    kLowPowerMode_Active,
} LowPowerMode_t;

static const char *TAG = "PwrMgr";
static TaskHandle_t PwrMgrTaskHandle = NULL;
static TimerHandle_t IdleSystemTimer = NULL;
static StaticTimer_t IdleTimerBuffer;
static volatile bool IdleTimerSuspended = false;
static volatile uint32_t IdleActivityGeneration = 0;
static volatile uint32_t IdleSleepRequestGeneration = 0;
static LowPowerMode_t LPM_Status;
static SemaphoreHandle_t xSemaphore;
static StaticSemaphore_t xMutexBuffer;

// The UART receive line is selected so the MCU wakes up any time the FPGA sends a message
static const gpio_config_t WakeUpPin = {
    .pin_bit_mask = BIT64(PIN_NUM_UART_FROM_FPGA),
    .mode = GPIO_MODE_INPUT,
    .pull_down_en = false,
    .pull_up_en = false,
    .intr_type = GPIO_INTR_DISABLE
};

static void IdleTimerCB( TimerHandle_t xTimer );

static bool ShouldAbortSleep(uint32_t sleep_request_generation)
{
    return IdleTimerSuspended || IdleActivityGeneration != sleep_request_generation;
}

static bool AbortSleepIfCanceled(uint32_t sleep_request_generation)
{
    if (!ShouldAbortSleep(sleep_request_generation))
    {
        return false;
    }
    FPGA_Rx_Resume();
    FPGA_Tx_Resume();
    return true;
}

void PwrMgr_Task(void *arg)
{
    (void)arg;

    PwrMgrTaskHandle = xTaskGetCurrentTaskHandle();

    IdleSystemTimer = xTimerCreateStatic("tmr-idle-sys", pdMS_TO_TICKS(kIdleTime_ms), pdFAIL, NULL, IdleTimerCB, &IdleTimerBuffer);
    if (xTimerStart(IdleSystemTimer, portMAX_DELAY) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to start idle timer");
    }

    xSemaphore = xSemaphoreCreateMutexStatic( &xMutexBuffer );
    if (xSemaphore == NULL)
    {
        ESP_LOGE(TAG, "Failed to create semaphore");
    }

    while(1)
    {
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
        {
            const uint32_t sleep_request_generation = IdleSleepRequestGeneration;
            if (ShouldAbortSleep(sleep_request_generation))
            {
                continue;
            }

            // Pause both tasks to ensure that they finish up any work in progress. Since the Sleep task is of higher priority,
            // the delays below are taken advantage of to permit a context switch.
            FPGA_Tx_Pause();
            FPGA_Rx_Pause();
            if (AbortSleepIfCanceled(sleep_request_generation))
            {
                continue;
            }

            // Clear out any pending data that the FPGA may have sent
            uart_flush(UART_NUM_1);
            Button_ResetAll();
            OSD_SetVisiblityState(false);

            // Configure the UART pin for wake up
            gpio_config(&WakeUpPin);
            gpio_wakeup_enable(PIN_NUM_UART_FROM_FPGA, GPIO_INTR_LOW_LEVEL);
            vTaskDelay(pdMS_TO_TICKS(10));
            if (AbortSleepIfCanceled(sleep_request_generation))
            {
                continue;
            }

            esp_sleep_enable_gpio_wakeup();
            vTaskDelay(pdMS_TO_TICKS(10));
            if (AbortSleepIfCanceled(sleep_request_generation))
            {
                continue;
            }

            // Tell the world we are sleeping and wait for the message to go out...
            ESP_LOGD(TAG, "Sleeping");
            uart_wait_tx_done(CONFIG_ESP_CONSOLE_UART_NUM, portMAX_DELAY);

            esp_light_sleep_start();

            ESP_LOGD(TAG, "Awake");
            uart_wait_tx_done(CONFIG_ESP_CONSOLE_UART_NUM, portMAX_DELAY);

            // Update the brightness in the event hot-keys were used to adjust it.
            // The Rx task will resume the Tx task once it receives the brightness level
            FPGA_Rx_UseBrightnessReadback();
            FPGA_Rx_Resume();
        }
    }

    ESP_LOGE(TAG, "Sleep loop exited");
}

void PwrMgr_TriggerLightSleep(void)
{
    if (IdleTimerSuspended)
    {
        return;
    }

    // Wait for the sleep task to start
    while(PwrMgrTaskHandle == NULL)
    {
        vTaskDelay(1);
    }

    IdleSleepRequestGeneration = IdleActivityGeneration;
    (void) xTaskNotifyGive(PwrMgrTaskHandle);
}

void PwrMgr_IdleTimerPet(void)
{
    IdleActivityGeneration++;

    // Wait for the task to start
    if (IdleSystemTimer == NULL)
    {
        return;
    }

    (void) xTimerReset(IdleSystemTimer, portMAX_DELAY);
}

void PwrMgr_IdleTimerSuspend(void)
{
    IdleTimerSuspended = true;
    if (IdleSystemTimer != NULL)
    {
        (void) xTimerStop(IdleSystemTimer, portMAX_DELAY);
    }
    if (PwrMgrTaskHandle != NULL)
    {
        (void) xTaskNotifyGive(PwrMgrTaskHandle);
    }
}

void PwrMgr_IdleTimerResume(void)
{
    IdleTimerSuspended = false;
    PwrMgr_IdleTimerPet();
}

static void IdleTimerCB( TimerHandle_t xTimer )
{
    (void)xTimer;
    PwrMgr_TriggerLightSleep();
}

void PwrMgr_SetLPM(const bool Active)
{
    if ((xSemaphore != NULL) && xSemaphoreTake(xSemaphore, portMAX_DELAY))
    {
        LPM_Status = Active ? kLowPowerMode_Active : kLowPowerMode_Inactive;
        xSemaphoreGive(xSemaphore);
    }
}

bool PwrMgr_IsLPMActive(void)
{
    LowPowerMode_t Status = kLowPowerMode_Inactive;

    if ((xSemaphore != NULL) && xSemaphoreTake(xSemaphore, portMAX_DELAY))
    {
        Status = LPM_Status;
        xSemaphoreGive(xSemaphore);
    }

    return Status;
}
