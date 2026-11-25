#include "hil_link.h"

#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"
#include "usart.h"

#include <string.h>

#define RX_BUFFER_SIZE 512U
#define RX_PROGRESS    (1U << 0)
#define UART_ERROR     (1U << 2)

#if HIL_LINK_LOOPBACK
#define TX_COMPLETE    (1U << 1)
#endif

static uint8_t rx_buffer[RX_BUFFER_SIZE];
/* Keep a stable copy while TX DMA runs or the parser handles the bytes. */
static uint8_t rx_staging[RX_BUFFER_SIZE];

#if !HIL_LINK_LOOPBACK
static HilSensorSample latest_sensor;
static bool latest_sensor_valid;

bool HilLink_GetLatestSensor(HilSensorSample *sample)
{
  taskENTER_CRITICAL();
  bool valid = latest_sensor_valid;
  if (valid)
  {
    *sample = latest_sensor;
  }
  taskEXIT_CRITICAL();
  return valid;
}
#endif

static osThreadId_t volatile link_task;
static volatile uint32_t rx_produced;

/* The UART and DMA interrupts share a preemption priority.
 * Task-side sampling holds a critical section to serialize with them.
 */
static uint32_t sample_rx_progress(void)
{
  uint32_t produced = rx_produced;
  uint32_t position = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
  uint32_t previous = produced % RX_BUFFER_SIZE;

  /* Read the live DMA cursor so a delayed HT/TC event cannot move it backward.
   * At least one sample per ring lap is required. Keep both HT and TC enabled.
   */
  produced += (position + RX_BUFFER_SIZE - previous) % RX_BUFFER_SIZE;
  rx_produced = produced;
  return produced;
}

static uint32_t rx_progress_from_task(void)
{
  taskENTER_CRITICAL();
  uint32_t produced = sample_rx_progress();
  taskEXIT_CRITICAL();
  return produced;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *uart, uint16_t size)
{
  (void)size;

  if (uart == &huart1 && link_task != NULL)
  {
    sample_rx_progress();
    osThreadFlagsSet(link_task, RX_PROGRESS);
  }
}

#if HIL_LINK_LOOPBACK
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
  if (uart == &huart1 && link_task != NULL)
  {
    osThreadFlagsSet(link_task, TX_COMPLETE);
  }
}
#endif

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
  if (uart == &huart1 && link_task != NULL)
  {
    osThreadFlagsSet(link_task, UART_ERROR);
  }
}

static const char *run_link(void)
{
  uint32_t consumed = 0U;
  uint32_t wait_flags = RX_PROGRESS | UART_ERROR;
#if HIL_LINK_LOOPBACK
  wait_flags |= TX_COMPLETE;
  bool tx_busy = false;
#else
  HilSensorParser parser = {0};
#endif

  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, sizeof(rx_buffer)) != HAL_OK)
  {
    return "RX DMA start failed";
  }

  for (;;)
  {
    uint32_t flags = osThreadFlagsWait(wait_flags, osFlagsWaitAny, osWaitForever);
    if ((flags & osFlagsError) != 0U)
    {
      return "task notification failed";
    }
    if ((flags & UART_ERROR) != 0U)
    {
      return "UART or DMA error";
    }
#if HIL_LINK_LOOPBACK
    if ((flags & TX_COMPLETE) != 0U)
    {
      tx_busy = false;
    }
#endif

    uint32_t available = rx_progress_from_task() - consumed;
    if (available > RX_BUFFER_SIZE)
    {
      return "RX buffer overrun";
    }
#if HIL_LINK_LOOPBACK
    if (tx_busy)
    {
      continue;
    }
#endif
    if (available == 0U)
    {
      continue;
    }

    const volatile uint8_t *dma_data = rx_buffer;
    for (uint32_t index = 0U; index < available; ++index)
    {
      rx_staging[index] = dma_data[(consumed + index) % RX_BUFFER_SIZE];
    }
    if (rx_progress_from_task() - consumed > RX_BUFFER_SIZE)
    {
      return "RX buffer overrun during copy";
    }

#if HIL_LINK_LOOPBACK
    if (HAL_UART_Transmit_DMA(&huart1, rx_staging, (uint16_t)available) != HAL_OK)
    {
      return "TX DMA start failed";
    }
    tx_busy = true;
#else
    HilSensorSample sample;
    for (uint32_t index = 0U; index < available; ++index)
    {
      if (HilSensor_PushByte(&parser, rx_staging[index], &sample))
      {
        taskENTER_CRITICAL();
        latest_sensor = sample;
        latest_sensor_valid = true;
        taskEXIT_CRITICAL();
      }
    }
#endif
    consumed += available;
  }
}

void HilLink_Task(void *argument)
{
  (void)argument;

  link_task = osThreadGetId();
  const char *error = run_link();

  /* Stop after a transport failure and invalidate the published sample. */
  link_task = NULL;
#if !HIL_LINK_LOOPBACK
  taskENTER_CRITICAL();
  latest_sensor_valid = false;
  taskEXIT_CRITICAL();
#endif
  (void)HAL_UART_Abort(&huart1);

#if HIL_LINK_LOOPBACK
  static const uint8_t prefix[] = "USART1 loopback: ";
#else
  static const uint8_t prefix[] = "USART1 HIL: ";
#endif
  static const uint8_t newline[] = "\r\n";
  (void)HAL_UART_Transmit(&huart2, prefix, sizeof(prefix) - 1U, 100U);
  (void)HAL_UART_Transmit(&huart2, (const uint8_t *)error, (uint16_t)strlen(error), 100U);
  (void)HAL_UART_Transmit(&huart2, newline, sizeof(newline) - 1U, 100U);
  osThreadExit();
}
