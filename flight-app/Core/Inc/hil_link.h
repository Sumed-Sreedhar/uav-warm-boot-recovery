#ifndef HIL_LINK_H
#define HIL_LINK_H

#include "hil_sensor.h"

/* Define HIL_LINK_LOOPBACK=1 for the USART1 DMA echo bench build. */
#ifndef HIL_LINK_LOOPBACK
#define HIL_LINK_LOOPBACK 0
#endif

void HilLink_Task(void *argument);

#if !HIL_LINK_LOOPBACK
/* Copies the latest complete sample. Call from task context.
 * Returns false before the first sample or after a transport failure.
 * The caller is responsible for checking freshness.
 */
bool HilLink_GetLatestSensor(HilSensorSample *sample);
#endif

#endif
