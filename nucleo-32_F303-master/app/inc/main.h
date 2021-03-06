#ifndef APP_INC_MAIN_H_
#define APP_INC_MAIN_H_

#include "stm32f3xx.h"
#include "bsp.h"
#include "delay.h"
#include "math.h"
#include "stdio.h"
#include "i2c.h"
#include "imu.h"
//#include "spi.h"
//#include "mpu9250.h"
//#include "MadgwickAHRS.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "queue.h"
#include "event_groups.h"
#include "stream_buffer.h"
/* Global functions */

int my_printf	(const char *format, ...);
int my_sprintf	(char *out, const char *format, ...);



#endif /* APP_INC_MAIN_H_ */
