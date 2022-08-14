#ifndef _DEFINES_H_
#define  _DEFINES_H_
#define PI (3.1415)
#define SAMPLE_TIME 10
#define bodyDiameter 1.1261289
#define interWheelDist 1.1288289
#define wheelDiameter 0.128
#include"stm32f4xx_hal.h"
#include"stm32f4xx_it.h"
#include"stm32f4xx_hal_conf.h"
#include <math.h>
#include <stdio.h>
#include"usart.h"
inline float normalizeAngle(float dx)
{
	if (dx >= 0)
		dx = fmodf(dx + (float)PI, 2 * (float)PI) - (float)PI;
	else
		dx = fmodf(dx - (float)PI, 2 * (float)PI) + (float)PI;
	return dx;
}
#endif //! _DEFINES_H_