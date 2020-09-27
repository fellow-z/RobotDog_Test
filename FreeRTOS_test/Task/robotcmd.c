#include "robotcmd.h"
#include "body_task.h"

void LedStateChange(void)
{
	if(remote.value.key_1 == 1)
	{
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
	}
	else
	{
		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
	}
}

void BodyParamChange(void)
{
	if(remote.state == 1)
	{
		if(remote.value.key_1 == 1 || remote.value.key_4 == 1)
		{
			body.workstate = Stop;
		}
		if(remote.value.key_2 == 1 || remote.value.key_5 == 1)
		{
			body.workstate = Attitude;
		}
		if(remote.value.key_3 == 1)
		{
			body.workstate = Walk;
		}
		if(remote.value.key_6 == 1)
		{
			body.workstate = Body_Stable;
		}
	}
	else
	{
		body.workstate = Stop;
	}
}
