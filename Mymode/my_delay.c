#include "my_delay.h"


void SysRunTime_tick(void)
{
	Tick_Time++;
}
uint64_t Get_Tick_Time()
{
	return Tick_Time;
}
