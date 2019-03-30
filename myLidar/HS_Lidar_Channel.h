#ifndef HS_Lidar_Channel_H
#define HS_Lidar_Channel_H

#include <stdint.h>

class HS_Lidar_Channel
{
public:
	uint32_t nHeader;	//帧头
	uint16_t nChannelNo;//通道号
	uint16_t nS0;		//第一段提取起点位置S0
	uint16_t nL0;		//第一段提取长度L0
	uint16_t nD0[320];	//数据D0
	uint16_t nS1;		//第二段提取起点位置S1
	uint16_t nL1;		//第二段提取长度L1
	uint16_t nD1;		//数据D1
	uint32_t nTest;		//验证数据
	HS_Lidar_Channel() {};
	~HS_Lidar_Channel() {};
};


#endif