#ifndef HS_Lidar_Header_H
#define HS_Lidar_Header_H

#include <stdint.h>
class HS_Lidar_Header
{
public:
	uint16_t nFill;				//帧头
	uint16_t nGPSWeek;			//GPS周
	double dGPSSecond;			//GPS秒
	uint32_t nGPSBreakdownTime;	//细分时间
	double dAzimuth;			//方位角(偏航角)
	double dPitch;				//俯仰角
	double dRoll;				//翻滚角(横滚角)
	double dX;					//GPS纬度
	double dY;					//GPS经度
	double dZ;					//GPS高度
	uint32_t nCodeDiscResolution;//码盘位数
	uint32_t nCodeNumber;		//码盘读数
	uint32_t nWaveNumber;		//波形通道数
	uint32_t nWaveLen;			//波形长度
	HS_Lidar_Header() {};
	~HS_Lidar_Header() {};
};


#endif