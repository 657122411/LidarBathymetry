#ifndef HS_Lidar_Channel_H
#define HS_Lidar_Channel_H

#include <stdint.h>

class HS_Lidar_Channel
{
public:
	uint32_t nHeader;	//֡ͷ
	uint16_t nChannelNo;//ͨ����
	uint16_t nS0;		//��һ����ȡ���λ��S0
	uint16_t nL0;		//��һ����ȡ����L0
	uint16_t nD0[320];	//����D0
	uint16_t nS1;		//�ڶ�����ȡ���λ��S1
	uint16_t nL1;		//�ڶ�����ȡ����L1
	uint16_t nD1;		//����D1
	uint32_t nTest;		//��֤����
	HS_Lidar_Channel() {};
	~HS_Lidar_Channel() {};
};


#endif