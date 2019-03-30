#ifndef HS_Lidar_Header_H
#define HS_Lidar_Header_H

#include <stdint.h>
class HS_Lidar_Header
{
public:
	uint16_t nFill;				//֡ͷ
	uint16_t nGPSWeek;			//GPS��
	double dGPSSecond;			//GPS��
	uint32_t nGPSBreakdownTime;	//ϸ��ʱ��
	double dAzimuth;			//��λ��(ƫ����)
	double dPitch;				//������
	double dRoll;				//������(�����)
	double dX;					//GPSγ��
	double dY;					//GPS����
	double dZ;					//GPS�߶�
	uint32_t nCodeDiscResolution;//����λ��
	uint32_t nCodeNumber;		//���̶���
	uint32_t nWaveNumber;		//����ͨ����
	uint32_t nWaveLen;			//���γ���
	HS_Lidar_Header() {};
	~HS_Lidar_Header() {};
};


#endif