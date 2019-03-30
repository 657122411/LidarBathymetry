#pragma once
#include <iostream>
#include <vector>
#include <math.h>
#include "HS_Lidar.h"
#include "TimeConvert.h"
#include "levmar.h"
using namespace std;


//�������ݵı�׼��
float calculateDeepSigma(vector<float> resultSet);

//��ˮ����������
class DeepWave
{
public:
	DeepWave();
	~DeepWave();
	void GetDeepData(HS_Lidar &hs);											//��ȡ��ˮ��������
	void DeepFilter(vector<float> &srcWave, float &noise);					//�˲�ƽ��
	void DeepResolve(vector<float> &srcWave, vector<float> &waveParam, float &noise);	//�ֽ��������
	void DeepOptimize(vector<float> &srcWave, vector<float> &waveParam);	//�����Ż���LM��

	static bool ostreamFlag;												//�������������Ȥͨ������
	friend ostream &operator<<(ostream &stream, const DeepWave &deepwave);	//�Զ��������Ϣ
	Time m_time;									//UTCʱ��

	vector<float> m_RedDeep;						//CH1������ͨ����ˮ����
	vector<float> m_BlueDeep;						//CH2ͨ����ˮ����
	vector<float> m_GreenDeep;						//CH3ͨ����ˮ����
	float m_BlueDeepNoise;							//CH2ͨ�����������
	float m_GreenDeepNoise;							//CH3ͨ�����������
	vector<float> m_BlueDeepPra;					//CH2���ݷ�ֵ������
	vector<float> m_GreenDeepPra;					//CH3���ݷ�ֵ������

	int redTime;									//������ͨ����ˮ���
	void GetRedTime(vector<float> &srcWave, int &redtime);//��ȡ������ͨ��ˮ���ʱ��
	void CalcuDeepDepthByRed(vector<float> &waveParam, int &redtime, float &BorGDepth);	//���ݽ�����ͨ��ˮ�������ͨ��ˮ�׼���ˮ��

	float blueDeepDepth;							//CH2ͨ���ļ���ˮ��
	float greenDeepDepth;							//CH3ͨ���ļ���ˮ��
	void CalcuDeepDepth(vector<float> &waveParam, float &BorGDepth);	//���ݻز����ݼ���ˮ��
};