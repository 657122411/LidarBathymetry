#pragma once

#include <iostream>
#include <vector>
#include <math.h>
#include "HS_Lidar.h"
#include "TimeConvert.h"

extern "C" {
#include "levmar.h"
}

using namespace std;


//��˹���������ṹ��
struct GaussParameter {
    float A;        //�����Ymax��
    float b;        //�������(�Գ���)
    float sigma;    //�����ȣ������
    bool wavetype;    //�ø�˹������ˮ��ˮ������
};


//�������ݵı�׼��
float calculateSigma(vector<float> resultSet);


//����������
class WaveData {
public:
    WaveData();

    ~WaveData();

    void GetData(HS_Lidar &hs);                                                //��ȡ��Ȥ����
    void Filter(vector<float> &srcWave, float &noise);                        //�˲�ƽ��
    void Resolve(vector<float> &srcWave, vector<GaussParameter> &waveParam, float &noise);    //�ֽ��˹��������
    void Optimize(vector<float> &srcWave, vector<GaussParameter> &waveParam);//�����Ż���LM��

    static bool ostreamFlag;                                                //�������������Ȥͨ������
    friend ostream &operator<<(ostream &stream, const WaveData &wavedata);    //�Զ��������Ϣ

    Time m_time;                                    //UTCʱ��
    vector<float> m_BlueWave;                        //CH2ͨ������
    vector<float> m_GreenWave;                        //CH3ͨ������
    float m_BlueNoise;                                //CH2ͨ�����������
    float m_GreenNoise;                                //CH3ͨ�����������
    vector<GaussParameter> m_BlueGauPra;            //CH2���ݸ�˹��������
    vector<GaussParameter> m_GreenGauPra;            //CH3���ݸ�˹��������
    vector<GaussParameter>::iterator gaussPraIter;    //��˹�����ṹ�������

    float blueDepth;                                //CH2ͨ���ļ���ˮ��
    float greenDepth;                                //CH3ͨ���ļ���ˮ��
    void CalcuDepth(vector<GaussParameter> &waveParam, float &BorGDepth);    //���ݻز����ݼ���ˮ��
    void CalcuDepthByGauss(vector<GaussParameter> &waveParam, float &BorGDepth);    //��ͨ��˹�ֽ�
};