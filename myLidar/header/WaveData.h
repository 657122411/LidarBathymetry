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


//高斯函数参数结构体
struct GaussParameter {
    float A;        //振幅（Ymax）
    float b;        //脉冲距离(对称轴)
    float sigma;    //脉冲宽度（宽幅）
    bool wavetype;    //该高斯组量的水表水底类型
};


//计算数据的标准差
float calculateSigma(vector<float> resultSet);


//波形数据类
class WaveData {
public:
    WaveData();

    ~WaveData();

    void GetData(HS_Lidar &hs);                                                //截取兴趣数据
    void Filter(vector<float> &srcWave, float &noise);                        //滤波平滑
    void Resolve(vector<float> &srcWave, vector<GaussParameter> &waveParam, float &noise);    //分解高斯分量参数
    void Optimize(vector<float> &srcWave, vector<GaussParameter> &waveParam);//迭代优化（LM）

    static bool ostreamFlag;                                                //控制流输出的兴趣通道数据
    friend ostream &operator<<(ostream &stream, const WaveData &wavedata);    //自定义输出信息

    Time m_time;                                    //UTC时间
    vector<float> m_BlueWave;                        //CH2通道数据
    vector<float> m_GreenWave;                        //CH3通道数据
    float m_BlueNoise;                                //CH2通道的随机噪声
    float m_GreenNoise;                                //CH3通道的随机噪声
    vector<GaussParameter> m_BlueGauPra;            //CH2数据高斯分量参数
    vector<GaussParameter> m_GreenGauPra;            //CH3数据高斯分量参数
    vector<GaussParameter>::iterator gaussPraIter;    //高斯参数结构体迭代器

    float blueDepth;                                //CH2通道的计算水深
    float greenDepth;                                //CH3通道的计算水深
    void CalcuDepth(vector<GaussParameter> &waveParam, float &BorGDepth);    //根据回波数据计算水深
    void CalcuDepthByGauss(vector<GaussParameter> &waveParam, float &BorGDepth);    //普通高斯分解
};