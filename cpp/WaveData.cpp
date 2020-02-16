#include "../header/WaveData.h"
#include <numeric>

#define PulseWidth 4        //���弤����������������ֵ�ο�
#define TimeDifference 8    //��UTC��ʱ��

#define BLUE true
#define GREEN false

#define c 0.3                //��Թ��ٳ�������
#define nwater 1.334        //ˮ�ʵ�������


//�ز�����
#define SURFACE true        //ˮ��ز������ܰ�������ɢ�䣩
#define BOTTOM false        //ˮ�׻�ˮ�����ʻز�


extern float Theta;


bool WaveData::ostreamFlag = BLUE;


/*���ܣ�  ��˹������
//kernel���洢���ɵĸ�˹��
//size��  �˵Ĵ�С
//sigma�� ��̬�ֲ���׼��
*/
void gau_kernel(float kernel[], int size, float sigma) {
    if (size <= 0 || sigma == 0)
        return;
    int x;
    float sum = 0;
    int m = (size - 1) / 2;

    //get kernel
    for (x = 0; x < size; x++) {
        kernel[x] = (1 / sigma * sqrt(2 * 3.1415)) * exp(-(x - m) * (x - m) / 2 * sigma * sigma);
        sum += kernel[x];
    }

    //normal
    for (x = 0; x < size; x++) {
        kernel[x] /= sum;
    }
}


/*����:	 ��˹ģ��
//src��  ����ԭͼ
//dst��  ģ��ͼ��
//size�� �˵Ĵ�С
//sigma����̬�ֲ���׼��
*/
void gaussian(float src[], float dst[]) {
    float kernel[5];
    gau_kernel(kernel, 5, 1);
    //gaussian���,��ʱ�߽�û�Ӵ���
    for (int i = (5 - 1) / 2; i <= 319 - (5 - 1) / 2; i++) {
        dst[i] = src[i - 2] * kernel[0] + src[i - 1] * kernel[1] + src[i] * kernel[2] + src[i + 1] * kernel[3] +
                 src[i + 2] * kernel[4];
    }
}


/*���ܣ�	�������ݵı�׼��
//:
//resultSet���������������
//stdev��	����ֵΪ��׼��
//
*/
float calculateSigma(vector<float> resultSet) {
    double sum = std::accumulate(std::begin(resultSet), std::end(resultSet), 0.0);
    double mean = sum / resultSet.size(); //��ֵ

    double accum = 0.0;
    for (vector<float>::iterator iter = resultSet.begin(); iter != resultSet.end(); iter++) {
        accum += (*iter - mean) * (*iter - mean);
    };

    float stdev = sqrt(accum / (resultSet.size() - 1)); //����

    return stdev;
}


/*���ܣ�	���������˹����ģ��
// *p:	�������
// *x��  ԭʼ���ݣ�����ֵ��
// m��	����ά��
// n��	����ֵά��
// *data:��
*/
void expfun2(double *p, double *x, int m, int n, void *data) {
    register int i;
    for (i = 0; i < n; ++i) {
        //д��������x[i]֮��Ĺ�ϵʽ���������﷽�̵��ұ�û�й۲�ֵ������ֻ�в���
        x[i] = p[0] * exp(-(i - p[1]) * (i - p[1]) / (2 * p[2] * p[2]))
               + p[3] * exp(-(i - p[4]) * (i - p[4]) / (2 * p[5] * p[5]));
    }
}


/*���ܣ�	�����˹����ģ�͵��ſɱȾ���
// *p:	�������
// jac�� �ſɱȾ������
// m��	����ά��
// n��	����ֵά��
// *data:��
*/
void jacexpfun2(double *p, double *jac, int m, int n, void *data) {
    register int i, j;
    //д���ſ˱Ⱦ���
    for (i = j = 0; i < n; ++i) {
        jac[j++] = exp(-(i - p[1]) * (i - p[1]) / (2 * p[2]) * p[2]);
        jac[j++] = p[0] * (i - p[1]) / (p[2] * p[2]) * exp(-(i - p[1]) * (i - p[1]) / (2 * p[2] * p[2]));
        jac[j++] = p[0] * (i - p[1]) * (i - p[1]) / (p[2] * p[2] * p[2]) *
                   exp(-(i - p[1]) * (i - p[1]) / (2 * p[2] * p[2]));

        jac[j++] = exp(-(i - p[4]) * (i - p[4]) / (2 * p[5]) * p[5]);
        jac[j++] = p[3] * (i - p[4]) / (p[5] * p[5]) * exp(-(i - p[4]) * (i - p[4]) / (2 * p[5] * p[5]));
        jac[j++] = p[3] * (i - p[4]) * (i - p[4]) / (p[5] * p[5] * p[5]) *
                   exp(-(i - p[4]) * (i - p[4]) / (2 * p[5] * p[5]));
    }
}


/*���ܣ�	���������˹����ģ��
// *p:	�������
// *x��  ԭʼ���ݣ�����ֵ��
// m��	����ά��
// n��	����ֵά��
// *data:��
*/
void expfun3(double *p, double *x, int m, int n, void *data) {
    register int i;
    for (i = 0; i < n; ++i) {
        //д��������x[i]֮��Ĺ�ϵʽ���������﷽�̵��ұ�û�й۲�ֵ������ֻ�в���
        x[i] = p[0] * exp(-(i - p[1]) * (i - p[1]) / (2 * p[2] * p[2]))
               + p[3] * exp(-(i - p[4]) * (i - p[4]) / (2 * p[5] * p[5]))
               + p[6] * exp(-(i - p[7]) * (i - p[7]) / (2 * p[8] * p[8]));
    }
}


/*���ܣ�	�����˹����ģ�͵��ſɱȾ���
// *p:	�������
// jac�� �ſɱȾ������
// m��	����ά��
// n��	����ֵά��
// *data:��
*/
void jacexpfun3(double *p, double *jac, int m, int n, void *data) {
    register int i, j;
    //д���ſ˱Ⱦ���
    for (i = j = 0; i < n; ++i) {
        jac[j++] = exp(-(i - p[1]) * (i - p[1]) / (2 * p[2]) * p[2]);
        jac[j++] = p[0] * (i - p[1]) / (p[2] * p[2]) * exp(-(i - p[1]) * (i - p[1]) / (2 * p[2] * p[2]));
        jac[j++] = p[0] * (i - p[1]) * (i - p[1]) / (p[2] * p[2] * p[2]) *
                   exp(-(i - p[1]) * (i - p[1]) / (2 * p[2] * p[2]));

        jac[j++] = exp(-(i - p[4]) * (i - p[4]) / (2 * p[5]) * p[5]);
        jac[j++] = p[3] * (i - p[4]) / (p[5] * p[5]) * exp(-(i - p[4]) * (i - p[4]) / (2 * p[5] * p[5]));
        jac[j++] = p[3] * (i - p[4]) * (i - p[4]) / (p[5] * p[5] * p[5]) *
                   exp(-(i - p[4]) * (i - p[4]) / (2 * p[5] * p[5]));

        jac[j++] = exp(-(i - p[7]) * (i - p[7]) / (2 * p[8]) * p[8]);
        jac[j++] = p[6] * (i - p[7]) / (p[8] * p[8]) * exp(-(i - p[7]) * (i - p[7]) / (2 * p[8] * p[8]));
        jac[j++] = p[6] * (i - p[7]) * (i - p[7]) / (p[8] * p[8] * p[8]) *
                   exp(-(i - p[7]) * (i - p[7]) / (2 * p[8] * p[8]));
    }
}


/*���ܣ�	���������˹����ģ��
// *p:	�������
// *x��  ԭʼ���ݣ�����ֵ��
// m��	����ά��
// n��	����ֵά��
// *data:��
*/
void expfun4(double *p, double *x, int m, int n, void *data) {
    register int i;
    for (i = 0; i < n; ++i) {
        //д��������x[i]֮��Ĺ�ϵʽ���������﷽�̵��ұ�û�й۲�ֵ������ֻ�в���
        x[i] = p[0] * exp(-(i - p[1]) * (i - p[1]) / (2 * p[2] * p[2]))
               + p[3] * exp(-(i - p[4]) * (i - p[4]) / (2 * p[5] * p[5]))
               + p[6] * exp(-(i - p[7]) * (i - p[7]) / (2 * p[8] * p[8]))
               + p[9] * exp(-(i - p[10]) * (i - p[10]) / (2 * p[11] * p[11]));
    }
}


/*���ܣ�	�����˹����ģ�͵��ſɱȾ���
// *p:	�������
// jac�� �ſɱȾ������
// m��	����ά��
// n��	����ֵά��
// *data:��
*/
void jacexpfun4(double *p, double *jac, int m, int n, void *data) {
    register int i, j;
    //д���ſ˱Ⱦ���
    for (i = j = 0; i < n; ++i) {
        jac[j++] = exp(-(i - p[1]) * (i - p[1]) / (2 * p[2]) * p[2]);
        jac[j++] = p[0] * (i - p[1]) / (p[2] * p[2]) * exp(-(i - p[1]) * (i - p[1]) / (2 * p[2] * p[2]));
        jac[j++] = p[0] * (i - p[1]) * (i - p[1]) / (p[2] * p[2] * p[2]) *
                   exp(-(i - p[1]) * (i - p[1]) / (2 * p[2] * p[2]));

        jac[j++] = exp(-(i - p[4]) * (i - p[4]) / (2 * p[5]) * p[5]);
        jac[j++] = p[3] * (i - p[4]) / (p[5] * p[5]) * exp(-(i - p[4]) * (i - p[4]) / (2 * p[5] * p[5]));
        jac[j++] = p[3] * (i - p[4]) * (i - p[4]) / (p[5] * p[5] * p[5]) *
                   exp(-(i - p[4]) * (i - p[4]) / (2 * p[5] * p[5]));

        jac[j++] = exp(-(i - p[7]) * (i - p[7]) / (2 * p[8]) * p[8]);
        jac[j++] = p[6] * (i - p[7]) / (p[8] * p[8]) * exp(-(i - p[7]) * (i - p[7]) / (2 * p[8] * p[8]));
        jac[j++] = p[6] * (i - p[7]) * (i - p[7]) / (p[8] * p[8] * p[8]) *
                   exp(-(i - p[7]) * (i - p[7]) / (2 * p[8] * p[8]));

        jac[j++] = exp(-(i - p[10]) * (i - p[10]) / (2 * p[11]) * p[11]);
        jac[j++] = p[9] * (i - p[10]) / (p[11] * p[11]) * exp(-(i - p[10]) * (i - p[10]) / (2 * p[11] * p[11]));
        jac[j++] = p[9] * (i - p[10]) * (i - p[10]) / (p[11] * p[11] * p[11]) *
                   exp(-(i - p[10]) * (i - p[10]) / (2 * p[11] * p[11]));
    }
}


/*���ܣ�	���������˹����ģ��
// *p:	�������
// *x��  ԭʼ���ݣ�����ֵ��
// m��	����ά��
// n��	����ֵά��
// *data:��
*/
void expfun5(double *p, double *x, int m, int n, void *data) {
    register int i;
    for (i = 0; i < n; ++i) {
        //д��������x[i]֮��Ĺ�ϵʽ���������﷽�̵��ұ�û�й۲�ֵ������ֻ�в���
        x[i] = p[0] * exp(-(i - p[1]) * (i - p[1]) / (2 * p[2] * p[2]))
               + p[3] * exp(-(i - p[4]) * (i - p[4]) / (2 * p[5] * p[5]))
               + p[6] * exp(-(i - p[7]) * (i - p[7]) / (2 * p[8] * p[8]))
               + p[9] * exp(-(i - p[10]) * (i - p[10]) / (2 * p[11] * p[11]))
               + p[12] * exp(-(i - p[13]) * (i - p[13]) / (2 * p[14] * p[14]));
    }
}


/*���ܣ�	�����˹����ģ�͵��ſɱȾ���
// *p:	�������
// jac�� �ſɱȾ������
// m��	����ά��
// n��	����ֵά��
// *data:��
*/
void jacexpfun5(double *p, double *jac, int m, int n, void *data) {
    register int i, j;
    //д���ſ˱Ⱦ���
    for (i = j = 0; i < n; ++i) {
        jac[j++] = exp(-(i - p[1]) * (i - p[1]) / (2 * p[2]) * p[2]);
        jac[j++] = p[0] * (i - p[1]) / (p[2] * p[2]) * exp(-(i - p[1]) * (i - p[1]) / (2 * p[2] * p[2]));
        jac[j++] = p[0] * (i - p[1]) * (i - p[1]) / (p[2] * p[2] * p[2]) *
                   exp(-(i - p[1]) * (i - p[1]) / (2 * p[2] * p[2]));

        jac[j++] = exp(-(i - p[4]) * (i - p[4]) / (2 * p[5]) * p[5]);
        jac[j++] = p[3] * (i - p[4]) / (p[5] * p[5]) * exp(-(i - p[4]) * (i - p[4]) / (2 * p[5] * p[5]));
        jac[j++] = p[3] * (i - p[4]) * (i - p[4]) / (p[5] * p[5] * p[5]) *
                   exp(-(i - p[4]) * (i - p[4]) / (2 * p[5] * p[5]));

        jac[j++] = exp(-(i - p[7]) * (i - p[7]) / (2 * p[8]) * p[8]);
        jac[j++] = p[6] * (i - p[7]) / (p[8] * p[8]) * exp(-(i - p[7]) * (i - p[7]) / (2 * p[8] * p[8]));
        jac[j++] = p[6] * (i - p[7]) * (i - p[7]) / (p[8] * p[8] * p[8]) *
                   exp(-(i - p[7]) * (i - p[7]) / (2 * p[8] * p[8]));

        jac[j++] = exp(-(i - p[10]) * (i - p[10]) / (2 * p[11]) * p[11]);
        jac[j++] = p[9] * (i - p[10]) / (p[11] * p[11]) * exp(-(i - p[10]) * (i - p[10]) / (2 * p[11] * p[11]));
        jac[j++] = p[9] * (i - p[10]) * (i - p[10]) / (p[11] * p[11] * p[11]) *
                   exp(-(i - p[10]) * (i - p[10]) / (2 * p[11] * p[11]));

        jac[j++] = exp(-(i - p[13]) * (i - p[13]) / (2 * p[14]) * p[14]);
        jac[j++] = p[12] * (i - p[13]) / (p[14] * p[14]) * exp(-(i - p[13]) * (i - p[13]) / (2 * p[14] * p[14]));
        jac[j++] = p[12] * (i - p[13]) * (i - p[13]) / (p[14] * p[14] * p[14]) *
                   exp(-(i - p[13]) * (i - p[13]) / (2 * p[14] * p[14]));
    }
}


/*���ܣ�	���������˹����ģ��
// *p:	�������
// *x��  ԭʼ���ݣ�����ֵ��
// m��	����ά��
// n��	����ֵά��
// *data:��
*/
void expfun6(double *p, double *x, int m, int n, void *data) {
    register int i;
    for (i = 0; i < n; ++i) {
        //д��������x[i]֮��Ĺ�ϵʽ���������﷽�̵��ұ�û�й۲�ֵ������ֻ�в���
        x[i] = p[0] * exp(-(i - p[1]) * (i - p[1]) / (2 * p[2] * p[2]))
               + p[3] * exp(-(i - p[4]) * (i - p[4]) / (2 * p[5] * p[5]))
               + p[6] * exp(-(i - p[7]) * (i - p[7]) / (2 * p[8] * p[8]))
               + p[9] * exp(-(i - p[10]) * (i - p[10]) / (2 * p[11] * p[11]))
               + p[12] * exp(-(i - p[13]) * (i - p[13]) / (2 * p[14] * p[14]))
               + p[15] * exp(-(i - p[16]) * (i - p[16]) / (2 * p[17] * p[17]));
    }
}


/*���ܣ�	�����˹����ģ�͵��ſɱȾ���
// *p:	�������
// jac�� �ſɱȾ������
// m��	����ά��
// n��	����ֵά��
// *data:��
*/
void jacexpfun6(double *p, double *jac, int m, int n, void *data) {
    register int i, j;
    //д���ſ˱Ⱦ���
    for (i = j = 0; i < n; ++i) {
        jac[j++] = exp(-(i - p[1]) * (i - p[1]) / (2 * p[2]) * p[2]);
        jac[j++] = p[0] * (i - p[1]) / (p[2] * p[2]) * exp(-(i - p[1]) * (i - p[1]) / (2 * p[2] * p[2]));
        jac[j++] = p[0] * (i - p[1]) * (i - p[1]) / (p[2] * p[2] * p[2]) *
                   exp(-(i - p[1]) * (i - p[1]) / (2 * p[2] * p[2]));

        jac[j++] = exp(-(i - p[4]) * (i - p[4]) / (2 * p[5]) * p[5]);
        jac[j++] = p[3] * (i - p[4]) / (p[5] * p[5]) * exp(-(i - p[4]) * (i - p[4]) / (2 * p[5] * p[5]));
        jac[j++] = p[3] * (i - p[4]) * (i - p[4]) / (p[5] * p[5] * p[5]) *
                   exp(-(i - p[4]) * (i - p[4]) / (2 * p[5] * p[5]));

        jac[j++] = exp(-(i - p[7]) * (i - p[7]) / (2 * p[8]) * p[8]);
        jac[j++] = p[6] * (i - p[7]) / (p[8] * p[8]) * exp(-(i - p[7]) * (i - p[7]) / (2 * p[8] * p[8]));
        jac[j++] = p[6] * (i - p[7]) * (i - p[7]) / (p[8] * p[8] * p[8]) *
                   exp(-(i - p[7]) * (i - p[7]) / (2 * p[8] * p[8]));

        jac[j++] = exp(-(i - p[10]) * (i - p[10]) / (2 * p[11]) * p[11]);
        jac[j++] = p[9] * (i - p[10]) / (p[11] * p[11]) * exp(-(i - p[10]) * (i - p[10]) / (2 * p[11] * p[11]));
        jac[j++] = p[9] * (i - p[10]) * (i - p[10]) / (p[11] * p[11] * p[11]) *
                   exp(-(i - p[10]) * (i - p[10]) / (2 * p[11] * p[11]));

        jac[j++] = exp(-(i - p[13]) * (i - p[13]) / (2 * p[14]) * p[14]);
        jac[j++] = p[12] * (i - p[13]) / (p[14] * p[14]) * exp(-(i - p[13]) * (i - p[13]) / (2 * p[14] * p[14]));
        jac[j++] = p[12] * (i - p[13]) * (i - p[13]) / (p[14] * p[14] * p[14]) *
                   exp(-(i - p[13]) * (i - p[13]) / (2 * p[14] * p[14]));

        jac[j++] = exp(-(i - p[16]) * (i - p[16]) / (2 * p[17]) * p[17]);
        jac[j++] = p[15] * (i - p[16]) / (p[17] * p[17]) * exp(-(i - p[16]) * (i - p[16]) / (2 * p[17] * p[17]));
        jac[j++] = p[15] * (i - p[16]) * (i - p[16]) / (p[17] * p[17] * p[17]) *
                   exp(-(i - p[16]) * (i - p[16]) / (2 * p[17] * p[17]));
    }
}


/*
//���ܣ����캯����ʼ������
*/
WaveData::WaveData() {
    m_time = {0, 0, 0, 0, 0, 0};
    m_BlueNoise = 0;
    m_GreenNoise = 0;
    blueDepth = 0;
    greenDepth = 0;
}


WaveData::~WaveData() {
    //�ֶ��ͷ�vector�ڴ棬��֪����û�б�Ҫ��
//    vector<float>().swap(m_BlueWave);
//    vector<float>().swap(m_GreenWave);
//    vector<GaussParameter>().swap(m_BlueGauPra);
//    vector<GaussParameter>().swap(m_GreenGauPra);
}


/*���ܣ�	��ȡԭʼ�����е���Ȥ��������
//&hs:	ԭʼLidar����
*/
void WaveData::GetData(HS_Lidar &hs) {
    //GPS->UTC->BeiJing
    PGPSTIME pgt = new GPSTIME;
    PCOMMONTIME pct = new COMMONTIME;
    pgt->wn = (int) hs.header.nGPSWeek;
    pgt->tow.sn = (long) hs.header.dGPSSecond;
    pgt->tow.tos = 0;
    GPSTimeToCommonTime(pgt, pct);
    m_time.year = pct->year;
    m_time.month = pct->month;
    m_time.day = pct->day;
    m_time.hour = pct->hour + TimeDifference;    //ֱ��ת��Ϊ����ʱ��
    m_time.minute = pct->minute;
    m_time.second = pct->second;
    delete pgt;
    delete pct;

    //ȡ����ͨ��
    m_BlueWave.assign(&hs.CH2.nD0[0], &hs.CH2.nD0[320]);
    m_GreenWave.assign(&hs.CH3.nD0[0], &hs.CH3.nD0[320]);
}


/*���ܣ�		Ԥ�������ݣ���ȡ��Ч���ֲ�����ȥ���˲�����
//&srcWave:	ͨ��ԭʼ����
//&noise��	��¼��������������
*/
void WaveData::Filter(vector<float> &srcWave, float &noise) {
    /*
    //��Ч���ݵĽ�ȡ����ʵ��Ч�������á���ʱû�м���
    int m = 30, n=30;//��Ȥ���������˵�
    vector<float> vm(srcWave.begin(), srcWave.begin() + m);
    float Svm = calculateSigma(vm);

    //ǰ������ȡ��
    for (int i = 30; i < srcWave.size()-1; i++)
    {
        if (srcWave.at(i - 1) > srcWave.at(i) && srcWave.at(i) < srcWave.at(i + 1))
        {
            m = i;
            for (int j = i+2; j < srcWave.size() - 1; j++)
            {
                if (srcWave.at(j - 1) < srcWave.at(j) && srcWave.at(j) > srcWave.at(j + 1))
                    n = j;
                    break;
            }
            if (n > m)
            {
                vector<float> v1(srcWave.begin(), srcWave.begin() + m);
                vector<float> v2(srcWave.begin(), srcWave.begin() + n);
                float Sv1 = calculateSigma(v1);
                float Sv2 = calculateSigma(v2);
                if (Sv1 > 1.5*Svm||Sv2 > 2 * Sv1)
                    break;
            }

        }
    }
    int k = 50, l = 50;//��Ȥ���������˵�
    vector<float> vk(srcWave.end()-k, srcWave.end());
    float Svk = calculateSigma(vk);

    //����ǰ����ȡ��
    for (int i = 50; i < srcWave.size() - 1; i++)
    {
        if (srcWave.at(320-(i - 1)) > srcWave.at(320-i) && srcWave.at(320-i) < srcWave.at(320-(i + 1)))
        {
            k = i;
            for (int j = i + 2; j < srcWave.size() - 1; j++)
            {
                if (srcWave.at(320-(j - 1)) < srcWave.at(320-j) && srcWave.at(320-j) > srcWave.at(320-(j + 1)))
                    l = j;
                break;
            }
            if (l > k)
            {
                vector<float> v1(srcWave.end() - k, srcWave.end());
                vector<float> v2(srcWave.end() - l, srcWave.end());
                float Sv1 = calculateSigma(v1);
                float Sv2 = calculateSigma(v2);
                if (Sv1 > 1.5 * Svk || Sv2 > 2 * Sv1)
                    break;
            }
        }
    }

    if ((n + 10) <= (320 - k))
    {
        for (int i = m - 1; i >= 0; i--)
        {
            srcWave.at(i) = srcWave.at(m);
        }
        for (int j = 320 - (k - 1); j <= 319; j++)
        {
            srcWave.at(j) = srcWave.at(320 - k);
        }
    }
    */

    //��˹�˲�ȥ��
    vector<float> dstWave;
    dstWave.assign(srcWave.begin(), srcWave.end());
    gaussian(&srcWave[0], &dstWave[0]);

    noise = 0;
    //�����������:�����˲�ǰ��Ĳ������ݵķ�ֵ��ľ������׼�
    for (int i = 0; i < srcWave.size(); i++) {
        noise += (srcWave.at(i) - dstWave.at(i)) * (srcWave.at(i) - dstWave.at(i));
    }
    noise = sqrt(noise / srcWave.size());

    srcWave.assign(dstWave.begin(), dstWave.end());
}


/*���ܣ�			��˹�����ֽ⺯��
//&srcWave:		ͨ��ԭʼ����
//&waveParam��	��ͨ���ĸ�˹��������
*/
void WaveData::Resolve(vector<float> &srcWave, vector<GaussParameter> &waveParam, float &noise) {
    //����ԭʼ����
    float data[320], temp[320];
    int i = 0, m = 0;
    for (vector<float>::iterator iter = srcWave.begin(); iter != srcWave.end(); ++iter, ++i) {
        data[i] = *iter;
    }

    //���˲����������Сֵ��Ϊ��������
    float backgroundNoise = data[0];
    for (m = 0; m < 320; m++) {
        temp[m] = data[m];
        if (data[m] < backgroundNoise)
            backgroundNoise = data[m];
    }

    //�������ݳ�ȥ��������
    for (m = 0; m < 320; m++) {
        temp[m] -= backgroundNoise;
    }
    srcWave.assign(&temp[0], &temp[320]);

    float A;    //���
    float b;    //�������
    float tg;    //��ֵʱ��λ��
    float tgl;    //���ʱ��λ�ã���)
    float tgr;    //���ʱ��λ�ã��ң�

    bool wavetypeFlag = true;            //�����ж�ˮ��ˮ�׻ز������flag
    float surfaceMin, surfaceMax;    //ˮ��ز�λ�����ڵĿ��Ʒ�Χ

    //ѭ���������
    do {
        A = 0;
        //�����ֵ����¼λ��
        for (m = 0; m < 320; m++) {
            if (temp[m] > A) {
                A = temp[m];
                b = m;
            }
        }

        //Ѱ�Ұ��λ��
        for (m = b; m < 319; m++) {
            if ((temp[m - 1] > A / 2) && (temp[m + 1] < A / 2)) {
                tgr = m;
                break;
            }
        }
        for (m = b; m > 0; m--) {
            if ((temp[m - 1] < A / 2) && (temp[m + 1] > A / 2)) {
                tgl = m;
                break;
            }
        }
        if ((b - tgl) > (tgr - b)) {
            tg = tgr;
        } else {
            tg = tgl;
        }

        //����sigma
        float sigma = fabs(tg - b) / sqrt(2 * log(2));

        //�ж�ˮ��ˮ�׻ز�
        if (wavetypeFlag == true) {
            //���Ҳ�յ�
            surfaceMax = b;
            float rval = abs(temp[(int) b + 1] - temp[(int) b]);
            for (int i = b; b < 320 - b; i++) {
                if (abs(temp[i + 1] - temp[i]) >= rval)
                    rval = abs(temp[i + 1] - temp[i]);
                else {
                    surfaceMax = i + 2;
                    break;
                }
            }
            //�����յ�
            surfaceMin = b;
            float lval = abs(temp[(int) b - 1] - temp[(int) b]);
            for (int i = b; b > 0; i--) {
                if (abs(temp[i - 1] - temp[i]) >= lval)
                    lval = abs(temp[i - 1] - temp[i]);
                else {
                    surfaceMin = i - 2;
                    break;
                }
            }

            wavetypeFlag = false;

        }

        if (surfaceMin <= b && b <= surfaceMax)//��ѡ�������ڵĸ�˹����Ϊͬһ�������ʵˮ��+����ɢ�䣩
        {
            //�������˹��������ѹ������
            GaussParameter param{A, b, sigma, SURFACE};
            waveParam.push_back(param);
        } else {
            //�������˹��������ѹ������
            GaussParameter param{A, b, sigma, BOTTOM};
            waveParam.push_back(param);
        }

        //����
        for (m = 0; m < 320; m++) {
            if (temp[m] > A * exp(-(m - b) * (m - b) / (2 * sigma * sigma))) {
                temp[m] -= A * exp(-(m - b) * (m - b) / (2 * sigma * sigma));
            } else
                temp[m] = 0;
        }

        //�ж��Ƿ��������
        A = 0;
        for (m = 0; m < 320; m++) {
            if (temp[m] > A) {
                A = temp[m];
            }
        }


    } while (A > 3 * noise);//ѭ������!!!ֵ��̽��


    //�Ը�˹������ɸѡ��ʱ����С��һ��ֵ���޳�������С�ķ���������vector�����sigmaֵ��Ϊ0
    for (int i = 0; i < waveParam.size() - 1; i++) {
        for (int j = i + 1; j < waveParam.size(); j++) {
            if (abs(waveParam.at(i).b - waveParam.at(j).b) < PulseWidth)//Key
            {
                if (waveParam.at(i).A >= waveParam.at(j).A) {
                    waveParam.at(j).sigma = 0;
                } else {
                    waveParam.at(i).sigma = 0;
                }
            }
        }
    }

    //�ٽ�sigmaС����ֵ�ķ����޳�
    for (gaussPraIter = waveParam.begin(); gaussPraIter != waveParam.end();) {
        if (gaussPraIter->sigma < ((float) PulseWidth / 8)) {
            gaussPraIter = waveParam.erase(gaussPraIter);
        } else {
            ++gaussPraIter;
        }
    }
}


/*���ܣ�			LM�㷨�����Ż�
//&srcWave:		ͨ��ԭʼ����
//&waveParam��	��ͨ���ĸ�˹��������
//LM�㷨�ο���	https://blog.csdn.net/shajun0153/article/details/75073137
*/
void WaveData::Optimize(vector<float> &srcWave, vector<GaussParameter> &waveParam) {
    int size = waveParam.size();
    //�����ֵΪ˫������
    switch (size) {
        case 2: {
            //��ȡ��˹��������
            double p[6];
            int i = 0;
            for (auto gp : waveParam) {
                p[i++] = gp.A;
                p[i++] = gp.b;
                p[i++] = gp.sigma;
            }
            int m = i;
            int n = srcWave.size();

            //��ȡ�������
            double x[320];
            i = 0;
            for (vector<float>::iterator iter = srcWave.begin(); iter != srcWave.end(); ++iter, ++i) {
                x[i] = *iter;
            }

            double info[LM_INFO_SZ];
            // ���õ�����ں���
            int ret = dlevmar_der(expfun2,    //��������ֵ֮���ϵ�ĺ���ָ��
                                  jacexpfun2,                    //�����ſ˱Ⱦ���ĺ���ָ��
                                  p,                            //��ʼ���Ĵ�����������һ������������
                                  x,                            //����ֵ
                                  m,                            //����ά��
                                  n,                            //����ֵά��
                                  1000,                        //����������
                                  NULL,                        //opts,       //������һЩ����
                                  info,                        //������С�������һЩ����������Ҫ��ΪNULL
                                  NULL, NULL, NULL            //һЩ�ڴ��ָ�룬��ʱ����Ҫ
            );
            /*printf("Levenberg-Marquardt returned in %g iter, reason %g, sumsq %g [%g]\n", info[5], info[6], info[1], info[0]);
            printf("Bestfit parameters: A:%.7g b:%.7g sigma:%.7g A:%.7g b:%.7g sigma:%.7g\n", p[0], p[1], p[2], p[3], p[4], p[5]);
            printf("����ʱ���: %.7g ns\n", abs(p[4] - p[1]));*/

            //���Ż���Ĳ����鸳��vector
            i = 0;
            for (gaussPraIter = waveParam.begin(); gaussPraIter != waveParam.end(); gaussPraIter++) {
                gaussPraIter->A = p[i++];
                gaussPraIter->b = p[i++];
                gaussPraIter->sigma = p[i++];
            }

            break;
        }
        case 3: {
            //��ȡ��˹��������
            double p[9];
            int i = 0;
            for (auto gp : waveParam) {
                p[i++] = gp.A;
                p[i++] = gp.b;
                p[i++] = gp.sigma;
            }
            int m = i;
            int n = srcWave.size();

            //��ȡ�������
            double x[320];
            i = 0;
            for (vector<float>::iterator iter = srcWave.begin(); iter != srcWave.end(); ++iter, ++i) {
                x[i] = *iter;
            }

            double info[LM_INFO_SZ];
            // ���õ�����ں���
            int ret = dlevmar_der(expfun3,    //��������ֵ֮���ϵ�ĺ���ָ��
                                  jacexpfun3,                    //�����ſ˱Ⱦ���ĺ���ָ��
                                  p,                            //��ʼ���Ĵ�����������һ������������
                                  x,                            //����ֵ
                                  m,                            //����ά��
                                  n,                            //����ֵά��
                                  1000,                        //����������
                                  NULL,                        //opts,       //������һЩ����
                                  info,                        //������С�������һЩ����������Ҫ��ΪNULL
                                  NULL, NULL, NULL            //һЩ�ڴ��ָ�룬��ʱ����Ҫ
            );
            /*printf("Levenberg-Marquardt returned in %g iter, reason %g, sumsq %g [%g]\n", info[5], info[6], info[1], info[0]);
            printf("Bestfit parameters: A:%.7g b:%.7g sigma:%.7g A:%.7g b:%.7g sigma:%.7g\n", p[0], p[1], p[2], p[3], p[4], p[5]);
            printf("����ʱ���: %.7g ns\n", abs(p[4] - p[1]));*/

            //���Ż���Ĳ����鸳��vector
            i = 0;
            for (gaussPraIter = waveParam.begin(); gaussPraIter != waveParam.end(); gaussPraIter++) {
                gaussPraIter->A = p[i++];
                gaussPraIter->b = p[i++];
                gaussPraIter->sigma = p[i++];
            }

            break;
        }
        case 4: {
            //��ȡ��˹��������
            double p[12];
            int i = 0;
            for (auto gp : waveParam) {
                p[i++] = gp.A;
                p[i++] = gp.b;
                p[i++] = gp.sigma;
            }
            int m = i;
            int n = srcWave.size();

            //��ȡ�������
            double x[320];
            i = 0;
            for (vector<float>::iterator iter = srcWave.begin(); iter != srcWave.end(); ++iter, ++i) {
                x[i] = *iter;
            }

            double info[LM_INFO_SZ];
            // ���õ�����ں���
            int ret = dlevmar_der(expfun4,    //��������ֵ֮���ϵ�ĺ���ָ��
                                  jacexpfun4,                    //�����ſ˱Ⱦ���ĺ���ָ��
                                  p,                            //��ʼ���Ĵ�����������һ������������
                                  x,                            //����ֵ
                                  m,                            //����ά��
                                  n,                            //����ֵά��
                                  1000,                        //����������
                                  NULL,                        //opts,       //������һЩ����
                                  info,                        //������С�������һЩ����������Ҫ��ΪNULL
                                  NULL, NULL, NULL            //һЩ�ڴ��ָ�룬��ʱ����Ҫ
            );
            /*printf("Levenberg-Marquardt returned in %g iter, reason %g, sumsq %g [%g]\n", info[5], info[6], info[1], info[0]);
            printf("Bestfit parameters: A:%.7g b:%.7g sigma:%.7g A:%.7g b:%.7g sigma:%.7g\n", p[0], p[1], p[2], p[3], p[4], p[5]);
            printf("����ʱ���: %.7g ns\n", abs(p[4] - p[1]));*/

            //���Ż���Ĳ����鸳��vector
            i = 0;
            for (gaussPraIter = waveParam.begin(); gaussPraIter != waveParam.end(); gaussPraIter++) {
                gaussPraIter->A = p[i++];
                gaussPraIter->b = p[i++];
                gaussPraIter->sigma = p[i++];
            }

            break;
        }
        case 5: {
            //��ȡ��˹��������
            double p[15];
            int i = 0;
            for (auto gp : waveParam) {
                p[i++] = gp.A;
                p[i++] = gp.b;
                p[i++] = gp.sigma;
            }
            int m = i;
            int n = srcWave.size();

            //��ȡ�������
            double x[320];
            i = 0;
            for (vector<float>::iterator iter = srcWave.begin(); iter != srcWave.end(); ++iter, ++i) {
                x[i] = *iter;
            }

            double info[LM_INFO_SZ];
            // ���õ�����ں���
            int ret = dlevmar_der(expfun5,    //��������ֵ֮���ϵ�ĺ���ָ��
                                  jacexpfun5,                    //�����ſ˱Ⱦ���ĺ���ָ��
                                  p,                            //��ʼ���Ĵ�����������һ������������
                                  x,                            //����ֵ
                                  m,                            //����ά��
                                  n,                            //����ֵά��
                                  1000,                        //����������
                                  NULL,                        //opts,       //������һЩ����
                                  info,                        //������С�������һЩ����������Ҫ��ΪNULL
                                  NULL, NULL, NULL            //һЩ�ڴ��ָ�룬��ʱ����Ҫ
            );
            /*printf("Levenberg-Marquardt returned in %g iter, reason %g, sumsq %g [%g]\n", info[5], info[6], info[1], info[0]);
            printf("Bestfit parameters: A:%.7g b:%.7g sigma:%.7g A:%.7g b:%.7g sigma:%.7g\n", p[0], p[1], p[2], p[3], p[4], p[5]);
            printf("����ʱ���: %.7g ns\n", abs(p[4] - p[1]));*/

            //���Ż���Ĳ����鸳��vector
            i = 0;
            for (gaussPraIter = waveParam.begin(); gaussPraIter != waveParam.end(); gaussPraIter++) {
                gaussPraIter->A = p[i++];
                gaussPraIter->b = p[i++];
                gaussPraIter->sigma = p[i++];
            }

            break;
        }
        case 6: {
            //��ȡ��˹��������
            double p[18];
            int i = 0;
            for (auto gp : waveParam) {
                p[i++] = gp.A;
                p[i++] = gp.b;
                p[i++] = gp.sigma;
            }
            int m = i;
            int n = srcWave.size();

            //��ȡ�������
            double x[320];
            i = 0;
            for (vector<float>::iterator iter = srcWave.begin(); iter != srcWave.end(); ++iter, ++i) {
                x[i] = *iter;
            }

            double info[LM_INFO_SZ];
            // ���õ�����ں���
            int ret = dlevmar_der(expfun6,    //��������ֵ֮���ϵ�ĺ���ָ��
                                  jacexpfun6,                    //�����ſ˱Ⱦ���ĺ���ָ��
                                  p,                            //��ʼ���Ĵ�����������һ������������
                                  x,                            //����ֵ
                                  m,                            //����ά��
                                  n,                            //����ֵά��
                                  1000,                        //����������
                                  NULL,                        //opts,       //������һЩ����
                                  info,                        //������С�������һЩ����������Ҫ��ΪNULL
                                  NULL, NULL, NULL            //һЩ�ڴ��ָ�룬��ʱ����Ҫ
            );
            /*printf("Levenberg-Marquardt returned in %g iter, reason %g, sumsq %g [%g]\n", info[5], info[6], info[1], info[0]);
            printf("Bestfit parameters: A:%.7g b:%.7g sigma:%.7g A:%.7g b:%.7g sigma:%.7g\n", p[0], p[1], p[2], p[3], p[4], p[5]);
            printf("����ʱ���: %.7g ns\n", abs(p[4] - p[1]));*/

            //���Ż���Ĳ����鸳��vector
            i = 0;
            for (gaussPraIter = waveParam.begin(); gaussPraIter != waveParam.end(); gaussPraIter++) {
                gaussPraIter->A = p[i++];
                gaussPraIter->b = p[i++];
                gaussPraIter->sigma = p[i++];
            }

            break;
        }
        default:
            break;
    }

    return;
}


/*���ܣ�	����ˮ��
//���ݣ�	��ȡ������ĿС��������ֱ���޳�������ȡ��һ�������������ֵ��Ϊˮ��ز�������ʱ�������Ϊˮ�׻ز�������ˮ��
*/
void WaveData::CalcuDepth(vector<GaussParameter> &waveParam, float &BorGDepth) {
    if (waveParam.size() <= 1) {
        BorGDepth = 0;
    } else if ((waveParam.size() > 1) && (waveParam.size() < 5)) {
        gaussPraIter = waveParam.begin();
        float tbegin = gaussPraIter->b;
        float tend = tbegin;

        for (gaussPraIter = waveParam.begin() + 1; gaussPraIter != waveParam.end(); gaussPraIter++) {

            if ((gaussPraIter->b > tend) &&
                (gaussPraIter->wavetype == BOTTOM))//ˮ�׻ز��ض�������ˮ��ز��ĺ���ʱ�̣�Ϊ��ײ������������𣬼ٶ�����ˮ��ز��Ļز�ʱ�������������ڣ�����ˮ�����ɢ�䣩
            {
                tend = gaussPraIter->b;
                break;
            }
        }
        //gaussPraIter = waveParam.end()-1;			//!!!��
        //float tend = gaussPraIter->b;

        BorGDepth = c * (tend - tbegin) * cos(asin(sin(Theta) / nwater)) / (2 * nwater);
    } else if (waveParam.size() >= 5) {
        // У�������������ȷ��ˮ����ˮ�ײ�ʱ��ֲ�����ˮ������������<ˮ��ǰ��Ϊ��Ч����
        gaussPraIter = waveParam.begin();
        float check = gaussPraIter->b;
        // ˮ��λ���ڡ�140��320����Ч
        if (check >= 140) {
            BorGDepth = 0;
            return;
        }
        int before = 0;
        int after = 0;
        for (gaussPraIter = waveParam.begin() + 1; gaussPraIter != waveParam.end(); gaussPraIter++) {
            if (gaussPraIter->b > check) {
                after++;
            } else {
                before++;
            }
        }
        if (after + 2 < before) {
            BorGDepth = 0;
            return;
        }


        gaussPraIter = waveParam.begin();
        float tbegin = gaussPraIter->b;
        gaussPraIter = waveParam.begin() + 1;
        float tend = gaussPraIter->b;

        // ���ι������ܸ��ţ�ȡ�����ܳ��ֵ�n������
        for (gaussPraIter = waveParam.begin() + 1; gaussPraIter != waveParam.end(); gaussPraIter++) {
            if (gaussPraIter->b > tend && gaussPraIter->wavetype == BOTTOM && gaussPraIter->b > tbegin) {
                tend = gaussPraIter->b;
            }
        }
        //gaussPraIter = waveParam.end()-1;			//!!!��
        //float tend = gaussPraIter->b;

        tend > tbegin ? BorGDepth = c * (tend - tbegin) * cos(asin(sin(Theta) / nwater)) / (2 * nwater) : BorGDepth = 0;
    }
}


/**
 * ����ͬ�ϣ���¼ˮ��ˮ�׷�����������
 * @param waveParam ��˹������
 * @param BorGDepth ���ˮ��
 * @param index ����λ��
 */
void WaveData::CalcuDepthOutIndex(vector<GaussParameter> &waveParam, float &BorGDepth, int *index) {
    index[0] = -1;
    index[1] = -1;
    int a = 0, b = 0;
    if (waveParam.size() <= 1) {
        BorGDepth = 0;
    } else if ((waveParam.size() > 1) && (waveParam.size() < 5)) {
        gaussPraIter = waveParam.begin();
        float tbegin = gaussPraIter->b;
        float tend = tbegin;

        for (gaussPraIter = waveParam.begin() + 1; gaussPraIter != waveParam.end(); gaussPraIter++) {
            b++;
            if ((gaussPraIter->b > tend) &&
                (gaussPraIter->wavetype == BOTTOM))//ˮ�׻ز��ض�������ˮ��ز��ĺ���ʱ�̣�Ϊ��ײ������������𣬼ٶ�����ˮ��ز��Ļز�ʱ�������������ڣ�����ˮ�����ɢ�䣩
            {
                tend = gaussPraIter->b;
                break;
            }
        }
        //gaussPraIter = waveParam.end()-1;			//!!!��
        //float tend = gaussPraIter->b;

        BorGDepth = c * (tend - tbegin) * cos(asin(sin(Theta) / nwater)) / (2 * nwater);
        index[0] = a;
        index[1] = b;
    } else if (waveParam.size() >= 5) {
        // У�������������ȷ��ˮ����ˮ�ײ�ʱ��ֲ�����ˮ������������<ˮ��ǰ��Ϊ��Ч����
        gaussPraIter = waveParam.begin();
        float check = gaussPraIter->b;
        // ˮ��λ���ڡ�140��320����Ч
        if (check >= 140) {
            BorGDepth = 0;
            return;
        }
        int before = 0;
        int after = 0;
        for (gaussPraIter = waveParam.begin() + 1; gaussPraIter != waveParam.end(); gaussPraIter++) {
            if (gaussPraIter->b > check) {
                after++;
            } else {
                before++;
            }
        }
        if (after + 2 < before) {
            BorGDepth = 0;
            return;
        }


        gaussPraIter = waveParam.begin();
        float tbegin = gaussPraIter->b;
        gaussPraIter = waveParam.begin() + 1;
        float tend = gaussPraIter->b;

        // ���ι������ܸ��ţ�ȡ�����ܳ��ֵ�n������
        for (gaussPraIter = waveParam.begin() + 1; gaussPraIter != waveParam.end(); gaussPraIter++) {
            b++;
            if (gaussPraIter->b > tend && gaussPraIter->wavetype == BOTTOM && gaussPraIter->b > tbegin) {
                tend = gaussPraIter->b;
            }
        }
        //gaussPraIter = waveParam.end()-1;			//!!!��
        //float tend = gaussPraIter->b;

        tend > tbegin ? BorGDepth = c * (tend - tbegin) * cos(asin(sin(Theta) / nwater)) / (2 * nwater) : BorGDepth = 0;
        index[0] = a;
        index[1] = b;
        return;
    }
}


/*���ܣ�	�Զ�����Ҫ�������Ϣ
//���ݣ�	�� �� �� ʱ �� ��
*/
ostream &operator<<(ostream &stream, const WaveData &wavedata) {
    stream << wavedata.m_time.year << " "
           << wavedata.m_time.month << " "
           << wavedata.m_time.day << " "
           << wavedata.m_time.hour << " "
           << wavedata.m_time.minute << " "
           << wavedata.m_time.second;

    //��Ȥ�����ݶ�Ϊ�ƶ�ͨ���Ĳ����������λ��
    switch (wavedata.ostreamFlag) {
        case BLUE: {
            stream << " " << wavedata.blueDepth << "m";

            if (!wavedata.m_BlueGauPra.empty()) {
                for (auto p : wavedata.m_BlueGauPra) {
                    stream << " " << p.b << (p.wavetype ? "S" : "B");
                }
            }
            break;
        }
        case GREEN: {
            stream << " " << wavedata.greenDepth << "m";
            if (!wavedata.m_GreenGauPra.empty()) {
                for (auto p : wavedata.m_GreenGauPra) {
                    stream << " " << p.b << (p.wavetype ? "S" : "B");
                }
            }
            break;
        }
    }

    stream << endl;
    return stream;
}


/*���ܣ�	��ͨ��˹�ֽ����ˮ��
//���ݣ�	ȡ����������ֱֵ�Ӽ���
*/
void WaveData::CalcuDepthByGauss(vector<GaussParameter> &waveParam, float &BorGDepth) {
    if ((waveParam.size() > 1) && (waveParam.size() < 7)) {
        gaussPraIter = waveParam.begin();
        float tbegin = gaussPraIter->b;
        gaussPraIter = waveParam.end() - 1;
        float tend = gaussPraIter->b;
        BorGDepth = c * abs(tend - tbegin) * cos(asin(sin(Theta) / nwater)) / (2 * nwater);
    } else {
        BorGDepth = 0;
    }
}

void WaveData::CalcuAfter(vector<GaussParameter> &waveParam, vector<float> &after) {

    for (int i = 0; i < 320; ++i) {
        float ans = 0;
        int size = waveParam.size();
        for (int j = 0; j < size; ++j) {
            ans += waveParam.at(j).A * exp(-(i - waveParam.at(j).b) * (i - waveParam.at(j).b) /
                                           (2 * waveParam.at(j).sigma * waveParam.at(j).sigma));
        }

        after.push_back(ans);
    }
}