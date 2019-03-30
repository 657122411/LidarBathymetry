#include "DeepWave.h"
#include <numeric>
#include <algorithm>

#define DeepPulseWidth 4	//���弤����������������ֵ�ο�
#define TimeDifference 8	//��UTC��ʱ��

#define BLUE true
#define GREEN false

#define c 0.3				//��Թ��ٳ�������
#define ndeepwater 1.34		//��ˮˮ�ʵ�������

// �ز�����
#define DEEPSURFACE true	//ˮ��ز������ܰ�������ɢ�䣩
#define DEEPBOTTOM false	//ˮ�׻�ˮ�����ʻز�

bool DeepWave::ostreamFlag = BLUE;


//�������ƽ��
void linearSmooth5(float in[], float out[], int N)
{
	int i;
	if (N < 5)
	{
		for (i = 0; i <= N - 1; i++)
		{
			out[i] = in[i];
		}
	}
	else
	{
		out[0] = (3.0 * in[0] + 2.0 * in[1] + in[2] - in[4]) / 5.0;
		out[1] = (4.0 * in[0] + 3.0 * in[1] + 2 * in[2] + in[3]) / 10.0;
		for (i = 2; i <= N - 3; i++)
		{
			out[i] = (in[i - 2] + in[i - 1] + in[i] + in[i + 1] + in[i + 2]) / 5.0;
		}
		out[N - 2] = (4.0 * in[N - 1] + 3.0 * in[N - 2] + 2 * in[N - 3] + in[N - 4]) / 10.0;
		out[N - 1] = (3.0 * in[N - 1] + 2.0 * in[N - 2] + in[N - 3] - in[N - 5]) / 5.0;
	}
}


//�ߵ�����ƽ��
void linearSmooth7(float in[], float out[], int N)
{
	int i;
	if (N < 7)
	{
		for (i = 0; i <= N - 1; i++)
		{
			out[i] = in[i];
		}
	}
	else
	{
		out[0] = (13.0 * in[0] + 10.0 * in[1] + 7.0 * in[2] + 4.0 * in[3] +
			in[4] - 2.0 * in[5] - 5.0 * in[6]) / 28.0;

		out[1] = (5.0 * in[0] + 4.0 * in[1] + 3 * in[2] + 2 * in[3] +
			in[4] - in[6]) / 14.0;

		out[2] = (7.0 * in[0] + 6.0 * in[1] + 5.0 * in[2] + 4.0 * in[3] +
			3.0 * in[4] + 2.0 * in[5] + in[6]) / 28.0;

		for (i = 3; i <= N - 4; i++)
		{
			out[i] = (in[i - 3] + in[i - 2] + in[i - 1] + in[i] + in[i + 1] + in[i + 2] + in[i + 3]) / 7.0;
		}

		out[N - 3] = (7.0 * in[N - 1] + 6.0 * in[N - 2] + 5.0 * in[N - 3] +
			4.0 * in[N - 4] + 3.0 * in[N - 5] + 2.0 * in[N - 6] + in[N - 7]) / 28.0;

		out[N - 2] = (5.0 * in[N - 1] + 4.0 * in[N - 2] + 3.0 * in[N - 3] +
			2.0 * in[N - 4] + in[N - 5] - in[N - 7]) / 14.0;

		out[N - 1] = (13.0 * in[N - 1] + 10.0 * in[N - 2] + 7.0 * in[N - 3] +
			4 * in[N - 4] + in[N - 5] - 2 * in[N - 6] - 5 * in[N - 7]) / 28.0;
	}
}


//�����κ������ƽ��
void quadraticSmooth5(float in[], float out[], int N)
{
	int i;
	if (N < 5)
	{
		for (i = 0; i <= N - 1; i++)
		{
			out[i] = in[i];
		}
	}
	else
	{
		out[0] = (31.0 * in[0] + 9.0 * in[1] - 3.0 * in[2] - 5.0 * in[3] + 3.0 * in[4]) / 35.0;
		out[1] = (9.0 * in[0] + 13.0 * in[1] + 12 * in[2] + 6.0 * in[3] - 5.0 *in[4]) / 35.0;
		for (i = 2; i <= N - 3; i++)
		{
			out[i] = (-3.0 * (in[i - 2] + in[i + 2]) +
				12.0 * (in[i - 1] + in[i + 1]) + 17 * in[i]) / 35.0;
		}
		out[N - 2] = (9.0 * in[N - 1] + 13.0 * in[N - 2] + 12.0 * in[N - 3] + 6.0 * in[N - 4] - 5.0 * in[N - 5]) / 35.0;
		out[N - 1] = (31.0 * in[N - 1] + 9.0 * in[N - 2] - 3.0 * in[N - 3] - 5.0 * in[N - 4] + 3.0 * in[N - 5]) / 35.0;
	}
}


//�ߵ���κ������ƽ��
void quadraticSmooth7(float in[], float out[], int N)
{
	int i;
	if (N < 7)
	{
		for (i = 0; i <= N - 1; i++)
		{
			out[i] = in[i];
		}
	}
	else
	{
		out[0] = (32.0 * in[0] + 15.0 * in[1] + 3.0 * in[2] - 4.0 * in[3] -
			6.0 * in[4] - 3.0 * in[5] + 5.0 * in[6]) / 42.0;

		out[1] = (5.0 * in[0] + 4.0 * in[1] + 3.0 * in[2] + 2.0 * in[3] +
			in[4] - in[6]) / 14.0;

		out[2] = (1.0 * in[0] + 3.0 * in[1] + 4.0 * in[2] + 4.0 * in[3] +
			3.0 * in[4] + 1.0 * in[5] - 2.0 * in[6]) / 14.0;
		for (i = 3; i <= N - 4; i++)
		{
			out[i] = (-2.0 * (in[i - 3] + in[i + 3]) +
				3.0 * (in[i - 2] + in[i + 2]) +
				6.0 * (in[i - 1] + in[i + 1]) + 7.0 * in[i]) / 21.0;
		}
		out[N - 3] = (1.0 * in[N - 1] + 3.0 * in[N - 2] + 4.0 * in[N - 3] +
			4.0 * in[N - 4] + 3.0 * in[N - 5] + 1.0 * in[N - 6] - 2.0 * in[N - 7]) / 14.0;

		out[N - 2] = (5.0 * in[N - 1] + 4.0 * in[N - 2] + 3.0 * in[N - 3] +
			2.0 * in[N - 4] + in[N - 5] - in[N - 7]) / 14.0;

		out[N - 1] = (32.0 * in[N - 1] + 15.0 * in[N - 2] + 3.0 * in[N - 3] -
			4.0 * in[N - 4] - 6.0 * in[N - 5] - 3.0 * in[N - 6] + 5.0 * in[N - 7]) / 42.0;
	}
}


/*
* �������ƽ��
*/
void cubicSmooth5(float in[], float out[], int N)
{

	int i;
	if (N < 5)
	{
		for (i = 0; i <= N - 1; i++)
			out[i] = in[i];
	}

	else
	{
		out[0] = (69.0 * in[0] + 4.0 * in[1] - 6.0 * in[2] + 4.0 * in[3] - in[4]) / 70.0;
		out[1] = (2.0 * in[0] + 27.0 * in[1] + 12.0 * in[2] - 8.0 * in[3] + 2.0 * in[4]) / 35.0;
		for (i = 2; i <= N - 3; i++)
		{
			out[i] = (-3.0 * (in[i - 2] + in[i + 2]) + 12.0 * (in[i - 1] + in[i + 1]) + 17.0 * in[i]) / 35.0;
		}
		out[N - 2] = (2.0 * in[N - 5] - 8.0 * in[N - 4] + 12.0 * in[N - 3] + 27.0 * in[N - 2] + 2.0 * in[N - 1]) / 35.0;
		out[N - 1] = (-in[N - 5] + 4.0 * in[N - 4] - 6.0 * in[N - 3] + 4.0 * in[N - 2] + 69.0 * in[N - 1]) / 70.0;
	}
	return;
}


/*
* �ߵ�����ƽ��
*/
void cubicSmooth7(float in[], float out[], int N)
{
	int i;
	if (N < 7)
	{
		for (i = 0; i <= N - 1; i++)
		{
			out[i] = in[i];
		}
	}
	else
	{
		out[0] = (39.0 * in[0] + 8.0 * in[1] - 4.0 * in[2] - 4.0 * in[3] +
			1.0 * in[4] + 4.0 * in[5] - 2.0 * in[6]) / 42.0;
		out[1] = (8.0 * in[0] + 19.0 * in[1] + 16.0 * in[2] + 6.0 * in[3] -
			4.0 * in[4] - 7.0* in[5] + 4.0 * in[6]) / 42.0;
		out[2] = (-4.0 * in[0] + 16.0 * in[1] + 19.0 * in[2] + 12.0 * in[3] +
			2.0 * in[4] - 4.0 * in[5] + 1.0 * in[6]) / 42.0;
		for (i = 3; i <= N - 4; i++)
		{
			out[i] = (-2.0 * (in[i - 3] + in[i + 3]) +
				3.0 * (in[i - 2] + in[i + 2]) +
				6.0 * (in[i - 1] + in[i + 1]) + 7.0 * in[i]) / 21.0;
		}
		out[N - 3] = (-4.0 * in[N - 1] + 16.0 * in[N - 2] + 19.0 * in[N - 3] +
			12.0 * in[N - 4] + 2.0 * in[N - 5] - 4.0 * in[N - 6] + 1.0 * in[N - 7]) / 42.0;
		out[N - 2] = (8.0 * in[N - 1] + 19.0 * in[N - 2] + 16.0 * in[N - 3] +
			6.0 * in[N - 4] - 4.0 * in[N - 5] - 7.0 * in[N - 6] + 4.0 * in[N - 7]) / 42.0;
		out[N - 1] = (39.0 * in[N - 1] + 8.0 * in[N - 2] - 4.0 * in[N - 3] -
			4.0 * in[N - 4] + 1.0 * in[N - 5] + 4.0 * in[N - 6] - 2.0 * in[N - 7]) / 42.0;
	}
}


/*���ܣ�	�������ݵı�׼��
//*:
//resultSet���������������
//stdev��	����ֵΪ��׼��
//*
*/
float calculateDeepSigma(vector<float> resultSet)
{
	double sum = std::accumulate(std::begin(resultSet), std::end(resultSet), 0.0);
	double mean = sum / resultSet.size(); //��ֵ  

	double accum = 0.0;
    for(vector<float>::iterator iter=resultSet.begin();iter!=resultSet.end();iter++)
	{
		accum += (*iter - mean)*(*iter - mean);
	};

    float stdev = sqrt(accum / (resultSet.size() - 1)); //����

    return  stdev;

}


//��ֵ����㷨 https://www.mathworks.com/help/signal/ref/findpeaks.html
vector<int> FindLocalMaxima(vector<float> dataVector, int minProminence, int maxProminence, int minWidth, int maxWidth)
{
	int dataVectorSize = dataVector.size();
	vector<int>localMaximaIndex, localMaximaValue, prominenceValue, inRangeLocalMaxima;
	int j;
	int currentValue, nextValue, prevValue;
	int totalLocalMaxima;
	int prevLocalMaximaIndex;
	int nextLocalMaximaIndex;
	int prevHigherLocalMaximaValue;
	int nextHigherLocalMaximaValue;
	int leftWidth, rightWidth, width;
	int prevMinimum, nextMinimum, minimumValue;
	int prominence;
	int i = 1;
	/********************* FIND LOCAL MAXIMA ***************************/
	while (i<dataVectorSize-1)
	{
		j = 1;
		currentValue = dataVector[i];
		prevValue = dataVector[i - 1];
		nextValue = dataVector[i + j];

		if ((prevValue<currentValue) && (currentValue >= nextValue))
		{
			while ((currentValue == nextValue) && ((i + j)<dataVectorSize-1))
			{
				j++;
				nextValue = dataVector[i + j];
			}
			if (currentValue>nextValue)
			{
				localMaximaIndex.push_back(i);
				localMaximaValue.push_back(currentValue);
			}
		}
		i += j;
	}

	totalLocalMaxima = localMaximaIndex.size();
	prominenceValue.erase(prominenceValue.begin(), prominenceValue.end());

	/********************* GET PROMINENCE ***************************/
	for (i = 0; i<totalLocalMaxima; i++)
	{
		prevLocalMaximaIndex = i - 1;
		nextLocalMaximaIndex = i + 1;
		prevHigherLocalMaximaValue = 0;
		nextHigherLocalMaximaValue = 0;

		if (prevLocalMaximaIndex >= 0)
		{
			while ((prevLocalMaximaIndex >= 1) && (localMaximaValue[prevLocalMaximaIndex] <= localMaximaValue[i]))
			{
				prevLocalMaximaIndex--;
			}
			if (localMaximaValue[prevLocalMaximaIndex]>localMaximaValue[i]) {
				prevHigherLocalMaximaValue = localMaximaValue[prevLocalMaximaIndex];
			}
			else {
				prevHigherLocalMaximaValue = 0;
			}
		}
		if (nextLocalMaximaIndex<totalLocalMaxima)
		{
			while ((nextLocalMaximaIndex<totalLocalMaxima - 1) && (localMaximaValue[nextLocalMaximaIndex] <= localMaximaValue[i]))//
			{
				nextLocalMaximaIndex++;
			}
			if (localMaximaValue[nextLocalMaximaIndex]>localMaximaValue[i]) {
				nextHigherLocalMaximaValue = localMaximaValue[nextLocalMaximaIndex];
			}
			else {
				nextHigherLocalMaximaValue = 0;
			}
		}
		prevMinimum = dataVector[localMaximaIndex[i]];
		nextMinimum = dataVector[localMaximaIndex[i]];
		minimumValue = dataVector[localMaximaIndex[i]];
		if ((prevHigherLocalMaximaValue == 0) && (nextHigherLocalMaximaValue == 0))
		{
			for (j = 0; j<dataVectorSize; j++)
			{
				if (dataVector[j]<minimumValue)
					minimumValue = dataVector[j];
			}


		}
		else
		{
			if (prevLocalMaximaIndex<0)
			{
				for (j = localMaximaIndex[i]; j >= 0; j--)
				{
					if (dataVector[j]<prevMinimum)
						prevMinimum = dataVector[j];
				}

			}
			else
			{
				for (j = localMaximaIndex[i]; j>localMaximaValue[prevLocalMaximaIndex]; j--)
				{
					if (dataVector[j]<prevMinimum)
						prevMinimum = dataVector[j];
				}

			}
			if (nextLocalMaximaIndex >= totalLocalMaxima - 1)
			{
				for (j = localMaximaIndex[i]; j<dataVectorSize; j++)
				{
					if (dataVector[j]<nextMinimum)
						nextMinimum = dataVector[j];
				}

			}
			else
			{
				for (j = localMaximaIndex[i]; j<localMaximaIndex[nextLocalMaximaIndex]; j++)
				{
					if (dataVector[j]<nextMinimum)
						nextMinimum = dataVector[j];
				}

			}
			minimumValue = prevMinimum>nextMinimum ? prevMinimum : nextMinimum;
		}
		prominence = localMaximaValue[i] - minimumValue;
		/*************************** GET WIDTH ********************************/
		j = 1;
		while (((localMaximaIndex[i] - j)>0) && (dataVector[localMaximaIndex[i] - j] - minimumValue>(prominence) / 2)) { j++; }
		leftWidth = j;
		j = 1;
		while (((localMaximaIndex[i] + j)<dataVectorSize) && (dataVector[localMaximaIndex[i] + j] - minimumValue>(prominence) / 2)) { j++; }
		rightWidth = j;
		int width = leftWidth + rightWidth;

		/********************* IS THE PEAK IN RANGE? ***************************/
		if ((localMaximaValue[i] - minimumValue >= minProminence) && (localMaximaValue[i] - minimumValue <= maxProminence) && (width >= minWidth) && (width <= maxWidth))
		{
			inRangeLocalMaxima.push_back(localMaximaIndex[i]);
		}

	}

	return inRangeLocalMaxima;
}


/*���ܣ�	���������˹����ģ��
//*p:	�������
//*x��  ԭʼ���ݣ�����ֵ��
//m��	����ά��
//n��	����ֵά��
//*data:��
*/
void deep_expfun2(double *p, double *x, int m, int n, void *data)
{
	register int i;
	for (i = 0; i<n; ++i)
	{
		//д��������x[i]֮��Ĺ�ϵʽ���������﷽�̵��ұ�û�й۲�ֵ������ֻ�в���
		x[i] = p[0] * exp(-(i - p[1])*(i - p[1]) / (2 * p[2])*(2 * p[2]))
			+ p[3] * exp(-(i - p[4])*(i - p[4]) / (2 * p[5])*(2 * p[5]));
	}
}


/*���ܣ�	�����˹����ģ�͵��ſɱȾ���
//*p:	�������
//jac�� �ſɱȾ������
//m��	����ά��
//n��	����ֵά��
//*data:��
*/
void deep_jacexpfun2(double *p, double *jac, int m, int n, void *data)
{
	register int i, j;
	//д���ſ˱Ⱦ���
	for (i = j = 0; i<n; ++i)
	{
		jac[j++] = exp(-(i - p[1])*(i - p[1]) / (2 * p[2])*p[2]);
		jac[j++] = p[0] * (i - p[1]) / (p[2] * p[2])*exp(-(i - p[1])*(i - p[1]) / (2 * p[2] * p[2]));
		jac[j++] = p[0] * (i - p[1])*(i - p[1]) / (p[2] * p[2] * p[2])*exp(-(i - p[1])*(i - p[1]) / (2 * p[2] * p[2]));

		jac[j++] = exp(-(i - p[4])*(i - p[4]) / (2 * p[5])*p[5]);
		jac[j++] = p[3] * (i - p[4]) / (p[5] * p[5])*exp(-(i - p[4])*(i - p[4]) / (2 * p[5] * p[5]));
		jac[j++] = p[3] * (i - p[4])*(i - p[4]) / (p[5] * p[5] * p[5])*exp(-(i - p[4])*(i - p[4]) / (2 * p[5] * p[5]));
	}
}


/*
//���ܣ����캯����ʼ������
*/
DeepWave::DeepWave()
{
	m_time = { 0,0,0,0,0,0 };
	m_BlueDeepNoise = 0;
	m_GreenDeepNoise = 0;
	blueDeepDepth = 0;
	greenDeepDepth = 0;
	redTime = 0;
}


DeepWave::~DeepWave()
{
}


/*���ܣ�		��ȡԭʼ��������ǳˮͨ���Ķ��λز�����
//&hs:		ͨ��ԭʼ����
*/
void DeepWave::GetDeepData(HS_Lidar & hs)
{
	//GPS->UTC->BeiJing
	PGPSTIME pgt = new GPSTIME;
	PCOMMONTIME pct = new COMMONTIME;
	pgt->wn = (int)hs.header.nGPSWeek;
	pgt->tow.sn = (long)hs.header.dGPSSecond;
	pgt->tow.tos = 0;
	GPSTimeToCommonTime(pgt, pct);
	m_time.year = pct->year;
	m_time.month = pct->month;
	m_time.day = pct->day;
	m_time.hour = pct->hour + TimeDifference;	//ֱ��ת��Ϊ����ʱ��
	m_time.minute = pct->minute;
	m_time.second = pct->second;
	delete pgt;
	delete pct;

	//ȡ�����⡢����ͨ����ˮ����
	vector<int >::iterator it;//����������
	for (it = hs.deepData1.begin(); it != hs.deepData1.end(); ++it)
	{
		m_RedDeep.push_back((float)*it);
	}
	for (it = hs.deepData2.begin(); it != hs.deepData2.end(); ++it) 
	{
		m_BlueDeep.push_back((float)*it);
	}
	for (it = hs.deepData3.begin(); it != hs.deepData3.end(); ++it) 
	{
		m_GreenDeep.push_back((float)*it);
	}
}


/*���ܣ�		Ԥ�������ݣ���ȡ��Ч���ֲ�����ȥ���˲�����
//			��ˮ����SG�˲��� https://blog.csdn.net/liyuanbhu/article/details/11119081
//&srcWave:	ͨ��ԭʼ����
//&noise��	��¼��������������
*/
void DeepWave::DeepFilter(vector<float> &srcWave, float &noise)
{
	//��Ч���ݵĽ�ȡ���¼��뼫Сֵ������ݱ�׼��ı仯
	//begin|______k__KK__|end
	//begin|_____L___KK__|end
	int k = 60, kk = 50, l = 60;//��Ȥ���������˵�

	//����ǰ����ȡ��
	for (int i = 60; i < srcWave.size() - 60; i++)
	{
		if (srcWave.at(srcWave.size() - (i - 1)) > srcWave.at(srcWave.size() - i) && srcWave.at(srcWave.size() - i) < srcWave.at(srcWave.size() - (i + 1)))
		{
			k = i;
			for (int j = i + 6/*����������*/; j < srcWave.size() - 66; j++)
			{
				if (srcWave.at(srcWave.size() - (j - 1)) < srcWave.at(srcWave.size() - j) && srcWave.at(srcWave.size() - j) > srcWave.at(srcWave.size() - (j + 1)))
					l = j;
				break;
			}
			if (l > k)
			{
				vector<float> v1(srcWave.end() - kk, srcWave.end());
				vector<float> v2(srcWave.end() - l, srcWave.end()-kk);
				kk = k; 
				float Sv1 = calculateDeepSigma(v1);
				float Sv2 = calculateDeepSigma(v2);
				if (Sv2 > 2 * Sv1)//��ֵ����
				{
					break;
				}
					
			}
		}
	}

	//���߳������޳�
	srcWave.erase(srcWave.end() - kk, srcWave.end());


	//��˹�˲�ȥ��
	vector<float> dstWave;
	dstWave.assign(srcWave.begin(), srcWave.end());
	
	//���������ڴ�
	float *buffer = new float[dstWave.size()];
	if (!dstWave.empty()) 
	{ 
		memcpy(buffer, &dstWave[0], dstWave.size() * sizeof(float));
	}
	float *buffer1 = new float[dstWave.size()];
	float *buffer2 = new float[dstWave.size()];
	float *buffer3 = new float[dstWave.size()];

	//�˲�����
	linearSmooth5(buffer, buffer1, dstWave.size());
	quadraticSmooth5(buffer1, buffer2, dstWave.size());
	cubicSmooth5(buffer2, buffer3, dstWave.size());

	dstWave.assign(&buffer3[0], &buffer3[dstWave.size()]);


	//�ͷ��ڴ�
	delete buffer; 
	buffer = nullptr;
	delete buffer1; 
	buffer1 = nullptr;
	delete buffer2; 
	buffer2 = nullptr;
	delete buffer3; 
	buffer3 = nullptr;

	noise = 0;
	//�����������:�����˲�ǰ��Ĳ������ݵķ�ֵ��ľ������׼�
	for (int i = 0; i < srcWave.size(); i++)
	{
		noise += (srcWave.at(i) - dstWave.at(i)) * (srcWave.at(i) - dstWave.at(i));
	}
	noise = sqrt(noise / srcWave.size());

	srcWave.assign(dstWave.begin(), dstWave.end());
}


/*���ܣ�			��ֵ�����㺯��
//&srcWave:		ͨ��ԭʼ����
//&waveParam��	��ͨ���ķ�ֵ����
//&noise��		��ͨ��������
*/
void DeepWave::DeepResolve(vector<float> &srcWave, vector<float> &waveParam, float &noise)
{
	//����ԭʼ����
	vector<float> data,temp;
	data.assign(srcWave.begin(), srcWave.end());

	//���˲����������Сֵ��Ϊ��������
	vector<float>::iterator smallest = min_element(begin(data), end(data));
	float backgroundNoise = *smallest;

	//�������ݳ�ȥ��������
	for (vector<float>::iterator m = data.begin(); m != data.end(); m++) //�õ������ķ�ʽ
	{
		*m -= backgroundNoise;
	}

	//Ѱ�ҷ�ֵ
	vector<int>answer = FindLocalMaxima(data, 3, 800, 1, 20);//��ֵ��������������

	//�����ֵ������
	for (auto ans : answer)
	{
		waveParam.push_back((float)ans);
	}
}


/*���ܣ�			LM�㷨�����Ż�
//&srcWave:		ͨ��ԭʼ����
//&waveParam��	��ͨ���ĸ�˹��������
//LM�㷨�ο���	https://blog.csdn.net/shajun0153/article/details/75073137
*/
void DeepWave::DeepOptimize(vector<float> &srcWave, vector<float> &waveParam)
{

	return;
}


/*���ܣ�	��ȡ������ͨ��ˮ���ʱ��
//���ݣ�	ֱ�ӶԽ�����ͨ�����з�ֵ��⣬ȡ��ֵ�����ǰ��ʱ��
*/
void DeepWave::GetRedTime(vector<float>& srcWave, int & redtime)
{
	//Ѱ�ҷ�ֵ
	vector<int>answer = FindLocalMaxima(srcWave, 3, 800, 1, 20);//��ֵ��������������

	redtime = *min_element(answer.begin(), answer.end());
}


/*���ܣ�	���ݽ�����ͨ��ˮ�������ͨ��ˮ�׼���ˮ��
//���ݣ�	ֱ��ȡ������Ϊˮ�棬����ͨ��ȡ��������������Ϊ˯�ߣ�������Ϊˮ��
*/
void DeepWave::CalcuDeepDepthByRed(vector<float>& waveParam, int & redtime, float & BorGDepth)
{
	if ((waveParam.size() <= 1) || (waveParam.size() >= 5))
	{
		BorGDepth = 0;
	}
	else
	{
		float tbegin = (redtime<*min_element(waveParam.begin(), waveParam.end()))? redtime: *min_element(waveParam.begin(), waveParam.end());
		float tend = *max_element(waveParam.begin(), waveParam.end());

		BorGDepth = c*(tend - tbegin) / (2 * ndeepwater);
	}
}


/*���ܣ�	����ˮ��
//���ݣ�	��ȡ������ĿС��������ֱ���޳�������ȡ��һ�������������ֵ��Ϊˮ��ز�������ʱ�������Ϊˮ�׻ز�������ˮ��
*/
void DeepWave::CalcuDeepDepth(vector<float>& waveParam, float &BorGDepth)
{
	if ((waveParam.size() <= 1) || (waveParam.size() >= 5))
	{
		BorGDepth = 0;
	}
	else
	{
		float tbegin = *min_element(waveParam.begin(),waveParam.end());
		float tend = *max_element(waveParam.begin(), waveParam.end());

		BorGDepth = c*(tend - tbegin) / (2 * ndeepwater);
	}
}


/*���ܣ�	�Զ�����Ҫ�������Ϣ
//���ݣ�	�� �� �� ʱ �� ��
*/
ostream &operator<<(ostream & stream, const DeepWave & wavedata)
{
	stream << wavedata.m_time.year << " "
		<< wavedata.m_time.month << " "
		<< wavedata.m_time.day << " "
		<< wavedata.m_time.hour << " "
		<< wavedata.m_time.minute << " "
		<< wavedata.m_time.second;


	//��Ȥ�����ݶ�Ϊ�ƶ�ͨ���Ĳ����������λ��
	switch (wavedata.ostreamFlag)
	{
	case BLUE: {
		stream << " " << wavedata.blueDeepDepth << "m";

		//������ͨ������
		if (wavedata.redTime != 0)
		{
			stream << " " << wavedata.redTime<<" "<<"|";
		}

		if (!wavedata.m_BlueDeepPra.empty())
		{
			for (auto p : wavedata.m_BlueDeepPra)
			{
				stream << " " << p;
			}
		}
		break;
	}
	case GREEN: {
		stream << " " << wavedata.greenDeepDepth << "m";

		//������ͨ������
		if (wavedata.redTime != 0)
		{
			stream << " " << wavedata.redTime <<" "<< "|";
		}

		if (!wavedata.m_GreenDeepPra.empty())
		{
			for (auto p : wavedata.m_GreenDeepPra)
			{
				stream << " " << p;
			}
		}
		break;
	}
	}

	stream << endl;
	return stream;
}