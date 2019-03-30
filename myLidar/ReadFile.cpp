#include "ReadFile.h"

#define BLUE true
#define GREEN false


//�ж�֡ͷ�Ƿ���ȷ
bool isHeaderRight(uint8_t header[8])
{
	uint8_t headerSign[] = { 1, 35, 69, 103, 137, 171, 205, 239 };
	bool returnVal = true;
	for (size_t i = 0; i < 8; i++)
	{
		if (header[i] != headerSign[i])
		{
			returnVal = false;
			break;
		}
	}
	return returnVal;
}


ReadFile::ReadFile()
{
}


ReadFile::~ReadFile()
{
}


/*���ܣ�	���ö�ȡ�ļ���ָ��
//in:	��ȡ�ļ��ľ���·��
//out:	����·�������ļ�ָ��
*/
bool ReadFile::setFilename(char filename[100])
{
	m_filename = filename;
	m_filePtr = fopen(m_filename, "rb");
	if (m_filePtr == NULL)
	{
	    printf("file loading failed!\n");
		return false;
	}
	else
	{
        printf("file loading successed!\n");
		return true;
	}
}


/*���ܣ�	����ȫ������ɫͨ��
//out:	��ȡͨ�������˲�ȥ��ֽ��Ż����
*/
void ReadFile::readBlueAll()
{
	unsigned long long j = 0;
	HS_Lidar hs;
	
	//���ļ���λ��ָ���Ƶ��ļ�β��ȡ�ļ�����
	unsigned long long length;
	fseeko(m_filePtr, 0L, SEEK_END);
	length = ftello(m_filePtr);

    printf("BLueChannelProcessing:");
	
	//���ȶ����� output_stream  ios::out ʾ���,ios::app��ʾ������ļ�β��
	fstream output_stream;
	output_stream.open("BlueOut.txt", ios::out);

	//���������flag
	WaveData::ostreamFlag = BLUE;

	//�����ļ���ȡ����
	do {
		fseeko(m_filePtr, j * 8, SEEK_SET);

		//Ѱ��֡ͷ
		uint8_t header[8];
		memset(header, 0, sizeof(uint8_t) * 8);
		fread(header, sizeof(uint8_t), 8, m_filePtr);
		if (isHeaderRight(header))
		{
			//�������ݵ����̣�
			fseeko(m_filePtr, -8, SEEK_CUR);
			hs.initData(m_filePtr);

			WaveData mywave;
			mywave.GetData(hs);
			mywave.Filter(mywave.m_BlueWave, mywave.m_BlueNoise);
			mywave.Resolve(mywave.m_BlueWave, mywave.m_BlueGauPra,mywave.m_BlueNoise);
			mywave.Optimize(mywave.m_BlueWave, mywave.m_BlueGauPra);
			
			mywave.CalcuDepth(mywave.m_BlueGauPra,mywave.blueDepth);

			//�����Ϣ���ļ�
			output_stream << mywave;

			//�ļ�ָ��ƫ��һ֡�������ݵ��ֽ�����2688/8
			j += 336;

			//��ӡ�������������ÿ���������
			printf("%5.2f%%", (float)j / (length / 800));
			printf("\b\b\b\b\b\b");

		}
		else
		{	
			//���ܻ������λز����ݣ�uint16_t[CH.nL1] -> 2*n
			j+=2;
		}
		
	} while (!feof(m_filePtr));

	//�ļ������˳�
	if (feof(m_filePtr) == 1)
	{
		output_stream.close();
        printf("finished!\n");
	}
}


/*���ܣ�	����ȫ������ɫͨ��
//out:	��ȡͨ�������˲�ȥ��ֽ��Ż����
*/
void ReadFile::readGreenAll()
{
	unsigned long long j = 0;
	HS_Lidar hs;

	//���ļ���λ��ָ���Ƶ��ļ�β��ȡ�ļ�����
	unsigned long long length;
	fseeko(m_filePtr, 0L, SEEK_END);
	length = ftello(m_filePtr);
    printf("GreenChannelProcessing:");

	//���ȶ����� output_stream  ios::out ʾ���,ios::app��ʾ������ļ�β��
	fstream output_stream;
	output_stream.open("GreenOut.txt", ios::out);

	//���������flag
	WaveData::ostreamFlag = GREEN;

	//�����ļ���ȡ����
	do {
		fseeko(m_filePtr, j * 8, SEEK_SET);

		//Ѱ��֡ͷ
		uint8_t header[8];
		memset(header, 0, sizeof(uint8_t) * 8);
		fread(header, sizeof(uint8_t), 8, m_filePtr);
		if (isHeaderRight(header))
		{
			//�������ݵ����̣�
			fseeko(m_filePtr, -8, SEEK_CUR);
			hs.initData(m_filePtr);

			WaveData mywave;
			mywave.GetData(hs);
			mywave.Filter(mywave.m_GreenWave, mywave.m_GreenNoise);
			mywave.Resolve(mywave.m_GreenWave, mywave.m_GreenGauPra, mywave.m_GreenNoise);
			mywave.Optimize(mywave.m_GreenWave, mywave.m_GreenGauPra);

			mywave.CalcuDepth(mywave.m_GreenGauPra, mywave.greenDepth);

			//�����Ϣ���ļ�
			output_stream << mywave;

			//�ļ�ָ��ƫ��һ֡�������ݵ��ֽ�����2688/8
			j += 336;

			//��ӡ�������������ÿ���������
			printf("%5.2f%%", (float)j / (length / 800));
			printf("\b\b\b\b\b\b");

		}
		else
		{
			//���ܻ������λز����ݣ�uint16_t[CH.nL1] -> 2*n
			j += 2;
		}

	} while (!feof(m_filePtr));

	//�ļ������˳�
	if (feof(m_filePtr) == 1)
	{
		output_stream.close();
        printf("finished!\n");
	}
}

/*���ܣ�	���ͨ������ѡ����Чͨ��
//out:	��ȡͨ�����ݣ�������ͨ���ı�׼���С����ѡ����Ӧ��ͨ��������ˮ�����
*/
void ReadFile::readMix()
{
	unsigned long long j = 0;
	HS_Lidar hs;

	//���ļ���λ��ָ���Ƶ��ļ�β��ȡ�ļ�����
	unsigned long long length;
	fseeko(m_filePtr, 0L, SEEK_END);
	length = ftello(m_filePtr);
    printf("MixChannelProcessing:");

	//���ȶ����� output_stream  ios::out ʾ���,ios::app��ʾ������ļ�β��
	fstream output_stream;
	output_stream.open("MixOut.txt", ios::out);

	int bgflag;
	float blueStd, greenStd;

	//�����ļ���ȡ����
	do {
		fseeko(m_filePtr, j * 8, SEEK_SET);

		//Ѱ��֡ͷ
		uint8_t header[8];
		memset(header, 0, sizeof(uint8_t) * 8);
		fread(header, sizeof(uint8_t), 8, m_filePtr);
		if (isHeaderRight(header))
		{
			//�������ݵ����̣�
			fseeko(m_filePtr, -8, SEEK_CUR);
			hs.initData(m_filePtr);

			WaveData mywave;
			mywave.GetData(hs);

			blueStd = calculateSigma(mywave.m_BlueWave);
			greenStd = calculateSigma(mywave.m_GreenWave);

			blueStd >= 1.2*greenStd ?  bgflag = BLUE : bgflag = GREEN;//�ж���ֵ

			switch (bgflag)
			{
			case BLUE:
				WaveData::ostreamFlag = BLUE;

				mywave.Filter(mywave.m_BlueWave, mywave.m_BlueNoise);
				mywave.Resolve(mywave.m_BlueWave, mywave.m_BlueGauPra, mywave.m_BlueNoise);
				mywave.Optimize(mywave.m_BlueWave, mywave.m_BlueGauPra);

				mywave.CalcuDepth(mywave.m_BlueGauPra, mywave.blueDepth);
				break;
			case GREEN:
				WaveData::ostreamFlag = GREEN;

				mywave.Filter(mywave.m_GreenWave, mywave.m_GreenNoise);
				mywave.Resolve(mywave.m_GreenWave, mywave.m_GreenGauPra, mywave.m_GreenNoise);
				mywave.Optimize(mywave.m_GreenWave, mywave.m_GreenGauPra);

				mywave.CalcuDepth(mywave.m_GreenGauPra, mywave.greenDepth);
				break;
			default:
				break;
			}
			
			//�����Ϣ���ļ�
			output_stream << mywave;

			//�ļ�ָ��ƫ��һ֡�������ݵ��ֽ�����2688/8
			j += 336;

			//��ӡ�������������ÿ���������
			printf("%5.2f%%", (float)j / (length / 800));
			printf("\b\b\b\b\b\b");

		}
		else
		{
			//���ܻ������λز����ݣ�uint16_t[CH.nL1] -> 2*n
			j += 2;
		}

	} while (!feof(m_filePtr));

	//�ļ������˳�
	if (feof(m_filePtr) == 1)
	{
		output_stream.close();
        printf("finished!\n");
	}
}


/*���ܣ�	���ԭʼ���ݣ��˲������ݣ���ʼ�����ݣ�����������
//out:	4files
*/
void ReadFile::outputData()
{
	unsigned long long j = 0;
	HS_Lidar hs;

	//���ļ���λ��ָ���Ƶ��ļ�β��ȡ�ļ�����
	unsigned long long length;
	fseeko(m_filePtr, 0L, SEEK_END);
	length = ftello(m_filePtr);
    printf("MixChannelProcessing:");

	//���ȶ����� output_stream  ios::out ʾ���,ios::app��ʾ������ļ�β��
	fstream output_stream;
	output_stream.open("MixOut.txt", ios::out);

	fstream origin;//��ʼ����
	fstream filter;//�˲�����
	fstream resolve;//����������
	fstream iterate;//��������
	origin.open("origin.txt", ios::out);
	filter.open("filter.txt", ios::out);
	resolve.open("resolve.txt", ios::out);
	iterate.open("iterate.txt", ios::out);

	int bgflag;
	float blueStd, greenStd;

	//�����ļ���ȡ����
	do {
		fseeko(m_filePtr, j * 8, SEEK_SET);

		//Ѱ��֡ͷ
		uint8_t header[8];
		memset(header, 0, sizeof(uint8_t) * 8);
		fread(header, sizeof(uint8_t), 8, m_filePtr);
		if (isHeaderRight(header))
		{
			//�������ݵ����̣�
			fseeko(m_filePtr, -8, SEEK_CUR);
			hs.initData(m_filePtr);

			WaveData mywave;
			mywave.GetData(hs);

			blueStd = calculateSigma(mywave.m_BlueWave);
			greenStd = calculateSigma(mywave.m_GreenWave);

			blueStd >= 1.2*greenStd ? bgflag = BLUE : bgflag = GREEN;//�ж���ֵ

			switch (bgflag)
			{
			case BLUE:
				WaveData::ostreamFlag = BLUE;

				//���ԭʼ����
				for (auto data : mywave.m_BlueWave)
				{
					origin << data << " ";
				}
				origin << endl;
	
				mywave.Filter(mywave.m_BlueWave, mywave.m_BlueNoise);

				//����˲�����
				for (auto data : mywave.m_BlueWave)
				{
					filter << data << " ";
				}
				filter << endl;

				mywave.Resolve(mywave.m_BlueWave, mywave.m_BlueGauPra, mywave.m_BlueNoise);

				//�����������
				for (auto data : mywave.m_BlueGauPra)
				{
					resolve << data.A << " "<<data.b<<" "<<data.sigma<<" ";
				}
				resolve << endl;

				mywave.Optimize(mywave.m_BlueWave, mywave.m_BlueGauPra);

				//�����������
				for (auto data : mywave.m_BlueGauPra)
				{
					iterate << data.A << " " << data.b << " " << data.sigma << " ";
				}
				iterate << endl;

				mywave.CalcuDepth(mywave.m_BlueGauPra, mywave.blueDepth);
				break;
			case GREEN:
				WaveData::ostreamFlag = GREEN;

				//����˲�����
				for (auto data : mywave.m_GreenWave)
				{
					filter << data << " ";
				}
				filter << endl;

				mywave.Filter(mywave.m_GreenWave, mywave.m_GreenNoise);

				//����˲�����
				for (auto data : mywave.m_GreenWave)
				{
					filter << data << " ";
				}
				filter << endl;

				mywave.Resolve(mywave.m_GreenWave, mywave.m_GreenGauPra, mywave.m_GreenNoise);

				//�����������
				for (auto data : mywave.m_GreenGauPra)
				{
					resolve << data.A << " " << data.b << " " << data.sigma << " ";
				}
				resolve << endl;

				mywave.Optimize(mywave.m_GreenWave, mywave.m_GreenGauPra);

				//�����������
				for (auto data : mywave.m_GreenGauPra)
				{
					iterate << data.A << " " << data.b << " " << data.sigma << " ";
				}
				iterate << endl;

				mywave.CalcuDepth(mywave.m_GreenGauPra, mywave.greenDepth);
				break;
			default:
				break;
			}

			//�����Ϣ���ļ�
			output_stream << mywave;

			//�ļ�ָ��ƫ��һ֡�������ݵ��ֽ�����2688/8
			j += 336;

			//��ӡ�������������ÿ���������
			printf("%5.2f%%", (float)j / (length / 800));
			printf("\b\b\b\b\b\b");

		}
		else
		{
			//���ܻ������λز����ݣ�uint16_t[CH.nL1] -> 2*n
			j += 2;
		}

	} while (!feof(m_filePtr));

	//�ļ������˳�
	if (feof(m_filePtr) == 1)
	{
		output_stream.close();
		origin.close();//��ʼ����
		filter.close();//�˲�����
		resolve.close();//����������
		iterate.close();//��������
        printf("finished!\n");
	}

}


/*���ܣ�	��ȡ��ˮ����
//out:	
*/
void ReadFile::readDeep()
{
	unsigned long long j = 0;
	HS_Lidar hs;

	//���ļ���λ��ָ���Ƶ��ļ�β��ȡ�ļ�����
	unsigned long long length;
	fseeko(m_filePtr, 0L, SEEK_END);
	length = ftello(m_filePtr);
    printf("ReadDeepProcessing:");


	//���ȶ����� output_stream  ios::out ʾ���,ios::app��ʾ������ļ�β��
	fstream output_stream;
	output_stream.open("DeepOut.txt", ios::out);

	int bgflag;
	float blueStd, greenStd;


	//�����ļ���ȡ����
	do {
		fseeko(m_filePtr, j * 8, SEEK_SET);

		//Ѱ��֡ͷ
		uint8_t header[8];
		memset(header, 0, sizeof(uint8_t) * 8);
		fread(header, sizeof(uint8_t), 8, m_filePtr);
		if (isHeaderRight(header))
		{
			//�������ݵ����̣�
			fseeko(m_filePtr, -8, SEEK_CUR);
			hs.initDeepData(m_filePtr);

			//��ȡͨ������ˮ�λز�����
			DeepWave dw;
			dw.GetDeepData(hs);

			//process
			blueStd = calculateSigma(dw.m_BlueDeep);
			greenStd = calculateSigma(dw.m_GreenDeep);

			blueStd >= 1.2*greenStd ? bgflag = BLUE : bgflag = GREEN;//�ж���ֵ

			switch (bgflag)
			{
			case BLUE:
				DeepWave::ostreamFlag = BLUE;

				dw.DeepFilter(dw.m_BlueDeep, dw.m_BlueDeepNoise);
				dw.DeepResolve(dw.m_BlueDeep, dw.m_BlueDeepPra, dw.m_BlueDeepNoise);
				dw.DeepOptimize(dw.m_BlueDeep, dw.m_BlueDeepPra);

				dw.CalcuDeepDepth(dw.m_BlueDeepPra, dw.blueDeepDepth);
				break;
			case GREEN:
				DeepWave::ostreamFlag = GREEN;

				dw.DeepFilter(dw.m_GreenDeep, dw.m_GreenDeepNoise);
				dw.DeepResolve(dw.m_GreenDeep, dw.m_GreenDeepPra, dw.m_GreenDeepNoise);
				dw.DeepOptimize(dw.m_GreenDeep, dw.m_GreenDeepPra);

				dw.CalcuDeepDepth(dw.m_GreenDeepPra, dw.greenDeepDepth);
				break;
			default:
				break;
			}

			//�����Ϣ���ļ�
			output_stream << dw;

			//�ļ�ָ��ƫ��һ֡�������ݵ��ֽ�����2688/8
			j += 336;

			//��ӡ����������
			cout.width(5);//���
		    //����������ȣ�������Ч����
			cout << fixed << setprecision(2) << /*100 * 8 **/ (float)j / (length / 800) << "%";
			cout << "\b\b\b\b\b\b";

		}
		else
		{
			//���ܻ������λز����ݣ�uint16_t[CH.nL1] -> 2*n
			j += 2;
		}

	} while (!feof(m_filePtr));

	//�ļ������˳�
	if (feof(m_filePtr) == 1)
	{
        printf("finished!\n");
	}
}


/*���ܣ�	��ȡ��ˮ���ݻ�Ͻ�����ͨ������
//out:
*/
void ReadFile::readDeepByRed()
{
	unsigned long long j = 0;
	HS_Lidar hs;

	//���ļ���λ��ָ���Ƶ��ļ�β��ȡ�ļ�����
	unsigned long long length;
	fseeko(m_filePtr, 0L, SEEK_END);
	length = ftello(m_filePtr);
    printf("ReadDeepByRedProcessing:");


	//���ȶ����� output_stream  ios::out ʾ���,ios::app��ʾ������ļ�β��
	fstream output_stream;
	output_stream.open("DeepByRedOut.txt", ios::out);

	int bgflag;
	float blueStd, greenStd;


	//�����ļ���ȡ����
	do {
		fseeko(m_filePtr, j * 8, SEEK_SET);

		//Ѱ��֡ͷ
		uint8_t header[8];
		memset(header, 0, sizeof(uint8_t) * 8);
		fread(header, sizeof(uint8_t), 8, m_filePtr);
		if (isHeaderRight(header))
		{
			//�������ݵ����̣�
			fseeko(m_filePtr, -8, SEEK_CUR);
			hs.initDeepData(m_filePtr);

			//��ȡͨ������ˮ�λز�����
			DeepWave dw;
			dw.GetDeepData(hs);

			//��ȡ������ˮ���
			dw.GetRedTime(dw.m_RedDeep, dw.redTime);

			//process
			blueStd = calculateSigma(dw.m_BlueDeep);
			greenStd = calculateSigma(dw.m_GreenDeep);

			blueStd >= 1.2*greenStd ? bgflag = BLUE : bgflag = GREEN;//�ж���ֵ

			switch (bgflag)
			{
			case BLUE:
				DeepWave::ostreamFlag = BLUE;

				dw.DeepFilter(dw.m_BlueDeep, dw.m_BlueDeepNoise);
				dw.DeepResolve(dw.m_BlueDeep, dw.m_BlueDeepPra, dw.m_BlueDeepNoise);
				dw.DeepOptimize(dw.m_BlueDeep, dw.m_BlueDeepPra);

				dw.CalcuDeepDepthByRed(dw.m_BlueDeepPra, dw.redTime, dw.blueDeepDepth);
				break;
			case GREEN:
				DeepWave::ostreamFlag = GREEN;

				dw.DeepFilter(dw.m_GreenDeep, dw.m_GreenDeepNoise);
				dw.DeepResolve(dw.m_GreenDeep, dw.m_GreenDeepPra, dw.m_GreenDeepNoise);
				dw.DeepOptimize(dw.m_GreenDeep, dw.m_GreenDeepPra);

				dw.CalcuDeepDepthByRed(dw.m_GreenDeepPra, dw.redTime, dw.greenDeepDepth);
				break;
			default:
				break;
			}

			//�����Ϣ���ļ�
			output_stream << dw;

			//�ļ�ָ��ƫ��һ֡�������ݵ��ֽ�����2688/8
			j += 336;

			//��ӡ�������������ÿ���������
			printf("%5.2f%%", (float)j / (length / 800));
			printf("\b\b\b\b\b\b");

		}
		else
		{
			//���ܻ������λز����ݣ�uint16_t[CH.nL1] -> 2*n
			j += 2;
		}

	} while (!feof(m_filePtr));

	//�ļ������˳�
	if (feof(m_filePtr) == 1)
	{
        printf("finished!\n");
	}
}


/*���ܣ�	
//out:
*/
void ReadFile::readDeepOutLas()
{
	unsigned long long j = 0;
	HS_Lidar hs;

	//���ļ���λ��ָ���Ƶ��ļ�β��ȡ�ļ�����
	unsigned long long length;
	fseeko(m_filePtr, 0L, SEEK_END);
	length = ftello(m_filePtr);
    printf("ReadDeepOutLasProcessing:");


	//���ȶ����� output_stream  ios::out ʾ���,ios::app��ʾ������ļ�β��
	fstream las_stream;
	las_stream.open("las2txt.txt", ios::out);

	int bgflag;
	float blueStd, greenStd;

	//��Чˮ��ļ�������ƽ��ˮ��
	int count = 0;
	float avedepth = 0;

	//�����ļ���ȡ����
	do {
		fseeko(m_filePtr, j * 8, SEEK_SET);

		//Ѱ��֡ͷ
		uint8_t header[8];
		memset(header, 0, sizeof(uint8_t) * 8);
		fread(header, sizeof(uint8_t), 8, m_filePtr);

		//��ʼ��¼λ�þ�γ��
		fseeko(m_filePtr, -8, SEEK_CUR);
		hs.initDeepData(m_filePtr);
		double tmpX = hs.header.dX;
		double tmpY = hs.header.dY;
		fseeko(m_filePtr, +8, SEEK_CUR);

		if (isHeaderRight(header))
		{
			//�������ݵ����̣�
			fseeko(m_filePtr, -8, SEEK_CUR);
			hs.initDeepData(m_filePtr);

			//��ȡͨ������ˮ�λز�����
			DeepWave dw;
			dw.GetDeepData(hs);

			//����γ�ȷ����仯ʱ����þ�γ�ȵ�ƽ����Чˮ��������
			if (hs.header.dX != tmpX || hs.header.dY != tmpY)
			{
				//�����������
				las_stream << setprecision(17) << tmpX << " " << tmpY << " " << setiosflags(ios::fixed) << setiosflags(ios::showpoint) << setprecision(3) << avedepth / count << endl;
			}

			//��ȡ������ˮ���
			dw.GetRedTime(dw.m_RedDeep, dw.redTime);

			//process
			blueStd = calculateSigma(dw.m_BlueDeep);
			greenStd = calculateSigma(dw.m_GreenDeep);

			blueStd >= 1.2*greenStd ? bgflag = BLUE : bgflag = GREEN;//�ж���ֵ

			switch (bgflag)
			{
			case BLUE:
				DeepWave::ostreamFlag = BLUE;

				dw.DeepFilter(dw.m_BlueDeep, dw.m_BlueDeepNoise);
				dw.DeepResolve(dw.m_BlueDeep, dw.m_BlueDeepPra, dw.m_BlueDeepNoise);
				dw.DeepOptimize(dw.m_BlueDeep, dw.m_BlueDeepPra);

				dw.CalcuDeepDepthByRed(dw.m_BlueDeepPra, dw.redTime, dw.blueDeepDepth);

				//��Чˮ�������һ�����
				if (dw.blueDeepDepth != 0)
				{
					avedepth += dw.blueDeepDepth;
					count++;
				}

				break;
			case GREEN:
				DeepWave::ostreamFlag = GREEN;

				dw.DeepFilter(dw.m_GreenDeep, dw.m_GreenDeepNoise);
				dw.DeepResolve(dw.m_GreenDeep, dw.m_GreenDeepPra, dw.m_GreenDeepNoise);
				dw.DeepOptimize(dw.m_GreenDeep, dw.m_GreenDeepPra);

				dw.CalcuDeepDepthByRed(dw.m_GreenDeepPra, dw.redTime, dw.greenDeepDepth);

				//��Чˮ�������һ�����
				if (dw.greenDeepDepth != 0)
				{
					avedepth += dw.blueDeepDepth;
					count++;
				}

				break;
			default:
				break;
			}

		
			//�ļ�ָ��ƫ��һ֡�������ݵ��ֽ�����2688/8
			j += 336;

			//��ӡ�������������ÿ���������
			printf("%5.2f%%", (float)j / (length / 800));
			printf("\b\b\b\b\b\b");

		}
		else
		{
			//���ܻ������λز����ݣ�uint16_t[CH.nL1] -> 2*n
			j += 2;
		}

	} while (!feof(m_filePtr));

	//�ļ������˳�
	if (feof(m_filePtr) == 1)
	{
		las_stream.close();
        printf("finished!\n");
	}
}