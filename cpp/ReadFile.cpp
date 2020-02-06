#include "../header/ReadFile.h"
#include <map>

#define BLUE true
#define GREEN false


//�ж�֡ͷ�Ƿ���ȷ
bool isHeaderRight(uint8_t header[8]) {
    uint8_t headerSign[] = {1, 35, 69, 103, 137, 171, 205, 239};
    bool returnVal = true;
    for (size_t i = 0; i < 8; i++) {
        if (header[i] != headerSign[i]) {
            returnVal = false;
            break;
        }
    }
    return returnVal;
}


ReadFile::ReadFile() {
}


ReadFile::~ReadFile() {
}


/*���ܣ�	���ö�ȡ�ļ���ָ��
//in:	��ȡ�ļ��ľ���·��
//out:	����·�������ļ�ָ��
*/
bool ReadFile::setFilename(char filename[100]) {
    m_filename = filename;
    m_filePtr = fopen(m_filename, "rb");
    if (m_filePtr == NULL) {
        printf("file loading failed!\n");
        return false;
    } else {
        printf("file loading successed!\n");
        return true;
    }
}


/*���ܣ�	����ȫ������ɫͨ��
//out:	��ȡͨ�������˲�ȥ��ֽ��Ż����
*/
void ReadFile::readBlueAll() {
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
        if (isHeaderRight(header)) {
            //�������ݵ����̣�
            fseeko(m_filePtr, -8, SEEK_CUR);
            hs.initData(m_filePtr);

            WaveData mywave;
            mywave.GetData(hs);
            mywave.Filter(mywave.m_BlueWave, mywave.m_BlueNoise);
            mywave.Resolve(mywave.m_BlueWave, mywave.m_BlueGauPra, mywave.m_BlueNoise);
            mywave.Optimize(mywave.m_BlueWave, mywave.m_BlueGauPra);

            mywave.CalcuDepth(mywave.m_BlueGauPra, mywave.blueDepth);

            //�����Ϣ���ļ�
            output_stream << mywave;

            //�ļ�ָ��ƫ��һ֡�������ݵ��ֽ�����2688/8
            j += 336;

            //��ӡ�������������ÿ���������
            printf("%5.2f%%", (float) j / (length / 800));
            printf("\b\b\b\b\b\b");

        } else {
            //���ܻ������λز����ݣ�uint16_t[CH.nL1] -> 2*n
            j += 2;
        }

    } while (!feof(m_filePtr));

    //�ļ������˳�
    if (feof(m_filePtr) == 1) {
        output_stream.close();
        printf("finished!\n");
    }
}


/*���ܣ�	����ȫ������ɫͨ��
//out:	��ȡͨ�������˲�ȥ��ֽ��Ż����
*/
void ReadFile::readGreenAll() {
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
        if (isHeaderRight(header)) {
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
            printf("%5.2f%%", (float) j / (length / 800));
            printf("\b\b\b\b\b\b");

        } else {
            //���ܻ������λز����ݣ�uint16_t[CH.nL1] -> 2*n
            j += 2;
        }

    } while (!feof(m_filePtr));

    //�ļ������˳�
    if (feof(m_filePtr) == 1) {
        output_stream.close();
        printf("finished!\n");
    }
}

/*���ܣ�	���ͨ������ѡ����Чͨ��
//out:	��ȡͨ�����ݣ�������ͨ���ı�׼���С����ѡ����Ӧ��ͨ��������ˮ�����
*/
void ReadFile::readMix() {
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
        if (isHeaderRight(header)) {
            //�������ݵ����̣�
            fseeko(m_filePtr, -8, SEEK_CUR);
            hs.initData(m_filePtr);

            WaveData mywave;
            mywave.GetData(hs);

            blueStd = calculateSigma(mywave.m_BlueWave);
            greenStd = calculateSigma(mywave.m_GreenWave);

            blueStd >= 1.2 * greenStd ? bgflag = BLUE : bgflag = GREEN;//�ж���ֵ

            switch (bgflag) {
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
            printf("%5.2f%%", (float) j / (length / 800));
            printf("\b\b\b\b\b\b");

        } else {
            //���ܻ������λز����ݣ�uint16_t[CH.nL1] -> 2*n
            j += 2;
        }

    } while (!feof(m_filePtr));

    //�ļ������˳�
    if (feof(m_filePtr) == 1) {
        output_stream.close();
        printf("finished!\n");
    }
}


/*���ܣ�	���ԭʼ���ݣ��˲������ݣ���ʼ�����ݣ������������Լ�ԭʼ��˹�ֽⷨ���
//out:	4files
*/
void ReadFile::outputData() {
    unsigned long long j = 0;
    unsigned long long index = 0;
    HS_Lidar hs;

    //���ļ���λ��ָ���Ƶ��ļ�β��ȡ�ļ�����
    unsigned long long length;
    fseeko(m_filePtr, 0L, SEEK_END);
    length = ftello(m_filePtr);
    printf("OutputDataProcessing:");

    //���ȶ����� output_stream  ios::out ʾ���,ios::app��ʾ������ļ�β��
    fstream output_stream;
    output_stream.open("Final.txt", ios::out);

    fstream origin;//��ʼ����
    fstream filter;//�˲�����
    fstream resolve;//����������
    fstream iterate;//��������
    fstream gaussB;//��ͳ�ⷨ��ͨ��
    fstream gaussG;//��ͳ�ⷨ��ͨ��
    origin.open("Origin.txt", ios::out);
    filter.open("Filter.txt", ios::out);
    resolve.open("Resolve.txt", ios::out);
    gaussB.open("GaussB.txt", ios::out);
    gaussG.open("GaussG.txt", ios::out);

    int bgflag;
    float blueStd, greenStd;

    //�����ļ���ȡ����
    do {
        fseeko(m_filePtr, j * 8, SEEK_SET);

        //Ѱ��֡ͷ
        uint8_t header[8];
        memset(header, 0, sizeof(uint8_t) * 8);
        fread(header, sizeof(uint8_t), 8, m_filePtr);
        if (isHeaderRight(header)) {
            //�������
            index++;
            //�������ݵ����̣�
            fseeko(m_filePtr, -8, SEEK_CUR);
            hs.initData(m_filePtr);

            WaveData mywave;
            mywave.GetData(hs);

            blueStd = calculateSigma(mywave.m_BlueWave);
            greenStd = calculateSigma(mywave.m_GreenWave);

            blueStd >= 1.2 * greenStd ? bgflag = BLUE : bgflag = GREEN;//�ж���ֵ

            //�������ͨ���ڸ��Ե�����
            //===========Blue start===============
            WaveData::ostreamFlag = BLUE;

            //���ԭʼ����
            origin << "<" << index << "B" << ">" << endl;
            for (auto data : mywave.m_BlueWave) {
                origin << data << " ";
            }
            origin << endl;

            mywave.Filter(mywave.m_BlueWave, mywave.m_BlueNoise);

            //����˲�����
            filter << "<" << index << "B" << ">" << endl;
            for (auto data : mywave.m_BlueWave) {
                filter << data << " ";
            }
            filter << endl;

            mywave.Resolve(mywave.m_BlueWave, mywave.m_BlueGauPra, mywave.m_BlueNoise);

            //�����������
            resolve << "<" << index << "B" << ">" << endl;
            //�����˹��������
            for (auto data : mywave.m_BlueGauPra) {
                resolve << data.A << " " << data.b << " " << data.sigma << " ";
            }
            resolve << endl;
            //����ӷ�
            int sizeB = mywave.m_BlueGauPra.size();
            for (int k = 0; k < sizeB; k++) {
                resolve << "Component" << k + 1 << endl;
                for (int i = 0; i < 320; ++i) {
                    resolve << mywave.m_BlueGauPra[k].A *
                               exp(-(i - mywave.m_BlueGauPra[k].b) * (i - mywave.m_BlueGauPra[k].b) /
                                   (2 * (mywave.m_BlueGauPra[k].sigma) * (mywave.m_BlueGauPra[k].sigma))) << " ";
                }
                resolve << endl;
            }
            //���������Ϣ
            resolve << "Sum" << endl;
            for (int x = 0; x < 320; x++) {
                int size = mywave.m_BlueGauPra.size();
                float da = 0;
                for (int i = 0; i < size; i++) {
                    da += mywave.m_BlueGauPra[i].A *
                          exp(-(x - mywave.m_BlueGauPra[i].b) * (x - mywave.m_BlueGauPra[i].b) /
                              (2 * (mywave.m_BlueGauPra[i].sigma) * (mywave.m_BlueGauPra[i].sigma)));
                }
                resolve << da << " ";
            }
            resolve << endl;


            //�����ͨ�ĸ�˹�ֽⷨ�õ����
            mywave.CalcuDepthByGauss(mywave.m_BlueGauPra, mywave.blueDepth);
            gaussB << "<" << index << ">" << " "
                   << mywave.m_time.year << " "
                   << mywave.m_time.month << " "
                   << mywave.m_time.day << " "
                   << mywave.m_time.hour << " "
                   << mywave.m_time.minute << " "
                   << mywave.m_time.second << " "
                   << "B" << " "
                   << mywave.blueDepth << "m ";
            for (auto data : mywave.m_BlueGauPra) {
                gaussB << data.A << " " << data.b << " " << data.sigma << " ";
            }
            gaussB << endl;


            //�����������
            mywave.Optimize(mywave.m_BlueWave, mywave.m_BlueGauPra);
            iterate << "<" << index << "B" << ">" << endl;
            //�����˹��������
            for (auto data : mywave.m_BlueGauPra) {
                iterate << data.A << " " << data.b << " " << data.sigma << " ";
            }
            iterate << endl;
            //����ӷ�
            //int sizeB = mywave.m_BlueGauPra.size();
            for (int k = 0; k < sizeB; k++) {
                iterate << "Component" << k + 1 << endl;
                for (int i = 0; i < 320; ++i) {
                    iterate << mywave.m_BlueGauPra[k].A *
                               exp(-(i - mywave.m_BlueGauPra[k].b) * (i - mywave.m_BlueGauPra[k].b) /
                                   (2 * (mywave.m_BlueGauPra[k].sigma) * (mywave.m_BlueGauPra[k].sigma))) << " ";
                }
                iterate << endl;
            }
            //���������Ϣ
            iterate << "Sum" << endl;
            for (int x = 0; x < 320; x++) {
                int size = mywave.m_BlueGauPra.size();
                float da = 0;
                for (int i = 0; i < size; i++) {
                    da += mywave.m_BlueGauPra[i].A *
                          exp(-(x - mywave.m_BlueGauPra[i].b) * (x - mywave.m_BlueGauPra[i].b) /
                              (2 * (mywave.m_BlueGauPra[i].sigma) * (mywave.m_BlueGauPra[i].sigma)));
                }
                iterate << da << " ";
            }
            iterate << endl;
            //==========Blue end================


            //==========Green start=============
            WaveData::ostreamFlag = GREEN;

            //�����ʼ����
            origin << "<" << index << "G" << ">" << endl;
            for (auto data : mywave.m_GreenWave) {
                origin << data << " ";
            }
            origin << endl;

            mywave.Filter(mywave.m_GreenWave, mywave.m_GreenNoise);

            //����˲�����
            filter << "<" << index << "G" << ">" << endl;
            for (auto data : mywave.m_GreenWave) {
                filter << data << " ";
            }
            filter << endl;

            mywave.Resolve(mywave.m_GreenWave, mywave.m_GreenGauPra, mywave.m_GreenNoise);

            //�����������
            resolve << "<" << index << "G" << ">" << endl;
            //�����˹��������
            for (auto data : mywave.m_GreenGauPra) {
                resolve << data.A << " " << data.b << " " << data.sigma << " ";
            }
            resolve << endl;
            //����ӷ�
            int sizeG = mywave.m_GreenGauPra.size();
            for (int k = 0; k < sizeG; k++) {
                resolve << "Component" << k + 1 << endl;
                for (int i = 0; i < 320; ++i) {
                    resolve << mywave.m_GreenGauPra[k].A *
                               exp(-(i - mywave.m_GreenGauPra[k].b) * (i - mywave.m_GreenGauPra[k].b) /
                                   (2 * (mywave.m_GreenGauPra[k].sigma) * (mywave.m_GreenGauPra[k].sigma))) << " ";
                }
                resolve << endl;
            }
            //���������Ϣ
            resolve << "Sum" << endl;
            for (int x = 0; x < 320; x++) {
                int size = mywave.m_GreenGauPra.size();
                float da = 0;
                for (int i = 0; i < size; i++) {
                    da += mywave.m_GreenGauPra[i].A *
                          exp(-(x - mywave.m_GreenGauPra[i].b) * (x - mywave.m_GreenGauPra[i].b) /
                              (2 * (mywave.m_GreenGauPra[i].sigma) * (mywave.m_GreenGauPra[i].sigma)));
                }
                resolve << da << " ";
            }
            resolve << endl;

            //�����ͨ�ĸ�˹�ֽⷨ�õ����
            mywave.CalcuDepthByGauss(mywave.m_GreenGauPra, mywave.greenDepth);
            gaussG << "<" << index << ">" << " "
                   << mywave.m_time.year << " "
                   << mywave.m_time.month << " "
                   << mywave.m_time.day << " "
                   << mywave.m_time.hour << " "
                   << mywave.m_time.minute << " "
                   << mywave.m_time.second << " "
                   << "G" << " "
                   << mywave.greenDepth << "m ";
            for (auto data : mywave.m_GreenGauPra) {
                gaussG << data.A << " " << data.b << " " << data.sigma << " ";
            }
            gaussG << endl;

            //�����������
            mywave.Optimize(mywave.m_GreenWave, mywave.m_GreenGauPra);
            iterate << "<" << index << "G" << ">" << endl;
            //�����˹��������
            for (auto data : mywave.m_GreenGauPra) {
                iterate << data.A << " " << data.b << " " << data.sigma << " ";
            }
            iterate << endl;
            //����ӷ�
            //int sizeB = mywave.m_BlueGauPra.size();
            for (int k = 0; k < sizeG; k++) {
                iterate << "Component" << k + 1 << endl;
                for (int i = 0; i < 320; ++i) {
                    iterate << mywave.m_GreenGauPra[k].A *
                               exp(-(i - mywave.m_GreenGauPra[k].b) * (i - mywave.m_GreenGauPra[k].b) /
                                   (2 * (mywave.m_GreenGauPra[k].sigma) * (mywave.m_GreenGauPra[k].sigma))) << " ";
                }
                iterate << endl;
            }
            //���������Ϣ
            iterate << "Sum" << endl;
            for (int x = 0; x < 320; x++) {
                int size = mywave.m_GreenGauPra.size();
                float da = 0;
                for (int i = 0; i < size; i++) {
                    da += mywave.m_GreenGauPra[i].A *
                          exp(-(x - mywave.m_GreenGauPra[i].b) * (x - mywave.m_GreenGauPra[i].b) /
                              (2 * (mywave.m_GreenGauPra[i].sigma) * (mywave.m_GreenGauPra[i].sigma)));
                }
                iterate << da << " ";
            }
            iterate << endl;
            //============Green end============

            //���������Ϣ��ѡȡ�ľ���ͨ��
            switch (bgflag) {
                case BLUE:
                    mywave.CalcuDepth(mywave.m_BlueGauPra, mywave.blueDepth);
                    //�����Ϣ���ļ�
                    output_stream << "<" << index << ">" << " "
                                  << mywave.m_time.year << " "
                                  << mywave.m_time.month << " "
                                  << mywave.m_time.day << " "
                                  << mywave.m_time.hour << " "
                                  << mywave.m_time.minute << " "
                                  << mywave.m_time.second << " "
                                  << "B" << " "
                                  << mywave.blueDepth << "m ";
                    for (auto data : mywave.m_BlueGauPra) {
                        output_stream << data.A << " " << data.b << " " << data.sigma << " ";
                    }
                    output_stream << endl;

                    break;
                case GREEN:
                    mywave.CalcuDepth(mywave.m_GreenGauPra, mywave.greenDepth);
                    output_stream << "<" << index << ">" << " "
                                  << mywave.m_time.year << " "
                                  << mywave.m_time.month << " "
                                  << mywave.m_time.day << " "
                                  << mywave.m_time.hour << " "
                                  << mywave.m_time.minute << " "
                                  << mywave.m_time.second << " "
                                  << "G" << " "
                                  << mywave.greenDepth << "m ";
                    for (auto data : mywave.m_GreenGauPra) {
                        output_stream << data.A << " " << data.b << " " << data.sigma << " ";
                    }
                    output_stream << endl;

                    break;
                default:
                    break;
            }

            //�ļ�ָ��ƫ��һ֡�������ݵ��ֽ�����2688/8
            j += 336;

            //��ӡ�������������ÿ���������
            printf("%5.2f%%", (float) j / (length / 800));
            printf("\b\b\b\b\b\b");

        } else {
            //���ܻ������λز����ݣ�uint16_t[CH.nL1] -> 2*n
            j += 2;
        }

    } while (!feof(m_filePtr));

    //�ļ������˳�
    if (feof(m_filePtr) == 1) {
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
void ReadFile::readDeep() {
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
        if (isHeaderRight(header)) {
            //�������ݵ����̣�
            fseeko(m_filePtr, -8, SEEK_CUR);
            hs.initDeepData(m_filePtr);

            //��ȡͨ������ˮ�λز�����
            DeepWave dw;
            dw.GetDeepData(hs);

            //process
            blueStd = calculateSigma(dw.m_BlueDeep);
            greenStd = calculateSigma(dw.m_GreenDeep);

            blueStd >= 1.2 * greenStd ? bgflag = BLUE : bgflag = GREEN;//�ж���ֵ

            switch (bgflag) {
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
            cout << fixed << setprecision(2) << /*100 * 8 **/ (float) j / (length / 800) << "%";
            cout << "\b\b\b\b\b\b";

        } else {
            //���ܻ������λز����ݣ�uint16_t[CH.nL1] -> 2*n
            j += 2;
        }

    } while (!feof(m_filePtr));

    //�ļ������˳�
    if (feof(m_filePtr) == 1) {
        printf("finished!\n");
    }
}


/*���ܣ�	��ȡ��ˮ���ݻ�Ͻ�����ͨ������
//out:
*/
void ReadFile::readDeepByRed() {
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
        if (isHeaderRight(header)) {
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

            blueStd >= 1.2 * greenStd ? bgflag = BLUE : bgflag = GREEN;//�ж���ֵ

            switch (bgflag) {
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
            printf("%5.2f%%", (float) j / (length / 800));
            printf("\b\b\b\b\b\b");

        } else {
            //���ܻ������λز����ݣ�uint16_t[CH.nL1] -> 2*n
            j += 2;
        }

    } while (!feof(m_filePtr));

    //�ļ������˳�
    if (feof(m_filePtr) == 1) {
        printf("finished!\n");
    }
}


/*���ܣ�	
//out:
*/
void ReadFile::readDeepOutLas() {
    unsigned long long j = 0;
    HS_Lidar hs;

    //���ļ���λ��ָ���Ƶ��ļ�β��ȡ�ļ�����
    unsigned long long length;
    fseeko(m_filePtr, 0L, SEEK_END);
    length = ftello(m_filePtr);
    printf("ReadDeepOutLasProcessing:");


    //���ȶ����� output_stream  ios::out ʾ���,ios::app��ʾ������ļ�β��
    fstream las_stream;
    las_stream.open("Las2Txt.txt", ios::out);

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

        if (isHeaderRight(header)) {
            //�������ݵ����̣�
            fseeko(m_filePtr, -8, SEEK_CUR);
            hs.initDeepData(m_filePtr);

            //��ȡͨ������ˮ�λز�����
            DeepWave dw;
            dw.GetDeepData(hs);

            //����γ�ȷ����仯ʱ����þ�γ�ȵ�ƽ����Чˮ��������
            if (hs.header.dX != tmpX || hs.header.dY != tmpY) {
                //�����������
                las_stream << setprecision(17) << tmpX << " " << tmpY << " " << setiosflags(ios::fixed)
                           << setiosflags(ios::showpoint) << setprecision(3) << avedepth / count << endl;
            }

            //��ȡ������ˮ���
            dw.GetRedTime(dw.m_RedDeep, dw.redTime);

            //process
            blueStd = calculateSigma(dw.m_BlueDeep);
            greenStd = calculateSigma(dw.m_GreenDeep);

            blueStd >= 1.2 * greenStd ? bgflag = BLUE : bgflag = GREEN;//�ж���ֵ

            switch (bgflag) {
                case BLUE:
                    DeepWave::ostreamFlag = BLUE;

                    dw.DeepFilter(dw.m_BlueDeep, dw.m_BlueDeepNoise);
                    dw.DeepResolve(dw.m_BlueDeep, dw.m_BlueDeepPra, dw.m_BlueDeepNoise);
                    dw.DeepOptimize(dw.m_BlueDeep, dw.m_BlueDeepPra);

                    dw.CalcuDeepDepthByRed(dw.m_BlueDeepPra, dw.redTime, dw.blueDeepDepth);

                    //��Чˮ�������һ�����
                    if (dw.blueDeepDepth != 0) {
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
                    if (dw.greenDeepDepth != 0) {
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
            printf("%5.2f%%", (float) j / (length / 800));
            printf("\b\b\b\b\b\b");

        } else {
            //���ܻ������λز����ݣ�uint16_t[CH.nL1] -> 2*n
            j += 2;
        }

    } while (!feof(m_filePtr));

    //�ļ������˳�
    if (feof(m_filePtr) == 1) {
        las_stream.close();
        printf("finished!\n");
    }
}


/**
 * �������ݣ������ͬˮ��ֲ��Ĳ����� ��˹�ֽ���LM�Ż�ǰ���ƽ�����
 */
void ReadFile::dataAnalysis() {
    unsigned long long j = 0;
    HS_Lidar hs;

    //���ļ���λ��ָ���Ƶ��ļ�β��ȡ�ļ�����
    unsigned long long length;
    fseeko(m_filePtr, 0L, SEEK_END);
    length = ftello(m_filePtr);
    printf("DataAnalysising:");

    //���ȶ����� output_stream  ios::out ʾ���,ios::app��ʾ������ļ�β��
    fstream output_stream;
    output_stream.open("Analysis.txt", ios::out);

    int bgflag;
    float blueStd, greenStd;

    //���ݷ�������
    //ʮ����ˮ��ֲ���
    int depthCount[10] = {0};
    //�ֲ��Ĳ�����
    int depthComponetSize[10] = {0};
    //��˹�ֽ��ƽ����
    float GauAvgDiff[10] = {0};
    //��˹�ֽ�ǰ�󷽲�
    float BeforeGauVariance[10] = {0};
    float AfterGauVariance[10] = {0};
    //lm��ƽ����
    float LmAvgDiff[10] = {0};
    //lmǰ�󷽲�
    float BeforeLmVariance[10] = {0};
    float AfterLmVariance[10] = {0};

    //�����ļ���ȡ����
    do {
        fseeko(m_filePtr, j * 8, SEEK_SET);

        //Ѱ��֡ͷ
        uint8_t header[8];
        memset(header, 0, sizeof(uint8_t) * 8);
        fread(header, sizeof(uint8_t), 8, m_filePtr);
        if (isHeaderRight(header)) {
            //�������ݵ����̣�
            fseeko(m_filePtr, -8, SEEK_CUR);
            hs.initData(m_filePtr);

            WaveData mywave;
            mywave.GetData(hs);

            //��ȡʱ�䷶Χ
            if (!inDuration(mywave.m_time)) {
                //�ļ�ָ��ƫ��һ֡�������ݵ��ֽ�����2688/8
                j += 336;

                //��ӡ�������������ÿ���������
                printf("%5.2f%%", (float) j / (length / 800));
                printf("\b\b\b\b\b\b");
                continue;
            }

            blueStd = calculateSigma(mywave.m_BlueWave);
            greenStd = calculateSigma(mywave.m_GreenWave);

            blueStd >= 1.2 * greenStd ? bgflag = BLUE : bgflag = GREEN;//�ж���ֵ

            float gauAvgDiff;
            float beforeGauVariance;
            float afterGauVariance;
            float lmAvgDiff;
            float beforeLmVariance;
            float afterLmVariance;

            switch (bgflag) {
                case BLUE:
                    WaveData::ostreamFlag = BLUE;

                    mywave.Filter(mywave.m_BlueWave, mywave.m_BlueNoise);
                    mywave.Resolve(mywave.m_BlueWave, mywave.m_BlueGauPra, mywave.m_BlueNoise);
                    mywave.CalcuAfter(mywave.m_BlueGauPra, mywave.afterGauss);
                    gauAvgDiff = calcuAvgDiff(mywave.m_BlueWave, mywave.afterGauss);
                    beforeGauVariance = calcuVariance(mywave.m_BlueWave);
                    afterGauVariance = calcuVariance(mywave.afterGauss);

                    mywave.Optimize(mywave.m_BlueWave, mywave.m_BlueGauPra);
                    mywave.CalcuAfter(mywave.m_BlueGauPra, mywave.afterLM);
                    lmAvgDiff = calcuAvgDiff(mywave.afterGauss, mywave.afterLM);
                    beforeLmVariance = calcuVariance(mywave.afterGauss);
                    afterLmVariance = calcuVariance(mywave.afterLM);

                    mywave.CalcuDepth(mywave.m_BlueGauPra, mywave.blueDepth);

                    //ͳ��
                    if (mywave.blueDepth > 0 && mywave.blueDepth <= 1.0) {
                        depthCount[0]++;
                        depthComponetSize[0] += mywave.m_BlueGauPra.size();
                        GauAvgDiff[0] += gauAvgDiff;
                        BeforeGauVariance[0] += beforeGauVariance;
                        AfterGauVariance[0] += afterGauVariance;
                        LmAvgDiff[0] += lmAvgDiff;
                        BeforeLmVariance[0] += beforeLmVariance;
                        AfterLmVariance[0] += afterLmVariance;
                    } else if (mywave.blueDepth > 1 && mywave.blueDepth <= 2.0) {
                        depthCount[1]++;
                        depthComponetSize[1] += mywave.m_BlueGauPra.size();
                        GauAvgDiff[1] += gauAvgDiff;
                        BeforeGauVariance[1] += beforeGauVariance;
                        AfterGauVariance[1] += afterGauVariance;
                        LmAvgDiff[1] += lmAvgDiff;
                        BeforeLmVariance[1] += beforeLmVariance;
                        AfterLmVariance[1] += afterLmVariance;
                    } else if (mywave.blueDepth > 2 && mywave.blueDepth <= 3.0) {
                        depthCount[2]++;
                        depthComponetSize[2] += mywave.m_BlueGauPra.size();
                        GauAvgDiff[2] += gauAvgDiff;
                        BeforeGauVariance[2] += beforeGauVariance;
                        AfterGauVariance[2] += afterGauVariance;
                        LmAvgDiff[2] += lmAvgDiff;
                        BeforeLmVariance[2] += beforeLmVariance;
                        AfterLmVariance[2] += afterLmVariance;
                    } else if (mywave.blueDepth > 3 && mywave.blueDepth <= 4.0) {
                        depthCount[3]++;
                        depthComponetSize[3] += mywave.m_BlueGauPra.size();
                        GauAvgDiff[3] += gauAvgDiff;
                        BeforeGauVariance[3] += beforeGauVariance;
                        AfterGauVariance[3] += afterGauVariance;
                        LmAvgDiff[3] += lmAvgDiff;
                        BeforeLmVariance[3] += beforeLmVariance;
                        AfterLmVariance[3] += afterLmVariance;
                    } else if (mywave.blueDepth > 4 && mywave.blueDepth <= 5.0) {
                        depthCount[4]++;
                        depthComponetSize[4] += mywave.m_BlueGauPra.size();
                        GauAvgDiff[4] += gauAvgDiff;
                        BeforeGauVariance[4] += beforeGauVariance;
                        AfterGauVariance[4] += afterGauVariance;
                        LmAvgDiff[4] += lmAvgDiff;
                        BeforeLmVariance[4] += beforeLmVariance;
                        AfterLmVariance[4] += afterLmVariance;
                    } else if (mywave.blueDepth > 5 && mywave.blueDepth <= 6.0) {
                        depthCount[5]++;
                        depthComponetSize[5] += mywave.m_BlueGauPra.size();
                        GauAvgDiff[5] += gauAvgDiff;
                        BeforeGauVariance[5] += beforeGauVariance;
                        AfterGauVariance[5] += afterGauVariance;
                        LmAvgDiff[5] += lmAvgDiff;
                        BeforeLmVariance[5] += beforeLmVariance;
                        AfterLmVariance[5] += afterLmVariance;
                    } else if (mywave.blueDepth > 6 && mywave.blueDepth <= 7.0) {
                        depthCount[6]++;
                        depthComponetSize[6] += mywave.m_BlueGauPra.size();
                        GauAvgDiff[6] += gauAvgDiff;
                        BeforeGauVariance[6] += beforeGauVariance;
                        AfterGauVariance[6] += afterGauVariance;
                        LmAvgDiff[6] += lmAvgDiff;
                        BeforeLmVariance[6] += beforeLmVariance;
                        AfterLmVariance[6] += afterLmVariance;
                    } else if (mywave.blueDepth > 7 && mywave.blueDepth <= 8.0) {
                        depthCount[7]++;
                        depthComponetSize[7] += mywave.m_BlueGauPra.size();
                        GauAvgDiff[7] += gauAvgDiff;
                        BeforeGauVariance[7] += beforeGauVariance;
                        AfterGauVariance[7] += afterGauVariance;
                        LmAvgDiff[7] += lmAvgDiff;
                        BeforeLmVariance[7] += beforeLmVariance;
                        AfterLmVariance[7] += afterLmVariance;
                    } else if (mywave.blueDepth > 8 && mywave.blueDepth <= 9.0) {
                        depthCount[8]++;
                        depthComponetSize[8] += mywave.m_BlueGauPra.size();
                        GauAvgDiff[8] += gauAvgDiff;
                        BeforeGauVariance[8] += beforeGauVariance;
                        AfterGauVariance[8] += afterGauVariance;
                        LmAvgDiff[8] += lmAvgDiff;
                        BeforeLmVariance[8] += beforeLmVariance;
                        AfterLmVariance[8] += afterLmVariance;
                    } else if (mywave.blueDepth > 9 && mywave.blueDepth <= 10.0) {
                        depthCount[9]++;
                        depthComponetSize[9] += mywave.m_BlueGauPra.size();
                        GauAvgDiff[9] += gauAvgDiff;
                        BeforeGauVariance[9] += beforeGauVariance;
                        AfterGauVariance[9] += afterGauVariance;
                        LmAvgDiff[9] += lmAvgDiff;
                        BeforeLmVariance[9] += beforeLmVariance;
                        AfterLmVariance[9] += afterLmVariance;
                    }

                    break;
                case GREEN:
                    WaveData::ostreamFlag = GREEN;

                    mywave.Filter(mywave.m_GreenWave, mywave.m_GreenNoise);
                    mywave.Resolve(mywave.m_GreenWave, mywave.m_GreenGauPra, mywave.m_GreenNoise);
                    mywave.CalcuAfter(mywave.m_GreenGauPra, mywave.afterGauss);
                    gauAvgDiff = calcuAvgDiff(mywave.m_GreenWave, mywave.afterGauss);
                    beforeGauVariance = calcuVariance(mywave.m_GreenWave);
                    afterGauVariance = calcuVariance(mywave.afterGauss);

                    mywave.Optimize(mywave.m_GreenWave, mywave.m_GreenGauPra);
                    mywave.CalcuAfter(mywave.m_GreenGauPra, mywave.afterLM);
                    lmAvgDiff = calcuAvgDiff(mywave.afterGauss, mywave.afterLM);
                    beforeLmVariance = calcuVariance(mywave.afterGauss);
                    afterLmVariance = calcuVariance(mywave.afterLM);

                    mywave.CalcuDepth(mywave.m_GreenGauPra, mywave.greenDepth);

                    //ͳ��
                    if (mywave.greenDepth > 0 && mywave.greenDepth <= 1.0) {
                        depthCount[0]++;
                        depthComponetSize[0] += mywave.m_GreenGauPra.size();
                        GauAvgDiff[0] += gauAvgDiff;
                        BeforeGauVariance[0] += beforeGauVariance;
                        AfterGauVariance[0] += afterGauVariance;
                        LmAvgDiff[0] += lmAvgDiff;
                        BeforeLmVariance[0] += beforeLmVariance;
                        AfterLmVariance[0] += afterLmVariance;
                    } else if (mywave.greenDepth > 1 && mywave.greenDepth <= 2.0) {
                        depthCount[1]++;
                        depthComponetSize[1] += mywave.m_GreenGauPra.size();
                        GauAvgDiff[1] += gauAvgDiff;
                        BeforeGauVariance[1] += beforeGauVariance;
                        AfterGauVariance[1] += afterGauVariance;
                        LmAvgDiff[1] += lmAvgDiff;
                        BeforeLmVariance[1] += beforeLmVariance;
                        AfterLmVariance[1] += afterLmVariance;
                    } else if (mywave.greenDepth > 2 && mywave.greenDepth <= 3.0) {
                        depthCount[2]++;
                        depthComponetSize[2] += mywave.m_GreenGauPra.size();
                        GauAvgDiff[2] += gauAvgDiff;
                        BeforeGauVariance[2] += beforeGauVariance;
                        AfterGauVariance[2] += afterGauVariance;
                        LmAvgDiff[2] += lmAvgDiff;
                        BeforeLmVariance[2] += beforeLmVariance;
                        AfterLmVariance[2] += afterLmVariance;
                    } else if (mywave.greenDepth > 3 && mywave.greenDepth <= 4.0) {
                        depthCount[3]++;
                        depthComponetSize[3] += mywave.m_GreenGauPra.size();
                        GauAvgDiff[3] += gauAvgDiff;
                        BeforeGauVariance[3] += beforeGauVariance;
                        AfterGauVariance[3] += afterGauVariance;
                        LmAvgDiff[3] += lmAvgDiff;
                        BeforeLmVariance[3] += beforeLmVariance;
                        AfterLmVariance[3] += afterLmVariance;
                    } else if (mywave.greenDepth > 4 && mywave.greenDepth <= 5.0) {
                        depthCount[4]++;
                        depthComponetSize[4] += mywave.m_GreenGauPra.size();
                        GauAvgDiff[4] += gauAvgDiff;
                        BeforeGauVariance[4] += beforeGauVariance;
                        AfterGauVariance[4] += afterGauVariance;
                        LmAvgDiff[4] += lmAvgDiff;
                        BeforeLmVariance[4] += beforeLmVariance;
                        AfterLmVariance[4] += afterLmVariance;
                    } else if (mywave.greenDepth > 5 && mywave.greenDepth <= 6.0) {
                        depthCount[5]++;
                        depthComponetSize[5] += mywave.m_GreenGauPra.size();
                        GauAvgDiff[5] += gauAvgDiff;
                        BeforeGauVariance[5] += beforeGauVariance;
                        AfterGauVariance[5] += afterGauVariance;
                        LmAvgDiff[5] += lmAvgDiff;
                        BeforeLmVariance[5] += beforeLmVariance;
                        AfterLmVariance[5] += afterLmVariance;
                    } else if (mywave.greenDepth > 6 && mywave.greenDepth <= 7.0) {
                        depthCount[6]++;
                        depthComponetSize[6] += mywave.m_GreenGauPra.size();
                        GauAvgDiff[6] += gauAvgDiff;
                        BeforeGauVariance[6] += beforeGauVariance;
                        AfterGauVariance[6] += afterGauVariance;
                        LmAvgDiff[6] += lmAvgDiff;
                        BeforeLmVariance[6] += beforeLmVariance;
                        AfterLmVariance[6] += afterLmVariance;
                    } else if (mywave.greenDepth > 7 && mywave.greenDepth <= 8.0) {
                        depthCount[7]++;
                        depthComponetSize[7] += mywave.m_GreenGauPra.size();
                        GauAvgDiff[7] += gauAvgDiff;
                        BeforeGauVariance[7] += beforeGauVariance;
                        AfterGauVariance[7] += afterGauVariance;
                        LmAvgDiff[7] += lmAvgDiff;
                        BeforeLmVariance[7] += beforeLmVariance;
                        AfterLmVariance[7] += afterLmVariance;
                    } else if (mywave.greenDepth > 8 && mywave.greenDepth <= 9.0) {
                        depthCount[8]++;
                        depthComponetSize[8] += mywave.m_GreenGauPra.size();
                        GauAvgDiff[8] += gauAvgDiff;
                        BeforeGauVariance[8] += beforeGauVariance;
                        AfterGauVariance[8] += afterGauVariance;
                        LmAvgDiff[8] += lmAvgDiff;
                        BeforeLmVariance[8] += beforeLmVariance;
                        AfterLmVariance[8] += afterLmVariance;
                    } else if (mywave.greenDepth > 9 && mywave.greenDepth <= 10.0) {
                        depthCount[9]++;
                        depthComponetSize[9] += mywave.m_GreenGauPra.size();
                        GauAvgDiff[9] += gauAvgDiff;
                        BeforeGauVariance[9] += beforeGauVariance;
                        AfterGauVariance[9] += afterGauVariance;
                        LmAvgDiff[9] += lmAvgDiff;
                        BeforeLmVariance[9] += beforeLmVariance;
                        AfterLmVariance[9] += afterLmVariance;
                    }

                    break;
                default:
                    break;
            }


            //�ļ�ָ��ƫ��һ֡�������ݵ��ֽ�����2688/8
            j += 336;

            //��ӡ�������������ÿ���������
            printf("%5.2f%%", (float) j / (length / 800));
            printf("\b\b\b\b\b\b");

        } else {
            //���ܻ������λز����ݣ�uint16_t[CH.nL1] -> 2*n
            j += 2;
        }

    } while (!feof(m_filePtr));

    for (int i = 0; i < 10; ++i) {
        output_stream << i << " depthCount " << depthCount[i] << " depthComponetSize " << depthComponetSize[i]
                      << " AVG " << (float) depthComponetSize[i] / depthCount[i] << endl;
    }

    for (int i = 0; i < 10; ++i) {
        output_stream << i << " GauAvgDiff " << GauAvgDiff[i] << " BeforeGauVariance " << BeforeGauVariance[i]
                      << " AfterGauVariance " << AfterGauVariance[i]
                      << " AVG " << GauAvgDiff[i] / depthCount[i] << " " << BeforeGauVariance[i] / depthCount[i] << " "
                      << AfterGauVariance[i] / depthCount[i] << endl;
    }

    for (int i = 0; i < 10; ++i) {
        output_stream << i << " LmAvgDiff " << LmAvgDiff[i] << " BeforeLmVariance " << BeforeLmVariance[i]
                      << " AfterLmVariance " << AfterLmVariance[i]
                      << " AVG " << LmAvgDiff[i] / depthCount[i] << " " << BeforeLmVariance[i] / depthCount[i] << " "
                      << AfterLmVariance[i] / depthCount[i] << endl;
    }

    //�ļ������˳�
    if (feof(m_filePtr) == 1) {

        output_stream.close();
        printf("finished!\n");
    }
}

/**
 * �ж��Ƿ���Ҫ����
 * @param time ��������utcʱ��
 * @return bool
 */
bool ReadFile::inDuration(Time time) {
    Time start = {2018, 5, 29, 12, 20, 0};
    Time end = {2018, 5, 29, 12, 33, 59};
    return time.year >= start.year && time.year <= end.year
           && time.month >= start.month && time.month <= end.month
           && time.day >= start.day && time.day <= end.day
           && time.hour >= start.hour && time.hour <= end.hour
           && time.minute >= start.minute && time.minute <= end.minute
           && time.second >= start.second && time.second <= end.second;
}

/**
 * ����ƽ����
 * @param v1 ����1
 * @param v2 ����2
 * @return float
 */
float ReadFile::calcuAvgDiff(vector<float> &v1, vector<float> &v2) {
    float ret = 0.0;
    if (v1.size() == 320 && v2.size() == 320) {
        for (std::vector<int>::size_type i = 0; i != v1.size(); i++) {
            ret += v1[i] - v2[i];
        }
        return ret / 320;
    }
    return ret;
}

/**
 * ���㷽��
 * @param v1 ����
 * @param avg ƽ��ֵ
 * @return float
 */
float ReadFile::calcuVariance(vector<float> &v) {
    float ret = 0;
    if (v.size() == 320) {
        float avg = 0.0;
        for (std::vector<int>::size_type i = 0; i != v.size(); i++) {
            avg += v[i];
        }
        avg /= 320;
        for (std::vector<int>::size_type i = 0; i != v.size(); i++) {
            ret += pow((v[i] - avg), 2);
        }
        return ret / 320;
    }
    return ret;
}

