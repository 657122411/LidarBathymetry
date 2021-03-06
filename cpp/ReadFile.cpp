#include "../header/ReadFile.h"
#include <map>

#define BLUE true
#define GREEN false


//判断帧头是否正确
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


/*功能：	设置读取文件的指针
//in:	读取文件的绝对路径
//out:	将该路径赋给文件指针
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


/*功能：	处理全数据蓝色通道
//out:	读取通道数据滤波去噪分解优化输出
*/
void ReadFile::readBlueAll() {
    unsigned long long j = 0;
    HS_Lidar hs;

    //把文件的位置指针移到文件尾获取文件长度
    unsigned long long length;
    fseeko(m_filePtr, 0L, SEEK_END);
    length = ftello(m_filePtr);

    printf("BLueChannelProcessing:");

    //首先定义流 output_stream  ios::out 示输出,ios::app表示输出到文件尾。
    fstream output_stream;
    output_stream.open("BlueOut.txt", ios::out);

    //设置输出流flag
    WaveData::ostreamFlag = BLUE;

    //遍历文件获取数据
    do {
        fseeko(m_filePtr, j * 8, SEEK_SET);

        //寻找帧头
        uint8_t header[8];
        memset(header, 0, sizeof(uint8_t) * 8);
        fread(header, sizeof(uint8_t), 8, m_filePtr);
        if (isHeaderRight(header)) {
            //处理数据的流程：
            fseeko(m_filePtr, -8, SEEK_CUR);
            hs.initData(m_filePtr);

            WaveData mywave;
            mywave.GetData(hs);
            mywave.Filter(mywave.m_BlueWave, mywave.m_BlueNoise);
            mywave.Resolve(mywave.m_BlueWave, mywave.m_BlueGauPra, mywave.m_BlueNoise);
            mywave.Optimize(mywave.m_BlueWave, mywave.m_BlueGauPra);

            mywave.CalcuDepth(mywave.m_BlueGauPra, mywave.blueDepth);

            //输出信息到文件
            output_stream << mywave;

            //文件指针偏移一帧完整数据的字节数：2688/8
            j += 336;

            //打印处理进程情况设置宽度输出精度
            printf("%5.2f%%", (float) j / (length / 800));
            printf("\b\b\b\b\b\b");

        } else {
            //可能会多出二段回波数据：uint16_t[CH.nL1] -> 2*n
            j += 2;
        }

    } while (!feof(m_filePtr));

    //文件结束退出
    if (feof(m_filePtr) == 1) {
        output_stream.close();
        printf("finished!\n");
    }
}


/*功能：	处理全数据绿色通道
//out:	读取通道数据滤波去噪分解优化输出
*/
void ReadFile::readGreenAll() {
    unsigned long long j = 0;
    HS_Lidar hs;

    //把文件的位置指针移到文件尾获取文件长度
    unsigned long long length;
    fseeko(m_filePtr, 0L, SEEK_END);
    length = ftello(m_filePtr);
    printf("GreenChannelProcessing:");

    //首先定义流 output_stream  ios::out 示输出,ios::app表示输出到文件尾。
    fstream output_stream;
    output_stream.open("GreenOut.txt", ios::out);

    //设置输出流flag
    WaveData::ostreamFlag = GREEN;

    //遍历文件获取数据
    do {
        fseeko(m_filePtr, j * 8, SEEK_SET);

        //寻找帧头
        uint8_t header[8];
        memset(header, 0, sizeof(uint8_t) * 8);
        fread(header, sizeof(uint8_t), 8, m_filePtr);
        if (isHeaderRight(header)) {
            //处理数据的流程：
            fseeko(m_filePtr, -8, SEEK_CUR);
            hs.initData(m_filePtr);

            WaveData mywave;
            mywave.GetData(hs);
            mywave.Filter(mywave.m_GreenWave, mywave.m_GreenNoise);
            mywave.Resolve(mywave.m_GreenWave, mywave.m_GreenGauPra, mywave.m_GreenNoise);
            mywave.Optimize(mywave.m_GreenWave, mywave.m_GreenGauPra);

            mywave.CalcuDepth(mywave.m_GreenGauPra, mywave.greenDepth);

            //输出信息到文件
            output_stream << mywave;

            //文件指针偏移一帧完整数据的字节数：2688/8
            j += 336;

            //打印处理进程情况设置宽度输出精度
            printf("%5.2f%%", (float) j / (length / 800));
            printf("\b\b\b\b\b\b");

        } else {
            //可能会多出二段回波数据：uint16_t[CH.nL1] -> 2*n
            j += 2;
        }

    } while (!feof(m_filePtr));

    //文件结束退出
    if (feof(m_filePtr) == 1) {
        output_stream.close();
        printf("finished!\n");
    }
}

/*功能：	混合通道处理，选择有效通道
//out:	读取通道数据，根据两通道的标准差大小决定选择相应的通道数据做水深解算
*/
void ReadFile::readMix() {
    unsigned long long j = 0;
    HS_Lidar hs;

    //把文件的位置指针移到文件尾获取文件长度
    unsigned long long length;
    fseeko(m_filePtr, 0L, SEEK_END);
    length = ftello(m_filePtr);
    printf("MixChannelProcessing:");

    //首先定义流 output_stream  ios::out 示输出,ios::app表示输出到文件尾。
    fstream output_stream;
    output_stream.open("MixOut.txt", ios::out);

    int bgflag;
    float blueStd, greenStd;

    //遍历文件获取数据
    do {
        fseeko(m_filePtr, j * 8, SEEK_SET);

        //寻找帧头
        uint8_t header[8];
        memset(header, 0, sizeof(uint8_t) * 8);
        fread(header, sizeof(uint8_t), 8, m_filePtr);
        if (isHeaderRight(header)) {
            //处理数据的流程：
            fseeko(m_filePtr, -8, SEEK_CUR);
            hs.initData(m_filePtr);

            WaveData mywave;
            mywave.GetData(hs);

            blueStd = calculateSigma(mywave.m_BlueWave);
            greenStd = calculateSigma(mywave.m_GreenWave);

            blueStd >= 1.2 * greenStd ? bgflag = BLUE : bgflag = GREEN;//判断阈值

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

            //输出信息到文件
            output_stream << mywave;

            //文件指针偏移一帧完整数据的字节数：2688/8
            j += 336;

            //打印处理进程情况设置宽度输出精度
            printf("%5.2f%%", (float) j / (length / 800));
            printf("\b\b\b\b\b\b");

        } else {
            //可能会多出二段回波数据：uint16_t[CH.nL1] -> 2*n
            j += 2;
        }

    } while (!feof(m_filePtr));

    //文件结束退出
    if (feof(m_filePtr) == 1) {
        output_stream.close();
        printf("finished!\n");
    }
}


/*功能：	输出原始数据，滤波后数据，初始解数据，迭代后数据以及原始高斯分解法结果
//out:	4files
*/
void ReadFile::outputData() {
    unsigned long long j = 0;
    unsigned long long index = 0;
    HS_Lidar hs;

    //把文件的位置指针移到文件尾获取文件长度
    unsigned long long length;
    fseeko(m_filePtr, 0L, SEEK_END);
    length = ftello(m_filePtr);
    printf("OutputDataProcessing:");

    //首先定义流 output_stream  ios::out 示输出,ios::app表示输出到文件尾。
    fstream output_stream;
    output_stream.open("Final.txt", ios::out);

    fstream origin;//初始数据
    fstream filter;//滤波数据
    fstream resolve;//初解算数据
    fstream iterate;//迭代数据
    fstream gaussB;//传统解法蓝通道
    fstream gaussG;//传统解法绿通道
    origin.open("Origin.txt", ios::out);
    filter.open("Filter.txt", ios::out);
    resolve.open("Resolve.txt", ios::out);
    gaussB.open("GaussB.txt", ios::out);
    gaussG.open("GaussG.txt", ios::out);

    int bgflag;
    float blueStd, greenStd;

    //遍历文件获取数据
    do {
        fseeko(m_filePtr, j * 8, SEEK_SET);

        //寻找帧头
        uint8_t header[8];
        memset(header, 0, sizeof(uint8_t) * 8);
        fread(header, sizeof(uint8_t), 8, m_filePtr);
        if (isHeaderRight(header)) {
            //数据序号
            index++;
            //处理数据的流程：
            fseeko(m_filePtr, -8, SEEK_CUR);
            hs.initData(m_filePtr);

            WaveData mywave;
            mywave.GetData(hs);

            blueStd = calculateSigma(mywave.m_BlueWave);
            greenStd = calculateSigma(mywave.m_GreenWave);

            blueStd >= 1.2 * greenStd ? bgflag = BLUE : bgflag = GREEN;//判断阈值

            //输出两个通道内各自的数据
            //===========Blue start===============
            WaveData::ostreamFlag = BLUE;

            //输出原始数据
            origin << "<" << index << "B" << ">" << endl;
            for (auto data : mywave.m_BlueWave) {
                origin << data << " ";
            }
            origin << endl;

            mywave.Filter(mywave.m_BlueWave, mywave.m_BlueNoise);

            //输出滤波数据
            filter << "<" << index << "B" << ">" << endl;
            for (auto data : mywave.m_BlueWave) {
                filter << data << " ";
            }
            filter << endl;

            mywave.Resolve(mywave.m_BlueWave, mywave.m_BlueGauPra, mywave.m_BlueNoise);

            //输出初解数据
            resolve << "<" << index << "B" << ">" << endl;
            //输出高斯分量参数
            for (auto data : mywave.m_BlueGauPra) {
                resolve << data.A << " " << data.b << " " << data.sigma << " ";
            }
            resolve << endl;
            //输出子峰
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
            //输出总量信息
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


            //输出普通的高斯分解法得到结果
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


            //输出迭代数据
            mywave.Optimize(mywave.m_BlueWave, mywave.m_BlueGauPra);
            iterate << "<" << index << "B" << ">" << endl;
            //输出高斯分量参数
            for (auto data : mywave.m_BlueGauPra) {
                iterate << data.A << " " << data.b << " " << data.sigma << " ";
            }
            iterate << endl;
            //输出子峰
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
            //输出总量信息
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

            //输出初始数据
            origin << "<" << index << "G" << ">" << endl;
            for (auto data : mywave.m_GreenWave) {
                origin << data << " ";
            }
            origin << endl;

            mywave.Filter(mywave.m_GreenWave, mywave.m_GreenNoise);

            //输出滤波数据
            filter << "<" << index << "G" << ">" << endl;
            for (auto data : mywave.m_GreenWave) {
                filter << data << " ";
            }
            filter << endl;

            mywave.Resolve(mywave.m_GreenWave, mywave.m_GreenGauPra, mywave.m_GreenNoise);

            //输出初解数据
            resolve << "<" << index << "G" << ">" << endl;
            //输出高斯分量参数
            for (auto data : mywave.m_GreenGauPra) {
                resolve << data.A << " " << data.b << " " << data.sigma << " ";
            }
            resolve << endl;
            //输出子峰
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
            //输出总量信息
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

            //输出普通的高斯分解法得到结果
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

            //输出迭代数据
            mywave.Optimize(mywave.m_GreenWave, mywave.m_GreenGauPra);
            iterate << "<" << index << "G" << ">" << endl;
            //输出高斯分量参数
            for (auto data : mywave.m_GreenGauPra) {
                iterate << data.A << " " << data.b << " " << data.sigma << " ";
            }
            iterate << endl;
            //输出子峰
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
            //输出总量信息
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

            //输出解算信息中选取的具体通道
            switch (bgflag) {
                case BLUE:
                    mywave.CalcuDepth(mywave.m_BlueGauPra, mywave.blueDepth);
                    //输出信息到文件
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

            //文件指针偏移一帧完整数据的字节数：2688/8
            j += 336;

            //打印处理进程情况设置宽度输出精度
            printf("%5.2f%%", (float) j / (length / 800));
            printf("\b\b\b\b\b\b");

        } else {
            //可能会多出二段回波数据：uint16_t[CH.nL1] -> 2*n
            j += 2;
        }

    } while (!feof(m_filePtr));

    //文件结束退出
    if (feof(m_filePtr) == 1) {
        output_stream.close();
        origin.close();//初始数据
        filter.close();//滤波数据
        resolve.close();//初解算数据
        iterate.close();//迭代数据
        printf("finished!\n");
    }

}


/*功能：	读取深水数据
//out:
*/
void ReadFile::readDeep() {
    unsigned long long j = 0;
    HS_Lidar hs;

    //把文件的位置指针移到文件尾获取文件长度
    unsigned long long length;
    fseeko(m_filePtr, 0L, SEEK_END);
    length = ftello(m_filePtr);
    printf("ReadDeepProcessing:");


    //首先定义流 output_stream  ios::out 示输出,ios::app表示输出到文件尾。
    fstream output_stream;
    output_stream.open("DeepOut.txt", ios::out);

    int bgflag;
    float blueStd, greenStd;


    //遍历文件获取数据
    do {
        fseeko(m_filePtr, j * 8, SEEK_SET);

        //寻找帧头
        uint8_t header[8];
        memset(header, 0, sizeof(uint8_t) * 8);
        fread(header, sizeof(uint8_t), 8, m_filePtr);
        if (isHeaderRight(header)) {
            //处理数据的流程：
            fseeko(m_filePtr, -8, SEEK_CUR);
            hs.initDeepData(m_filePtr);

            //获取通道的深水段回波数据
            DeepWave dw;
            dw.GetDeepData(hs);

            //process
            blueStd = calculateSigma(dw.m_BlueDeep);
            greenStd = calculateSigma(dw.m_GreenDeep);

            blueStd >= 1.2 * greenStd ? bgflag = BLUE : bgflag = GREEN;//判断阈值

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

            //输出信息到文件
            output_stream << dw;

            //文件指针偏移一帧完整数据的字节数：2688/8
            j += 336;

            //打印处理进程情况
            cout.width(5);//宽度
            //设置输出精度，保留有效数字
            cout << fixed << setprecision(2) << /*100 * 8 **/ (float) j / (length / 800) << "%";
            cout << "\b\b\b\b\b\b";

        } else {
            //可能会多出二段回波数据：uint16_t[CH.nL1] -> 2*n
            j += 2;
        }

    } while (!feof(m_filePtr));

    //文件结束退出
    if (feof(m_filePtr) == 1) {
        printf("finished!\n");
    }
}


/*功能：	读取深水数据混合近红外通道处理
//out:
*/
void ReadFile::readDeepByRed() {
    unsigned long long j = 0;
    HS_Lidar hs;

    //把文件的位置指针移到文件尾获取文件长度
    unsigned long long length;
    fseeko(m_filePtr, 0L, SEEK_END);
    length = ftello(m_filePtr);
    printf("ReadDeepByRedProcessing:");


    //首先定义流 output_stream  ios::out 示输出,ios::app表示输出到文件尾。
    fstream output_stream;
    output_stream.open("DeepByRedOut.txt", ios::out);

    int bgflag;
    float blueStd, greenStd;


    //遍历文件获取数据
    do {
        fseeko(m_filePtr, j * 8, SEEK_SET);

        //寻找帧头
        uint8_t header[8];
        memset(header, 0, sizeof(uint8_t) * 8);
        fread(header, sizeof(uint8_t), 8, m_filePtr);
        if (isHeaderRight(header)) {
            //处理数据的流程：
            fseeko(m_filePtr, -8, SEEK_CUR);
            hs.initDeepData(m_filePtr);

            //获取通道的深水段回波数据
            DeepWave dw;
            dw.GetDeepData(hs);

            //获取近红外水面点
            dw.GetRedTime(dw.m_RedDeep, dw.redTime);

            //process
            blueStd = calculateSigma(dw.m_BlueDeep);
            greenStd = calculateSigma(dw.m_GreenDeep);

            blueStd >= 1.2 * greenStd ? bgflag = BLUE : bgflag = GREEN;//判断阈值

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

            //输出信息到文件
            output_stream << dw;

            //文件指针偏移一帧完整数据的字节数：2688/8
            j += 336;

            //打印处理进程情况设置宽度输出精度
            printf("%5.2f%%", (float) j / (length / 800));
            printf("\b\b\b\b\b\b");

        } else {
            //可能会多出二段回波数据：uint16_t[CH.nL1] -> 2*n
            j += 2;
        }

    } while (!feof(m_filePtr));

    //文件结束退出
    if (feof(m_filePtr) == 1) {
        printf("finished!\n");
    }
}


/*功能：
//out:
*/
void ReadFile::readDeepOutLas() {
    unsigned long long j = 0;
    HS_Lidar hs;

    //把文件的位置指针移到文件尾获取文件长度
    unsigned long long length;
    fseeko(m_filePtr, 0L, SEEK_END);
    length = ftello(m_filePtr);
    printf("ReadDeepOutLasProcessing:");


    //首先定义流 output_stream  ios::out 示输出,ios::app表示输出到文件尾。
    fstream las_stream;
    las_stream.open("Las2Txt.txt", ios::out);

    int bgflag;
    float blueStd, greenStd;

    //有效水深的计数器，平均水深
    int count = 0;
    float avedepth = 0;

    //遍历文件获取数据
    do {
        fseeko(m_filePtr, j * 8, SEEK_SET);

        //寻找帧头
        uint8_t header[8];
        memset(header, 0, sizeof(uint8_t) * 8);
        fread(header, sizeof(uint8_t), 8, m_filePtr);

        //初始记录位置经纬度
        fseeko(m_filePtr, -8, SEEK_CUR);
        hs.initDeepData(m_filePtr);
        double tmpX = hs.header.dX;
        double tmpY = hs.header.dY;
        fseeko(m_filePtr, +8, SEEK_CUR);

        if (isHeaderRight(header)) {
            //处理数据的流程：
            fseeko(m_filePtr, -8, SEEK_CUR);
            hs.initDeepData(m_filePtr);

            //获取通道的深水段回波数据
            DeepWave dw;
            dw.GetDeepData(hs);

            //当经纬度发生变化时计算该经纬度的平均有效水深进行输出
            if (hs.header.dX != tmpX || hs.header.dY != tmpY) {
                //控制输出精度
                las_stream << setprecision(17) << tmpX << " " << tmpY << " " << setiosflags(ios::fixed)
                           << setiosflags(ios::showpoint) << setprecision(3) << avedepth / count << endl;
            }

            //获取近红外水面点
            dw.GetRedTime(dw.m_RedDeep, dw.redTime);

            //process
            blueStd = calculateSigma(dw.m_BlueDeep);
            greenStd = calculateSigma(dw.m_GreenDeep);

            blueStd >= 1.2 * greenStd ? bgflag = BLUE : bgflag = GREEN;//判断阈值

            switch (bgflag) {
                case BLUE:
                    DeepWave::ostreamFlag = BLUE;

                    dw.DeepFilter(dw.m_BlueDeep, dw.m_BlueDeepNoise);
                    dw.DeepResolve(dw.m_BlueDeep, dw.m_BlueDeepPra, dw.m_BlueDeepNoise);
                    dw.DeepOptimize(dw.m_BlueDeep, dw.m_BlueDeepPra);

                    dw.CalcuDeepDepthByRed(dw.m_BlueDeepPra, dw.redTime, dw.blueDeepDepth);

                    //有效水深计数加一并求和
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

                    //有效水深计数加一并求和
                    if (dw.greenDeepDepth != 0) {
                        avedepth += dw.blueDeepDepth;
                        count++;
                    }

                    break;
                default:
                    break;
            }


            //文件指针偏移一帧完整数据的字节数：2688/8
            j += 336;

            //打印处理进程情况设置宽度输出精度
            printf("%5.2f%%", (float) j / (length / 800));
            printf("\b\b\b\b\b\b");

        } else {
            //可能会多出二段回波数据：uint16_t[CH.nL1] -> 2*n
            j += 2;
        }

    } while (!feof(m_filePtr));

    //文件结束退出
    if (feof(m_filePtr) == 1) {
        las_stream.close();
        printf("finished!\n");
    }
}


/**
 * 分析数据：输出不同水深分布的波形数 高斯分解与LM优化前后的平均差方差
 */
void ReadFile::dataAnalysis() {
    unsigned long long j = 0;
    HS_Lidar hs;

    //把文件的位置指针移到文件尾获取文件长度
    unsigned long long length;
    fseeko(m_filePtr, 0L, SEEK_END);
    length = ftello(m_filePtr);
    printf("DataAnalysising:");

    //首先定义流 output_stream  ios::out 示输出,ios::app表示输出到文件尾。
    fstream output_stream;
    output_stream.open("Analysis.txt", ios::out);

    int bgflag;
    float blueStd, greenStd;

    //数据分析容器
    //十米内水深分布数
    int depthCount[10] = {0};
    //分布的波形数
    int depthComponetSize[10] = {0};
    //后向散射分布的波形数
    int backComponetSize[10] = {0};
    //后向散射波形能量
    float backComponetSum[10] = {0};
    //使用B通道数据
    int Bchannel[10] = {0};
    //使用G通道数据
    int Gchannel[10] = {0};
    //数据截取范围
    int Region[10] = {0};
    int EffectiveRegion[10] = {0};
    int EffectiveRegionCount[10] = {0};
    //滤波后平均差
    float FilterAvgDiff[10] = {0};
    //滤波前后方差
    float FilterVariance[10] = {0};
    //高斯分解后平均差
    float GauAvgDiff[10] = {0};
    //高斯分解前后方差
    float GauVariance[10] = {0};
    //lm后平均差
    float LmAvgDiff[10] = {0};
    //lm前后方差
    float LmVariance[10] = {0};
    //lm前后水面水底高斯分量的aus三参数变化
    float DiffA0[10] = {0};
    float DiffU0[10] = {0};
    float DiffSigma0[10] = {0};
    float DiffA1[10] = {0};
    float DiffU1[10] = {0};
    float DiffSigma1[10] = {0};

    //遍历文件获取数据
    do {
        fseeko(m_filePtr, j * 8, SEEK_SET);

        //寻找帧头
        uint8_t header[8];
        memset(header, 0, sizeof(uint8_t) * 8);
        fread(header, sizeof(uint8_t), 8, m_filePtr);
        if (isHeaderRight(header)) {
            //处理数据的流程：
            fseeko(m_filePtr, -8, SEEK_CUR);
            hs.initData(m_filePtr);

            WaveData mywave;
            mywave.GetData(hs);

            //截取时间范围
            if (!inDuration(mywave.m_time)) {
                //文件指针偏移一帧完整数据的字节数：2688/8
                j += 336;

                //打印处理进程情况设置宽度输出精度
                printf("%5.2f%%", (float) j / (length / 800));
                printf("\b\b\b\b\b\b");
                continue;
            }

            blueStd = calculateSigma(mywave.m_BlueWave);
            greenStd = calculateSigma(mywave.m_GreenWave);

            blueStd >= 1.2 * greenStd ? bgflag = BLUE : bgflag = GREEN;//判断阈值

            float filterAvgDiff;
            float filterVariance;
            float gauAvgDiff;
            float gauVariance;
            float lmAvgDiff;
            float lmVariance;
            vector<float> beforeFilter;
            float diffA0, diffA1, diffU0, diffU1, diffSigma0, diffSigma1;
            vector<GaussParameter> init;
            int index[2];
            int range = 0;
            int region;

            switch (bgflag) {
                case BLUE:
                    WaveData::ostreamFlag = BLUE;

                    beforeFilter = mywave.m_BlueWave;
                    region = mywave.FilterRegion(mywave.m_BlueWave, mywave.m_BlueNoise);
                    filterAvgDiff = calcuAvgAbsDiff(beforeFilter, mywave.m_BlueWave);
                    filterVariance = calcuAbsVariance(beforeFilter, mywave.m_BlueWave, filterAvgDiff);

                    mywave.Resolve(mywave.m_BlueWave, mywave.m_BlueGauPra, mywave.m_BlueNoise);
                    mywave.CalcuAfter(mywave.m_BlueGauPra, mywave.afterGauss);
                    gauAvgDiff = calcuAvgDiff(mywave.m_BlueWave, mywave.afterGauss);
                    gauVariance = calcuVariance(mywave.m_BlueWave, mywave.afterGauss, gauAvgDiff);

                    init = mywave.m_BlueGauPra;

                    mywave.Optimize(mywave.m_BlueWave, mywave.m_BlueGauPra);
                    mywave.CalcuAfter(mywave.m_BlueGauPra, mywave.afterLM);
                    lmAvgDiff = calcuAvgDiff(mywave.afterGauss, mywave.afterLM);
                    lmVariance = calcuVariance(mywave.afterGauss, mywave.afterLM, lmAvgDiff);

                    mywave.CalcuDepthOutIndex(mywave.m_BlueGauPra, mywave.blueDepth, index);
                    diffA0 = getDiffA(index[0], init, mywave.m_BlueGauPra);
                    diffA1 = getDiffA(index[1], init, mywave.m_BlueGauPra);
                    diffU0 = getDiffU(index[0], init, mywave.m_BlueGauPra);
                    diffU1 = getDiffU(index[1], init, mywave.m_BlueGauPra);
                    diffSigma0 = getDiffSigma(index[0], init, mywave.m_BlueGauPra);
                    diffSigma1 = getDiffSigma(index[1], init, mywave.m_BlueGauPra);

                    //按1米间隔统计
                    range = mywave.blueDepth == 0 ? -1 : (int) mywave.blueDepth;
                    if (range >= 0 && range <= 9) {
                        depthCount[range]++;
                        depthComponetSize[range] += mywave.m_BlueGauPra.size();
                        Bchannel[range]++;
                        Region[range] += region;
                        if (region != 320) {
                            EffectiveRegionCount[range]++;
                            EffectiveRegion[range] += region;
                        }
                        FilterAvgDiff[range] += filterAvgDiff;
                        FilterVariance[range] += filterVariance;
                        GauAvgDiff[range] += gauAvgDiff;
                        GauVariance[range] += gauVariance;
                        LmAvgDiff[range] += lmAvgDiff;
                        LmVariance[range] += lmVariance;
                        DiffA0[range] += diffA0;
                        DiffU0[range] += diffU0;
                        DiffSigma0[range] += diffSigma0;
                        DiffA1[range] += diffA1;
                        DiffU1[range] += diffU1;
                        DiffSigma1[range] += diffSigma1;
                        backComponetSize[range] += (index[1] - index[0] - 1);
                        backComponetSum[range] += getEnergy(mywave.m_BlueGauPra, index[0], index[1]);
                    }

                    break;
                case GREEN:
                    WaveData::ostreamFlag = GREEN;

                    beforeFilter = mywave.m_GreenWave;
                    region = mywave.FilterRegion(mywave.m_GreenWave, mywave.m_GreenNoise);
                    filterAvgDiff = calcuAvgAbsDiff(beforeFilter, mywave.m_GreenWave);
                    filterVariance = calcuAbsVariance(beforeFilter, mywave.m_GreenWave, filterAvgDiff);

                    mywave.Resolve(mywave.m_GreenWave, mywave.m_GreenGauPra, mywave.m_GreenNoise);
                    mywave.CalcuAfter(mywave.m_GreenGauPra, mywave.afterGauss);
                    gauAvgDiff = calcuAvgDiff(mywave.m_GreenWave, mywave.afterGauss);
                    gauVariance = calcuVariance(mywave.m_GreenWave, mywave.afterGauss, gauAvgDiff);

                    init = mywave.m_GreenGauPra;

                    mywave.Optimize(mywave.m_GreenWave, mywave.m_GreenGauPra);
                    mywave.CalcuAfter(mywave.m_GreenGauPra, mywave.afterLM);
                    lmAvgDiff = calcuAvgDiff(mywave.afterGauss, mywave.afterLM);
                    lmVariance = calcuVariance(mywave.afterGauss, mywave.afterLM, lmAvgDiff);

                    mywave.CalcuDepthOutIndex(mywave.m_GreenGauPra, mywave.greenDepth, index);
                    diffA0 = getDiffA(index[0], init, mywave.m_GreenGauPra);
                    diffA1 = getDiffA(index[1], init, mywave.m_GreenGauPra);
                    diffU0 = getDiffU(index[0], init, mywave.m_GreenGauPra);
                    diffU1 = getDiffU(index[1], init, mywave.m_GreenGauPra);
                    diffSigma0 = getDiffSigma(index[0], init, mywave.m_GreenGauPra);
                    diffSigma1 = getDiffSigma(index[1], init, mywave.m_GreenGauPra);

                    //按1米间隔统计
                    range = mywave.greenDepth == 0 ? -1 : (int) mywave.greenDepth;
                    if (range >= 0 && range <= 9) {
                        depthCount[range]++;
                        depthComponetSize[range] += mywave.m_GreenGauPra.size();
                        Gchannel[range]++;
                        Region[range] += region;
                        if (region != 320) {
                            EffectiveRegionCount[range]++;
                            EffectiveRegion[range] += region;
                        }
                        FilterAvgDiff[range] += filterAvgDiff;
                        FilterVariance[range] += filterVariance;
                        GauAvgDiff[range] += gauAvgDiff;
                        GauVariance[range] += gauVariance;
                        LmAvgDiff[range] += lmAvgDiff;
                        LmVariance[range] += lmVariance;
                        DiffA0[range] += diffA0;
                        DiffU0[range] += diffU0;
                        DiffSigma0[range] += diffSigma0;
                        DiffA1[range] += diffA1;
                        DiffU1[range] += diffU1;
                        DiffSigma1[range] += diffSigma1;
                        backComponetSize[range] += (index[1] - index[0] - 1);
                        backComponetSum[range] += getEnergy(mywave.m_GreenGauPra, index[0], index[1]);
                    }

                    break;
            }


            //文件指针偏移一帧完整数据的字节数：2688/8
            j += 336;

            //打印处理进程情况设置宽度输出精度
            printf("%5.2f%%", (float) j / (length / 800));
            printf("\b\b\b\b\b\b");

        } else {
            //可能会多出二段回波数据：uint16_t[CH.nL1] -> 2*n
            j += 2;
        }

    } while (!feof(m_filePtr));

    for (int i = 0; i < 10; ++i) {
        output_stream << i << " depthCount " << depthCount[i] << " depthComponetSize " << depthComponetSize[i]
                      << " AVG " << (float) depthComponetSize[i] / depthCount[i]
                      << " backAvgSize " << (float) backComponetSize[i] / depthCount[i]
                      << " backAvgEnergy " << backComponetSum[i] / depthCount[i]
                      << endl;
    }

    for (int i = 0; i < 10; ++i) {
        output_stream << i << " Bchannel " << Bchannel[i] << " " << (float) Bchannel[i] / (Bchannel[i] + Gchannel[i])
                      << " Gchannel " << Gchannel[i] << " " << (float) Gchannel[i] / (Bchannel[i] + Gchannel[i])
                      << endl;
    }

    for (int i = 0; i < 10; ++i) {
        output_stream << i << " RegionAvg " << Region[i] / depthCount[i]
                      << " EffctiveRegionAvg " << EffectiveRegion[i] / EffectiveRegionCount[i]
                      << endl;
    }

    for (int i = 0; i < 10; ++i) {
        output_stream << i << " FilterAvgDiff " << FilterAvgDiff[i] << " FilterVariance " << FilterVariance[i]
                      << " AVG " << FilterAvgDiff[i] / depthCount[i] << " " << FilterVariance[i] / depthCount[i] << " "
                      << endl;
    }

    for (int i = 0; i < 10; ++i) {
        output_stream << i << " GauAvgDiff " << GauAvgDiff[i] << " GauVariance " << GauVariance[i]
                      << " AVG " << GauAvgDiff[i] / depthCount[i] << " " << GauVariance[i] / depthCount[i] << " "
                      << endl;
    }

    for (int i = 0; i < 10; ++i) {
        output_stream << i << " LmAvgDiff " << LmAvgDiff[i] << " LmVariance " << LmVariance[i]
                      << " AVG " << LmAvgDiff[i] / depthCount[i] << " " << LmVariance[i] / depthCount[i] << " "
                      << endl;
    }

    for (int i = 0; i < 10; ++i) {
        output_stream << i << " DiffA0 " << DiffA0[i] / depthCount[i] << " "
                      << " DiffU0 " << DiffU0[i] / depthCount[i] << " "
                      << " DiffSigma0 " << DiffSigma0[i] / depthCount[i] << " "
                      << " DiffA1 " << DiffA1[i] / depthCount[i] << " "
                      << " DiffU1 " << DiffU1[i] / depthCount[i] << " "
                      << " DiffSigma1 " << DiffSigma1[i] / depthCount[i] << " "
                      << endl;
    }

    //文件结束退出
    if (feof(m_filePtr) == 1) {

        output_stream.close();
        printf("finished!\n");
    }
}

/**
 * 判断是否需要处理
 * @param time 数据现在utc时间
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
 * 计算平均差
 * @param v1 数组1
 * @param v2 数据2
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
 * 计算绝对平均差
 * @param v1 数组1
 * @param v2 数据2
 * @return float
 */
float ReadFile::calcuAvgAbsDiff(vector<float> &v1, vector<float> &v2) {
    float ret = 0.0;
    if (v1.size() == 320 && v2.size() == 320) {
        for (std::vector<int>::size_type i = 0; i != v1.size(); i++) {
            ret += abs(v1[i] - v2[i]);
        }
        return ret / 320;
    }
    return ret;
}

/**
 * 计算绝对方差
 * @param v1 数组
 * @param avg 平均值
 * @return float
 */
float ReadFile::calcuAbsVariance(vector<float> &v1, vector<float> &v2, float avg) {
    float ret = 0;
    if (v1.size() == 320 && v2.size() == 320) {
        for (std::vector<int>::size_type i = 0; i != v1.size(); i++) {
            ret += pow((abs(v1[i] - v2[i]) - avg), 2);
        }
        return ret / 320;
    }
    return ret;
}

/**
 * 计算方差
 * @param v1 数组
 * @param avg 平均值
 * @return float
 */
float ReadFile::calcuVariance(vector<float> &v1, vector<float> &v2, float avg) {
    float ret = 0;
    if (v1.size() == 320 && v2.size() == 320) {
        for (std::vector<int>::size_type i = 0; i != v1.size(); i++) {
            ret += pow((v1[i] - v2[i] - avg), 2);
        }
        return ret / 320;
    }
    return ret;
}

/**
 * 计算参数变化
 * @param index 分量索引
 * @param v1 lm前参数组
 * @param v2 lm后参数组
 * @return float
 */
float ReadFile::getDiffA(int index, vector<GaussParameter> &v1, vector<GaussParameter> &v2) {
    float ret = 0.0;
    if (index >= 0 && index <= 10) {
        ret = v2[index].A - v1[index].A;
    }
    return ret;
}

float ReadFile::getDiffU(int index, vector<GaussParameter> &v1, vector<GaussParameter> &v2) {
    float ret = 0.0;
    if (index >= 0 && index <= 10) {
        ret = v2[index].b - v1[index].b;
    }
    return ret;
}

float ReadFile::getDiffSigma(int index, vector<GaussParameter> &v1, vector<GaussParameter> &v2) {
    float ret = 0.0;
    if (index >= 0 && index <= 10) {
        ret = v2[index].sigma - v1[index].sigma;
    }
    return ret;
}

/**
 * 计算后向散射波形能量和
 * @param v 高斯分量组
 * @param start 后向散射起始索引
 * @param end 终止索引
 * @return float
 */
float ReadFile::getEnergy(vector<GaussParameter> &v, int start, int end) {
    float ans = 0.0;
    for (int i = 0; i < 320; ++i) {
        for (int j = start + 1; j < end; ++j) {
            ans += v.at(j).A * exp(-(i - v.at(j).b) * (i - v.at(j).b) /
                                   (2 * v.at(j).sigma * v.at(j).sigma));
        }
    }
    return ans;
}