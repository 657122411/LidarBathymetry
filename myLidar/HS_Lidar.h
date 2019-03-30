#ifndef HS_Lidar_H
#define HS_Lidar_H

#include "HS_Lidar_Channel.h"
#include "HS_Lidar_Header.h"
#include "iostream"
#include "fstream"
#include <vector>
using namespace std;


//原始数据结构类
class HS_Lidar
{
public:

	HS_Lidar_Header header;							//帧头
	HS_Lidar_Channel CH1;							//通道一
	HS_Lidar_Channel CH2;							//通道二
	HS_Lidar_Channel CH3;							//通道三
	HS_Lidar_Channel CH4;							//通道四

	HS_Lidar();
	~HS_Lidar();

	void initData(FILE *fp);						//数据初始化
	void getHeader(FILE *fp);						//获取帧头
	void getChannel(FILE *fp, HS_Lidar_Channel &CH);//获取通道

	void initDeepData(FILE *fp);					//针对深水数据的初始化
	void getDeepChannel(FILE *fp, HS_Lidar_Channel &CH, vector<int> &deepData);//获得深水数据的通道数据

	vector<int> deepData1;							//通道一的二段回波
	vector<int> deepData2;							//通道二的二段回波
	vector<int> deepData3;							//通道三的二段回波
	vector<int> deepData4;							//通道四的二段回波	
};


#endif


