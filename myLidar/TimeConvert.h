#ifndef TIMECONVERT_H
#define TIMECONVERT_H

//UTCʱ��ṹ��
typedef struct Time
{
	int year;
	int month;
	int day;
	int hour;
	int minute;
	int second;
};

//ͨ��ʱ
typedef struct tagCOMMONTIME
{
	int   year;
	int   month;
	int   day;
	int   hour;
	int   minute;
	double   second;
}COMMONTIME;

typedef COMMONTIME *PCOMMONTIME;

typedef struct tagTOD
{
	long sn;  //��������������
	double tos;//������С������
}TOD;

typedef TOD *PTOD;

typedef struct
{
	long day; //��������
	TOD tod;  //һ���ڵ�����
}JULIANDAY;//������

typedef JULIANDAY *PJULIANDAY;

typedef struct tagMJULIANDAY
{
	long day;
	TOD  tod;
}MJULIANDAY;//��������

typedef MJULIANDAY *PMJIANDAY;

typedef struct tagTOW
{
	long sn;//����������
	double tos;//��С������
}TOW;

typedef TOW *PTOW;

typedef struct tagGPSTIME
{
	int wn; //����
	TOW tow;//һ���ڵ�����
}GPSTIME;//GPSʱ

typedef GPSTIME *PGPSTIME;

typedef struct tagDOY
{
	unsigned short year;
	unsigned short day;
	TOD tod;
}DOY;//�����

typedef DOY *PDOY;

double FRAC(double morigin);// ȡС������


void CommonTimeToJulianDay(PCOMMONTIME pct, PJULIANDAY pjd);//ͨ��ʱ�������յ�ת��

void JulianDayToCommonTime(PJULIANDAY pjd, PCOMMONTIME pct);//�����յ�ͨ��ʱ��ת��

void JulianDayToGPSTime(PJULIANDAY pjd, PGPSTIME pgt);//�����յ�GPSʱ��ת��

void GPSTimeToJulianDay(PGPSTIME pgt, PJULIANDAY pjd);//GPSʱ�������յ�ת��

void CommonTimeToGPSTime(PCOMMONTIME pct, PGPSTIME pgt);//ͨ��ʱ��GPSʱ��ת��

void GPSTimeToCommonTime(PGPSTIME pgt, PCOMMONTIME pct);//GPSʱ��ͨ��ʱ��ת��

void CommonTimeToDOY(PCOMMONTIME pct, PDOY pdoy);//ͨ��ʱ�������

void DOYToCommonTime(PDOY pdoy, PCOMMONTIME pct);//����յ�ͨ��ʱ

void GPSTimeToDOY(PGPSTIME pgt, PDOY pdoy);//GPSʱ�������

void DOYToGPSTime(PDOY pdoy, PGPSTIME pgt);//����յ�GPSʱ

void JulianDayToDOY(PJULIANDAY pjd, PDOY pdoy);//�����յ������

void DOYToJulianDay(PDOY pdoy, PJULIANDAY pjd);//����յ�������


#endif