#ifndef TIMECONVERT_H
#define TIMECONVERT_H

//UTC时间结构体
typedef struct Time {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
} UTCTIME;

//通用时
typedef struct tagCOMMONTIME {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    double second;
} COMMONTIME;

typedef COMMONTIME *PCOMMONTIME;

typedef struct tagTOD {
    long sn;  //秒数的整数部分
    double tos;//秒数的小数部分
} TOD;

typedef TOD *PTOD;

typedef struct {
    long day; //整数天数
    TOD tod;  //一天内的秒数
} JULIANDAY;//儒略日

typedef JULIANDAY *PJULIANDAY;

typedef struct tagMJULIANDAY {
    long day;
    TOD tod;
} MJULIANDAY;//新儒略日

typedef MJULIANDAY *PMJIANDAY;

typedef struct tagTOW {
    long sn;//秒整数部分
    double tos;//秒小数部分
} TOW;

typedef TOW *PTOW;

typedef struct tagGPSTIME {
    int wn; //周数
    TOW tow;//一周内的秒数
} GPSTIME;//GPS时

typedef GPSTIME *PGPSTIME;

typedef struct tagDOY {
    unsigned short year;
    unsigned short day;
    TOD tod;
} DOY;//年积日

typedef DOY *PDOY;

double FRAC(double morigin);// 取小数部分


void CommonTimeToJulianDay(PCOMMONTIME pct, PJULIANDAY pjd);//通用时到儒略日的转换

void JulianDayToCommonTime(PJULIANDAY pjd, PCOMMONTIME pct);//儒略日到通用时的转换

void JulianDayToGPSTime(PJULIANDAY pjd, PGPSTIME pgt);//儒略日到GPS时的转换

void GPSTimeToJulianDay(PGPSTIME pgt, PJULIANDAY pjd);//GPS时到儒略日的转换

void CommonTimeToGPSTime(PCOMMONTIME pct, PGPSTIME pgt);//通用时到GPS时的转换

void GPSTimeToCommonTime(PGPSTIME pgt, PCOMMONTIME pct);//GPS时到通用时的转换

void CommonTimeToDOY(PCOMMONTIME pct, PDOY pdoy);//通用时到年积日

void DOYToCommonTime(PDOY pdoy, PCOMMONTIME pct);//年积日到通用时

void GPSTimeToDOY(PGPSTIME pgt, PDOY pdoy);//GPS时到年积日

void DOYToGPSTime(PDOY pdoy, PGPSTIME pgt);//年积日到GPS时

void JulianDayToDOY(PJULIANDAY pjd, PDOY pdoy);//儒略日到年积日

void DOYToJulianDay(PDOY pdoy, PJULIANDAY pjd);//年积日到儒略日


#endif