//
// Created by �ս��� on 2019-07-14.
//

#ifndef PHOTONDATA_H
#define PHOTONDATA_H


#include "iostream"
#include "fstream"
#include <vector>
#include <bitset>

using namespace std;


//ԭʼ���ݽṹ��
class PhotonData {
public:
    PhotonData() {};

    ~PhotonData() {}


    bitset<1> controlFlag;
    bitset<6> channelMes;
    bitset<15> timeMes;
    bitset<10> pulseSeq;

    void getData(FILE *fp);


};


#endif //PHOTONDATA_H
