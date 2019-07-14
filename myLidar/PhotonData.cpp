//
// Created by Ã’Ω£∫∆ on 2019-07-14.
//

#include "PhotonData.h"

void PhotonData::getData(FILE *fp){
    fread(&controlFlag, sizeof(bitset<1>), 1, fp);
    fread(&channelMes, sizeof(bitset<6>), 1, fp);
    fread(&timeMes, sizeof(bitset<15>), 1, fp);
    fread(&pulseSeq, sizeof(bitset<10>), 1, fp);

}
