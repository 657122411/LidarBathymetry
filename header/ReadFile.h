#pragma once

#include <iostream>
#include "WaveData.h"
#include "DeepWave.h"
#include "TimeConvert.h"
#include <iomanip>

using namespace std;

class ReadFile {
public:
    ReadFile();

    ~ReadFile();

    bool setFilename(char filename[100]);

    void readBlueAll();

    void readGreenAll();

    void readMix();

    void outputData();

    void readDeep();

    void readDeepByRed();

    void readDeepOutLas();

    void dataAnalysis();

    bool inDuration(Time time);

    float calcuAvgDiff(vector<float> &v1, vector<float> &v2);

    float calcuVariance(vector<float> &v1, float avg);

private:
    char *m_filename;
    FILE *m_filePtr;
};