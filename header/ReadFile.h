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

    float calcuAvgAbsDiff(vector<float> &v1, vector<float> &v2);

    float calcuVariance(vector<float> &v1, vector<float> &v2, float avg);

    float calcuAbsVariance(vector<float> &v1, vector<float> &v2, float avg);

    float getDiffA(int index, vector<GaussParameter> &v1, vector<GaussParameter> &v2);

    float getDiffU(int index, vector<GaussParameter> &v1, vector<GaussParameter> &v2);

    float getDiffSigma(int index, vector<GaussParameter> &v1, vector<GaussParameter> &v2);

    float getEnergy(vector<GaussParameter> &v, int start, int end);

private:
    char *m_filename;
    FILE *m_filePtr;
};