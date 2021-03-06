// myLidar.cpp : 定义控制台应用程序的入口点。
//


#include "../header/ReadFile.h"

using namespace std;

float Theta = 0;

int main() {
    int flag = 1;
    while (flag) {
        cout << "file address:" << endl;
        char name[100];
        cin >> name;
        ReadFile myfile;
        bool ret = myfile.setFilename(name);
        if (ret) {
            cout
                    << "Channel?\n0:Blue/\n1:Green/\n2:All/\n3:Mix/\n4:OutputData/\n5:ReadDeep/\n6:ReadDeepByRed/\n7:ReadDeepOutLas/\n8:DataAnalysis/\n"
                    << endl;
            cin >> flag;
            switch (flag) {
                case 0: {
                    cout << "Incidence angle?" << endl;
                    cin >> Theta;
                    myfile.readBlueAll();
                    break;
                }
                case 1: {
                    cout << "Incidence angle?" << endl;
                    cin >> Theta;
                    myfile.readGreenAll();
                    break;
                }
                case 2: {
                    cout << "Incidence angle?" << endl;
                    cin >> Theta;
                    myfile.readBlueAll();
                    myfile.readGreenAll();
                    break;
                }
                case 3: {
                    cout << "Incidence angle?" << endl;
                    cin >> Theta;
                    myfile.readMix();
                    break;
                }
                case 4: {
                    cout << "Incidence angle?" << endl;
                    cin >> Theta;
                    myfile.outputData();
                    break;
                }
                case 5: {
                    cout << "Incidence angle?" << endl;
                    cin >> Theta;
                    myfile.readDeep();
                    break;
                }
                case 6: {
                    cout << "Incidence angle?" << endl;
                    cin >> Theta;
                    myfile.readDeepByRed();
                    break;
                }
                case 7: {
                    cout << "Incidence angle?" << endl;
                    cin >> Theta;
                    myfile.readDeepOutLas();
                    break;
                }
                case 8: {
                    cout << "Incidence angle?" << endl;
                    cin >> Theta;
                    myfile.dataAnalysis();
                    break;
                }
            }
        } else
            continue;

        cout << "continue?(1:Y/0:N):" << endl;
        cin >> flag;
    }

    return 0;
}