// myLidar.cpp : �������̨Ӧ�ó������ڵ㡣
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
            cout << "Channel?(0:Blue/1:Green/2:All/3:Mix/4:OutputData/5:ReadDeep/6:ReadDeepByRed/7:ReadDeepOutLas):"
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
                    myfile.readPhoton();
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