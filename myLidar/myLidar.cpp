// myLidar.cpp : 定义控制台应用程序的入口点。
//


#include "ReadFile.h"
using namespace std;

float Theta = 0;

int main()
{
	int flag = 1;
	while (flag)
	{
		cout << "file address:"<<endl;
		char name[100];
		cin >> name;
		ReadFile myfile;
		bool ret = myfile.setFilename(name);
		if (ret)
		{
			cout << "Channel?(0:Blue/1:Green/2:All/3:Mix/4:OutputData/5:ReadDeep/6:ReadDeepByRed/7:ReadDeepOutLas):" << endl;
			cin >> flag;
			switch (flag)
			{
			case 0: {
				myfile.readBlueAll();
				break;
			}
			case 1: {
				myfile.readGreenAll();
				break;
			}
			case 2: {
				myfile.readBlueAll();
				myfile.readGreenAll();
				break;
			}
			case 3: {
			    cout<<"Incidence angle?"<<endl;
			    cin>>Theta;
				myfile.readMix();
				break;
			}
			case 4: {
				myfile.outputData();
				break;
			}
			case 5: {
				myfile.readDeep();
				break;
			}
			case 6: {
				myfile.readDeepByRed();
				break;
			}
			case 7: {
				myfile.readDeepOutLas();
				break;
			}
            case 8: {
                myfile.readPhoton();
                break;
            }
			}
		}
		else
			continue;

		cout << "continue?(1:Y/0:N):" << endl;
		cin >> flag;
	}
	
	return 0;
}