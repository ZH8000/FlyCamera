#include <fstream>
#include <iostream>
#include <string>

using namespace std;

int main(int argc, char** argv) {

	char c;
	//while (1) {
		//ifstream infile("gpio_in", ios::out);
		fstream afile("gpio_in", ios::out | ios::in);
		fstream bfile("gpio_out", ios::out | ios::in);
		for (string line; getline(afile, line);) {
			//if (line == "0") {
			//} else if (line == "1") {
			//	c = 's';
			//}
			cout << line << endl;
			if (line == "1") {
				if (bfile.is_open()) {
					bfile << "333" << endl;
					bfile.close();
					cout << "333" << endl;
				}				
			}
		}
	//}
	return 0;
}
