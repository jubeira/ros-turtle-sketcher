#include <vector>
#include <fstream>
#include <string>
#include <exception>

using namespace std;

bool parseFile(string fileName, vector<double>& points)
{
	ifstream in(fileName);

	if (!in.good()) {
		return false;
	}

	vector<string> lines;

	for (string line; getline(in, line);) {
		lines.push_back(line);
	}

	string::size_type sz;

	try {
		for (auto a : lines) {
			points.push_back(stod(a, &sz));
			points.push_back(stod(a.substr(sz)));
		}
	}
	catch (exception e){
		return false;
	}

	return true;
}
