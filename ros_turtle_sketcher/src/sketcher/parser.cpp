#include <vector>
#include <fstream>
#include <string>
#include <exception>
#include <iostream>

using namespace std;


void print_state (const std::ifstream& stream) {
  std::cerr << " good()=" << stream.good();
  std::cerr << " eof()=" << stream.eof();
  std::cerr << " fail()=" << stream.fail();
  std::cerr << " bad()=" << stream.bad() << endl;;
}

/*
 * parseFile
 * Parses point file: two floating point numbers per line, ASCII, separated by a space.
 * @param fileName Filename to parse.
 * @param points Vector to fill
 */
bool parseFile(string fileName, vector<double>& points)
{
	cout << "Parsing file: " << fileName << endl;
	ifstream in;
	in.open(fileName, ifstream::in);

	if (!in.good()) {
		cerr << "File not OK" << endl;
		print_state(in);
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
