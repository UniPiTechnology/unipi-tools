//============================================================================
// Name        : EEPROMDetectApp.cpp
// Author      : Tomas Knot
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

/*
 * EEPROMDetectApp.c
 *
 *  Created on: 20 Dec 2017
 *      Author: Tom Knot <knot@faster.cz>
 */
#include <iostream>
#include <fstream>
#include <iterator>
#include <algorithm>
#include <vector>
using namespace std;

const char* eeprom_unipi_path = "/sys/class/i2c-dev/i2c-1/device/1-0050/eeprom";
const char* eeprom_neuron_path = "/sys/class/i2c-dev/i2c-1/device/1-0057/eeprom";


bool check_existence(const char* path) {
	ifstream fs(path, ios::binary);
	return fs.good();
}

bool read_eeprom(const char* path, vector<char>* outp, int* outp_len)
{
    ifstream fs(path, ios::binary);
    if (fs.good() == false) {
    	return false;
    }
    fs.unsetf(ios::skipws);
    streampos size;
    fs.seekg(0, ios::end);
    size = fs.tellg();
    fs.seekg(0, ios::beg);
    *outp_len = (int)size;
    outp->reserve(size);
    outp->insert(outp->begin(), istream_iterator<char>(fs), istream_iterator<char>());
    return true;
}


int main() {
	int ret = 255;
	vector<char> *cont = new vector<char>;
	int *outp_len = new int;
	if (check_existence(eeprom_unipi_path)) {
		if (read_eeprom(eeprom_unipi_path, cont, outp_len)) {
			if (cont->at(226) == 1) {
				cout << "UNIPI 1.1" << endl;
				ret = 2;
			} else if (cont->at(226) == 11) {
				cout << "UNIPI LITE" << endl;
				ret = 3;
			} else {
				cout << "UNIPI 1.0" << endl;
				ret = 1;
			}
		} else {
			cout << "Invalid UNIPI 1.1 EEPROM detected!!!" << endl;
		}
	} else if (check_existence(eeprom_neuron_path)){
		if (read_eeprom(eeprom_neuron_path, cont, outp_len)) {
			ret = 4;
			cout << "NEURON " << cont->at(106) << cont->at(107) << cont->at(108) << cont->at(109) << endl;
		} else {
			cout << "Invalid NEURON EEPROM detected!!!" << endl;
		}
	} else {
		cout << "No EEPROM detected!!!" << endl;
	}

	delete cont;
	delete outp_len;
	return ret;
}
