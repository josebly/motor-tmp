

// Save param to a file for uploading

// build with gcc param.c ../parameters/param_ec16.c param_gen.cpp -lstdc++

#include "param.h"
#include <iostream>
#include <fstream>
#include <cstring>

int main() {
    init_param_from_flash();
    float table[COGGING_TABLE_SIZE] = {
#include "../cogprocessed.csv"
    };
    Param p = *param();
    p.fast_loop_param.foc_param.current_filter_frequency_hz = 5000;
    char name[64] = "J2";
    std::strncpy(p.name, name, 64);
    for(int i=0; i < COGGING_TABLE_SIZE; i++) {
        p.fast_loop_param.cogging.table[i] = table[i];
    }
    std::ofstream myfile;
    myfile.open ("param.bin");
    auto pos = myfile.tellp();
    myfile.write((char *) &p, sizeof(p));
    std::cout << "Wrote " << myfile.tellp()-pos << " bytes" << std::endl;
    myfile.close();

}