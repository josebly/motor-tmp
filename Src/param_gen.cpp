

// Save param to a file for uploading

// build with gcc param.c param_gen.cpp -lstdc++

#include "param.h"
#include <iostream>
#include <fstream>

int main() {
    init_param_from_flash();
    Param p = *param();
    p.fast_loop_param.foc_param.current_filter_frequency_hz = 5000;
    std::ofstream myfile;
    myfile.open ("param.bin");
    auto pos = myfile.tellp();
    myfile.write((char *) &p, sizeof(p));
    std::cout << "Wrote " << myfile.tellp()-pos << " bytes" << std::endl;
    myfile.close();

}