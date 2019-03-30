

// Save OTP to a file for uploading

// build with gcc otp_gen.cpp -lstdc++

#include "otp.h"
#include <iostream>
#include <fstream>
#include <cstring>

int main() {
    BoardID board_id_gen;
    std::memset(&board_id_gen, 0xFF, sizeof(board_id_gen));

    board_id_gen.manufacturer = BoardID::ST;
    board_id_gen.board_type = BoardID::Nucleo;
    board_id_gen.board_sub_type = 0;
    board_id_gen.version.major = 11;
    board_id_gen.version.minor = 36;
    board_id_gen.version.revision = 'c';
    std::strncpy((char *) &board_id_gen.sn, "123456789", sizeof(board_id_gen.sn));


    std::ofstream myfile;
    myfile.open ("otp.bin");
    auto pos = myfile.tellp();
    myfile.write((char *) &board_id_gen, sizeof(board_id_gen));
    std::cout << "Wrote " << myfile.tellp()-pos << " bytes" << std::endl;
    myfile.close();
}