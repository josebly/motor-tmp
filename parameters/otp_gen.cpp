

// Save OTP to a file for uploading

// build with g++ otp_gen.cpp otp.cpp -o otp_gen

#include "otp.h"
#include <iostream>
#include <fstream>
#include <cstring>
#include "CLI11.hpp"

void print_board_id(BoardID * board_id);

int main(int argc, char** argv) {
    BoardID board_id_gen;
    std::memset(&board_id_gen, 0xFF, sizeof(board_id_gen));
    BoardIDWrapper board_id_wrapper = {&board_id_gen};

    CLI::App app{"OTP (one time programmable memory) binary generator"};
    std::string serial_number;
    std::string part_number = "MB1136 rC.3";
    board_id_gen.manufacturer = BoardID::ST;
    board_id_gen.board_type = BoardID::BoardType::Nucleo446RE;
    app.add_option("-s,--serial", serial_number, "Serial number")->required();
    app.add_option("-m,--manufacturer", board_id_gen.manufacturer, "Manufacturer", true);
    app.add_option("-b,--board_type", board_id_gen.board_type, "Board type", true);
    app.add_option("-p,--part_number", part_number, "Part number", true);
    CLI11_PARSE(app, argc, argv);
  
    board_id_gen.board_sub_type = 0;
    board_id_gen.version.major = 0;
    board_id_gen.version.minor = 0;
    board_id_gen.version.revision = 0;
    std::strncpy((char *) board_id_gen.part_number, part_number.c_str(), sizeof(board_id_gen.part_number));
    std::strncpy((char *) board_id_gen.serial_number, serial_number.c_str(), sizeof(board_id_gen.serial_number));
    // ensure null terminated
    board_id_gen.serial_number[sizeof(board_id_gen.serial_number)-1] = '\0';
    board_id_gen.part_number[sizeof(board_id_gen.part_number)-1] = '\0';

    std::ofstream myfile;
    myfile.open ("otp.bin");
    auto pos = myfile.tellp();
    myfile.write((char *) &board_id_gen, sizeof(board_id_gen));
    std::cout << "Wrote " << myfile.tellp()-pos << " bytes" << std::endl;
    myfile.close();

    print_board_id(&board_id_gen);
}

void print_board_id(BoardID * board_id) {
    BoardIDWrapper board_id_wrapper = {board_id};
    std::cout << "Manufacturer: " << board_id_wrapper.get_manufacturer_string() << std::endl;
    std::cout << "Product type: " << board_id_wrapper.get_product_string() << std::endl;
    std::cout << "Serial number: " << board_id_wrapper.get_serial_number() << std::endl;
    std::cout << "Part number: " << board_id_wrapper.get_part_number() << std::endl;
}