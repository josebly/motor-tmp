#ifndef OTP_H
#define OTP_H

// OTP (one time programmable) has 16 blocks of 32 bytes. Each block can be locked.
// Store serial number, hardware version in one block

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct BoardID {
#ifdef __cplusplus
    union BoardType {
        enum STBoardType {STUnknownBoardType=0, Nucleo446RE=1, Nucleo446ZE=2};
        enum FabulabBoardType {FabulabUnknownBoardType=0, DevBoard=1};
    };
    enum Manufacturer {UnknownManufacturer=0, ST=1, FabulabSL=2, Unprogrammed=0xFF};
    BoardID();
#endif

    uint8_t manufacturer;
    uint8_t board_type;
    uint8_t board_sub_type;
    struct {
        uint8_t major, minor, revision;
    } version;
    uint8_t serial_number[10];      // null terminated, pad with 0xFF
    uint8_t part_number[12];        // null terminated, pad with 0xFF
    uint8_t reserved[4];            // write 0xFF x 4
};

const struct BoardID * const get_board_id();

const char * board_id_serial_number();
const char * board_id_product_string();
const char * board_id_manufacturer_string();

#ifdef __cplusplus
}
#endif

#endif
