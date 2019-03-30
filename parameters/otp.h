#ifndef OTP_H
#define OTP_H

// OTP (one time programmable) has 16 blocks of 32 bytes. Each block can be locked.
// Store serial number, hardware version in one block

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct BoardID {
    enum BoardType {Nucleo=0};
    enum Manufacturer {ST=0};

    uint8_t manufacturer;
    uint8_t board_type;
    uint8_t board_sub_type;
    struct {
        uint8_t major, minor, revision;
    } version;
    uint8_t sn[10];
    uint8_t resv[16];
};

//extern const volatile struct BoardId board_id;

#ifdef __cplusplus
}
#endif

#endif
