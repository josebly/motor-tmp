#include "otp.h"

static const BoardID default_board_id;

static const char * ProductStrings[] = {
    "Unknown",
    "Nucleo F446RE"
};

static const char * ManufacturerStrings[] = {
    "Unknown",
    "ST"
};

BoardID::BoardID() : serial_number("0") {}

const BoardID * const get_board_id() {
    const BoardID * board_id = (const BoardID * const) 0x1FFF7800;
    if (board_id->manufacturer == 0xFF) {
        board_id = &default_board_id;
    }
    return board_id;
}

const char * board_id_serial_number() {
    return reinterpret_cast<const char *>(get_board_id()->serial_number);
}

const char * board_id_product_string() {
    uint8_t product_type = 0;
    if (get_board_id()->board_type <= sizeof(ProductStrings)) {
        product_type = get_board_id()->board_type;
    }
    return ProductStrings[product_type];
}

const char * board_id_manufacturer_string() {
    uint8_t manufacturer = 0;
    if (get_board_id()->manufacturer <= sizeof(ManufacturerStrings)) {
        manufacturer = get_board_id()->manufacturer;
    }
    return ManufacturerStrings[manufacturer];
}