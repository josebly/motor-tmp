#include "otp.h"

static const BoardID default_board_id;
#ifdef STM32F446xx
static BoardIDWrapper board_id_wrapper = {(const BoardID * const) 0x1FFF7800};
#else
static BoardIDWrapper board_id_wrapper = {&default_board_id};
#endif

BoardID::BoardID() : serial_number("0") {}

static const char * ProductStrings[][4] = {
    {
        "Unknown",
    },
    {
        "Unknown ST",
        "Nucleo F446RE",
        "Nucleo F446ZE",
    },
    {
        "Unknown Fabulab",
        "dev_00",
    },
    {
        "Unknown Ve",
        "Rev0",
    },
};

static const char * ManufacturerStrings[] = {
    "Unknown",
    "ST",
    "FabulabSL",
    "Ve",
};



const BoardID * const get_board_id() {
    return board_id_wrapper.get_board_id();
}

const char * board_id_serial_number() {
    return board_id_wrapper.get_serial_number();
}

//TODO make safe for bad values
const char * board_id_product_string() {
    return board_id_wrapper.get_product_string();
}

//TODO make safe for bad values
const char * board_id_manufacturer_string() {
    return board_id_wrapper.get_manufacturer_string();
}

BoardIDWrapper::BoardIDWrapper(const BoardID * const board_id) 
    : board_id_(board_id) {
    // TODO check bad values here maybe
    if (board_id_->manufacturer == 0xFF) {
        board_id_ = &default_board_id;
    }
}

const BoardID * const BoardIDWrapper::get_board_id() const {
    return board_id_;
}

const char * BoardIDWrapper::get_serial_number() const {
    return reinterpret_cast<const char *>(board_id_->serial_number);
}

const char * BoardIDWrapper::get_manufacturer_string() const {
    uint8_t manufacturer = 0;
    if (board_id_->manufacturer <= sizeof(ManufacturerStrings)) {
        manufacturer = board_id_->manufacturer;
    }
    return ManufacturerStrings[manufacturer];
}

const char * BoardIDWrapper::get_product_string() const {
    uint8_t product_type = 0;
    if (board_id_->board_type <= sizeof(ProductStrings)) {
        product_type = board_id_->board_type;
    }
    return ProductStrings[board_id_->manufacturer][product_type];
}

const char * BoardIDWrapper::get_part_number() const {
    return reinterpret_cast<const char *>(board_id_->part_number);
}