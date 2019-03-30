#include "otp.h"

//const volatile struct BoardID __attribute__ ((section ("otp"))) board_id = {};
const BoardID * const board_id = (const BoardID * const) 0x08060000; //0x1FFF7800;
