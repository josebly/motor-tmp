#include "otp.h"

const volatile struct BoardID __attribute__ ((section ("otp"))) board_id = {};

