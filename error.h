#ifndef ERROR_H
#define ERROR_H

#include <stdio.h>
#include <stdint.h>

#include "core_cm0.h"

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name);

#endif
