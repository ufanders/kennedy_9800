#pragma once
#include <stdbool.h>
#include "esp_err.h"
esp_err_t usb_msc_init(void);
void      disk_io_set_ready(bool ready);   /* forward declared here for usb_msc.c use */
