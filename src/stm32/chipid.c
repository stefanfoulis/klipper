// Support for extracting the hardware chip id on stm32
//
// Copyright (C) 2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "generic/usb_cdc.h" // usb_fill_serial
#include "generic/usbstd.h" // usb_string_descriptor
#include "internal.h" // UID_BASE
#include "sched.h" // DECL_INIT

#define CHIP_UID_LEN 12

static struct {
    struct usb_string_descriptor desc;
    uint16_t data[CHIP_UID_LEN * 2];
} cdc_string_serial_chipid;

struct usb_string_descriptor *
usbserial_get_serialid(void)
{
   return &cdc_string_serial_chipid.desc;
}

void
chipid_init(void)
{
    if (!CONFIG_USB_SERIAL_NUMBER_CHIPID)
        return;

    uint8_t *chipid = (void*)UID_BASE;
    usb_fill_serial(&cdc_string_serial_chipid.desc, chipid, CHIP_UID_LEN * 2);
}
DECL_INIT(chipid_init);
