/*===========================================================================*\
 *  firmware_info.h - Firmware Identification Header
 *
 *  Fixed at offset 0x200 from Flash start in every project.
 *  Bootloader reads this struct to verify the target MCU & board type
 *  before jumping to application or flashing firmware.
 *
 *  Copyright (c) 2024, connexthings@naver.com
\*===========================================================================*/
#ifndef FIRMWARE_INFO_H
#define FIRMWARE_INFO_H

#include <stdint.h>

/*---------------------------------------------------------------------------*\
 *  Magic Number: "FWID" in little-endian = 0x44495746
\*---------------------------------------------------------------------------*/
#define FWINFO_MAGIC            0x44495746U   /* 'F','W','I','D' */

/*---------------------------------------------------------------------------*\
 *  Target MCU IDs
\*---------------------------------------------------------------------------*/
#define FWINFO_MCU_H562         562U
#define FWINFO_MCU_F407         407U

/*---------------------------------------------------------------------------*\
 *  Board Type IDs
\*---------------------------------------------------------------------------*/
#define FWINFO_TYPE_APP         0U
#define FWINFO_TYPE_BOOT        1U

/*---------------------------------------------------------------------------*\
 *  Version Encoding: MAJOR.MINOR.PATCH -> 0x00MMNNPP
 *  Example: v1.2.3 -> 0x00010203
\*---------------------------------------------------------------------------*/
#define FWINFO_VERSION(major, minor, patch) \
    (((uint32_t)(major) << 16) | ((uint32_t)(minor) << 8) | (uint32_t)(patch))

#define FWINFO_VER_MAJOR(v)     (((v) >> 16) & 0xFF)
#define FWINFO_VER_MINOR(v)     (((v) >>  8) & 0xFF)
#define FWINFO_VER_PATCH(v)     ( (v)        & 0xFF)

/*---------------------------------------------------------------------------*\
 *  Firmware Info Structure (32 bytes, fixed layout)
\*---------------------------------------------------------------------------*/
typedef struct __attribute__((packed)) {
    uint32_t magic;             /* 0x00: FWINFO_MAGIC                      */
    uint16_t target_mcu;        /* 0x04: FWINFO_MCU_H562 or FWINFO_MCU_F407 */
    uint16_t board_type;        /* 0x06: FWINFO_TYPE_APP or FWINFO_TYPE_BOOT */
    uint32_t fw_version;        /* 0x08: FWINFO_VERSION(major,minor,patch) */
    char     project_name[16];  /* 0x0C: e.g. "RF562", "RF562_BOOT", etc.  */
    uint32_t build_date;        /* 0x1C: YYYYMMDD decimal                  */
} FirmwareInfo_t;               /* Total: 32 bytes                         */

/*---------------------------------------------------------------------------*\
 *  External declaration (defined in firmware_info.c per project)
\*---------------------------------------------------------------------------*/
extern const FirmwareInfo_t firmware_info;

/*---------------------------------------------------------------------------*\
 *  Validation helper
\*---------------------------------------------------------------------------*/
static inline int FirmwareInfo_IsValid(const FirmwareInfo_t *info)
{
    return (info != (void *)0) && (info->magic == FWINFO_MAGIC);
}

static inline int FirmwareInfo_MatchTarget(const FirmwareInfo_t *info,
                                           uint16_t mcu, uint16_t type)
{
    return FirmwareInfo_IsValid(info)
        && (info->target_mcu == mcu)
        && (info->board_type == type);
}

#endif /* FIRMWARE_INFO_H */
