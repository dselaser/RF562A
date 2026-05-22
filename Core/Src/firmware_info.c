/*===========================================================================*\
 *  firmware_info.c - RF562A Application Firmware ID
 *  Placed at fixed Flash offset 0x200 via linker section .fw_info
\*===========================================================================*/
#include "firmware_info.h"

/* Auto build date from __DATE__: "Mar 28 2026" -> 20260328 */
#define _MON (__DATE__[0]=='J'?(__DATE__[1]=='a'?1:(__DATE__[2]=='n'?6:7)): \
              __DATE__[0]=='F'?2: \
              __DATE__[0]=='M'?(__DATE__[2]=='r'?3:5): \
              __DATE__[0]=='A'?(__DATE__[1]=='p'?4:8): \
              __DATE__[0]=='S'?9:__DATE__[0]=='O'?10:__DATE__[0]=='N'?11:12)
#define _DAY ((__DATE__[4]==' '?0:(__DATE__[4]-'0'))*10+(__DATE__[5]-'0'))
#define _YR  ((__DATE__[7]-'0')*1000+(__DATE__[8]-'0')*100+ \
              (__DATE__[9]-'0')*10+(__DATE__[10]-'0'))
#define _BUILD_DATE (_YR*10000+_MON*100+_DAY)

__attribute__((section(".fw_info"), used))
const FirmwareInfo_t firmware_info = {
    .magic        = FWINFO_MAGIC,
    .target_mcu   = FWINFO_MCU_H562,
    .board_type   = FWINFO_TYPE_APP,
    .fw_version   = FWINFO_VERSION(1, 1, 0),
    .project_name = "RF562A",
    .build_date   = _BUILD_DATE,
};
