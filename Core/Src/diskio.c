#include "ff.h"
#include "diskio.h"


extern int sd_init(void);
extern int sd_read_block(uint32_t block, uint8_t *buf);
extern int sd_write_block(uint32_t block, const uint8_t *buf);

extern void sendStr(char *str);
extern void sendHex(uint8_t b);

static DSTATUS status = STA_NOINIT;   // "card not started yet"

DSTATUS disk_initialize(BYTE pdrv) {
    if (pdrv != 0) return STA_NOINIT;               // only one card, number 0
    status = (sd_init() == 0) ? 0 : STA_NOINIT;
    return status;
}

DSTATUS disk_status(BYTE pdrv) {
    return (pdrv == 0) ? status : STA_NOINIT;
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count) {
    if (pdrv != 0 || status) return RES_NOTRDY;       // keep this

    for (UINT i = 0; i < count; i++) {
        uint8_t e = sd_read_block(sector + i, buff + 512 * i);
        if (e != 0) {
            uint32_t b = sector + i;
            sendStr("read FAIL block 0x");
            sendHex(b >> 24); sendHex(b >> 16); sendHex(b >> 8); sendHex(b);
            sendStr(" err "); sendHex(e); sendStr("\r\n");
            return RES_ERROR;
        }
    }
    return RES_OK;                                      // keep this
}

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count) {
    if (pdrv != 0 || status) return RES_NOTRDY;
    for (UINT i = 0; i < count; i++)
        if (sd_write_block(sector + i, buff + 512 * i) != 0) return RES_ERROR;
    return RES_OK;
}

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff) {
    (void)buff;
    if (pdrv != 0 || status) return RES_NOTRDY;
    if (cmd == CTRL_SYNC) return RES_OK;   // writes are finished when they return
    return RES_PARERR;                     // other commands are only for formatting
}
