#include "rpii2cslv.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <bcm_host.h>
#include <wiringPi.h>

#define print_error(fmt, ...) \
    do { \
        fprintf(stderr, "%s:%d (%s): error: " fmt, \
                __FILE__, __LINE__, __func__, ##__VA_ARGS__); \
    } while (0)


/* SPI/BSC(I2C) slave. p.160 */
#define BSC_OFFSET 0x00214000
#define BSC_SIZE 0x40
static void *bsc = NULL;

#define BSC_REG(offset) (((volatile uint32_t*) bsc)[(offset)>>2])
#define BSC_DR      BSC_REG(0x00)
#  define BSC_DR_DATA_SET     0x000000ff
#  define BSC_DR_DATA_LSB     0
#define BSC_RSR     BSC_REG(0x04)
#define BSC_SLV     BSC_REG(0x08)
#  define BSC_SLV_ADDR_SET    0x0000007f
#  define BSC_SLV_ADDR_LSB    0
#define BSC_CR      BSC_REG(0x0c)
#  define BSC_CR_RXE_SET      0x00000200
#  define BSC_CR_RXE_LSB      9
#  define BSC_CR_TXE_SET      0x00000100
#  define BSC_CR_TXE_LSB      8
#  define BSC_CR_I2C_SET      0x00000004
#  define BSC_CR_I2C_LSB      2
#  define BSC_CR_EN_SET       0x00000001
#  define BSC_CR_EN_LSB       0
#define BSC_FR      BSC_REG(0x10)
#  define BSC_FR_RXFLEVEL_SET 0x0000f800
#  define BSC_FR_RXFLEVEL_LSB 11
#  define BSC_FR_TXFLEVEL_SET 0x000007c0
#  define BSC_FR_TXFLEVEL_LSB 6
#  define BSC_FR_TXFF_SET     0x00000004
#  define BSC_FR_TXFF_LSB     2
#  define BSC_FR_RXFE_SET     0x00000002
#  define BSC_FR_RXFE_LSB     1
#define BSC_IFLS    BSC_REG(0x14)
#define BSC_IMSC    BSC_REG(0x18)
#define BSC_RIS     BSC_REG(0x1c)
#define BSC_MIS     BSC_REG(0x20)
#define BSC_ICR     BSC_REG(0x24)
#define BSC_DMACR   BSC_REG(0x28)
#define BSC_TDR     BSC_REG(0x2c)
#define BSC_GPUSTAT BSC_REG(0x30)
#define BSC_HCTRL   BSC_REG(0x34)
#define BSC_DEBUG1  BSC_REG(0x38)
#define BSC_DEBUG2  BSC_REG(0x3c)


static int open_bsc(void)
{
    int fd;
    uint32_t peri_addr;
    int err;

    bcm_host_init();
    peri_addr = bcm_host_get_peripheral_address();
    bcm_host_deinit();

    fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) {
        print_error("open: /dev/mem: %s\n", strerror(errno));
        return 1;
    }

    bsc = mmap(0, BSC_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
            peri_addr + BSC_OFFSET);
    if (bsc == MAP_FAILED) {
        print_error("mmap: %s\n", strerror(errno));
        (void) close(fd);
        return 1;
    }

    err = close(fd);
    if (err) {
        print_error("close: %s\n", strerror(errno));
        (void) munmap(bsc, BSC_SIZE);
        return 1;
    }

    return 0;
}

static int close_bsc(void)
{
    int err;

    err = munmap(bsc, BSC_SIZE);
    if (err) {
        print_error("munmap: %s\n", strerror(errno));
        return 1;
    }

    return 0;
}

int i2cslv_init(const uint8_t addr)
{
    int err;

    if (open_bsc())
        return 1;

    BSC_CR = 0;     /* Disable SPI/BSC. */
    BSC_RSR = 0;    /* Clear errors. */
    BSC_IMSC = 0xf; /* Clear interrupt masks. */
    BSC_ICR = 0xf;  /* Clear interrupts. */

    /* Set slave address. */
    BSC_SLV = (addr << BSC_SLV_ADDR_LSB) & BSC_SLV_ADDR_SET;
    /* Enable BSC(I2C). */
    BSC_CR = BSC_CR_EN_SET | BSC_CR_I2C_SET | BSC_CR_TXE_SET | BSC_CR_RXE_SET;
    BSC_RSR = 0;    /* Clear errors. */

    err = wiringPiSetupGpio();
    if (err) {
        print_error("wiringPiSetupGpio: %d\n", err);
        return 1;
    }
    /* 0b111 means ALT3. */
    pinModeAlt(18, 0b111); /* BSCSL_SDA. p.102 */
    pinModeAlt(19, 0b111); /* BSCSL_SCL. p.102 */

    return 0;
}

int i2cslv_finalize(void)
{
    BSC_SLV = 0; /* Clear slave address. */
    BSC_CR = 0;  /* Disable SPI/BSC. */
    BSC_RSR = 0; /* Clear errors. */

    if (close_bsc())
        return 1;

    return 0;
}


uint8_t i2cslv_available(void)
{
    return (BSC_FR & BSC_FR_RXFLEVEL_SET) >> BSC_FR_RXFLEVEL_LSB;
}

uint8_t i2cslv_queued(void)
{
    return (BSC_FR & BSC_FR_TXFLEVEL_SET) >> BSC_FR_TXFLEVEL_LSB;
}

void i2cslv_read(uint8_t *buf, size_t count)
{
    while (count-- > 0) {
        while (BSC_FR & BSC_FR_RXFE_SET)
            ;
        *buf++ = (BSC_DR & BSC_DR_DATA_SET) >> BSC_DR_DATA_LSB;
    }
}

void i2cslv_write(const uint8_t *buf, size_t count)
{
    while (count-- > 0) {
        while (BSC_FR & BSC_FR_TXFF_SET)
            ;
        BSC_DR = ((*buf++) << BSC_DR_DATA_LSB) & BSC_DR_DATA_SET;
    }
}
