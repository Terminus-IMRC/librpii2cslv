#ifndef RPII2CSLV_H_
#define RPII2CSLV_H_

#include <stdint.h>
#include <sys/types.h>

    int i2cslv_init(const uint8_t addr);
    int i2cslv_finalize(void);

    uint8_t i2cslv_available(void);
    uint8_t i2cslv_queued(void);
    void i2cslv_read(uint8_t *buf, size_t count);
    void i2cslv_write(const uint8_t *buf, size_t count);

#endif /* RPII2CSLV_H_ */
