#include <rpii2cslv.h>
#include <stdio.h>

int main(void)
{
    if (i2cslv_init(0x22))
        return 1;

    printf("avail: %zu\n", i2cslv_available());
    printf("queued: %zu\n", i2cslv_queued());

    if (i2cslv_finalize())
        return 1;
    return 0;
}
