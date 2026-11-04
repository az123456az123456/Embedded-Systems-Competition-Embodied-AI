#include <stddef.h>

/* GCC may lower ordinary structure assignments to these standard routines,
 * even with -fno-builtin. Keep their implementation local to the freestanding
 * image: no libc, allocation, unaligned word access, or division helpers. */
void *memcpy(void *destination, const void *source, size_t length)
{
    unsigned char *out = destination;
    const unsigned char *in = source;
    size_t index;
    for (index = 0; index < length; ++index) out[index] = in[index];
    return destination;
}

void *memset(void *destination, int value, size_t length)
{
    unsigned char *out = destination;
    size_t index;
    for (index = 0; index < length; ++index) out[index] = (unsigned char)value;
    return destination;
}
