// dry_lib.c -- freestanding libc subset for Dhrystone (t06).
//
// strcpy/strcmp are INSIDE the measured loop (ground rule: real library
// procedure calls) -> out-of-line, noinline, byte-at-a-time like a classic
// small libc. memcpy/memset back clang's struct-assignment / memset
// expansion. No RV32M or vector tricks: measure the core, not the libc.
#include "dry.h"

__attribute__((noinline))
char *strcpy(char *dest, const char *src)
{
    char *d = dest;
    while ((*d++ = *src++) != NUL)
        ;
    return dest;
}

__attribute__((noinline))
int strcmp(const char *a, const char *b)
{
    /* char is unsigned on RV32 clang; Dhrystone uses ASCII only, so the
       canonical signed-char sign convention cannot diverge here */
    while (*a && *a == *b) { a++; b++; }
    return (int)(unsigned char)*a - (int)(unsigned char)*b;
}

__attribute__((noinline))
void *memcpy(void *dest, const void *src, unsigned long n)
{
    unsigned char *d = (unsigned char *)dest;
    const unsigned char *s = (const unsigned char *)src;
    while (n--)
        *d++ = *s++;
    return dest;
}

__attribute__((noinline))
void *memset(void *dest, int c, unsigned long n)
{
    unsigned char *d = (unsigned char *)dest;
    while (n--)
        *d++ = (unsigned char)c;
    return dest;
}
