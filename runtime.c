#include <stddef.h>
/* No stdio, heap, constructors or syscalls in this firmware. */
void __libc_init_array(void) {}
void *memset(void *p, int c, size_t n) { unsigned char *d = p; while (n--) *d++ = (unsigned char)c; return p; }
void *memcpy(void *d, const void *s, size_t n) { unsigned char *a = d; const unsigned char *b = s; while (n--) *a++ = *b++; return d; }