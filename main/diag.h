#ifndef DIAG_H
#define DIAG_H

#include <stdarg.h>

int diag(const char *fmt, ...) __attribute__ ((format(printf,1,2)));
void diag_cfg(FILE *f, char *fmt, ...);
void *diagp(char *fmt, ...) __attribute__ ((format(printf,1,2)));
int sysdiag(const char *syscallname, const char *fmt, ...)
 __attribute__ ((format(printf,2,3)));
void *sysdiagp(char *syscallname, char *fmt, ...)
 __attribute__ ((format(printf,2,3)));

#endif /* ifndef DIAG_H */
