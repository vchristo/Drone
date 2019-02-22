/*****************************************************************************
diag -- diagnostic library

This library implements a set of four functions which make it easy to
report and return errors using compact code.

The functions are:

   diag() -- emit a diagnostic message; arguments are like printf; always
             returns -1
   diagp() -- like diag() but always returns NULL
   sysdiag() -- takes an additional argument before the format string: the
             name of a function; produces additional output based on errno;
             always returns -1
   sysdiagp() -- like sysdiag() but always returns NULL

This allows you to replace code like:

   if((infile = fopen(fname, "r")) == NULL) {
      fprintf(stderr, "can't open file %s\n", fname);
      perror("fopen");
      return -1;
   }

with the equivalent but much cleaner:

   if((infile = fopen(fname, "r")) == NULL)
      return sysdiag("fopen", "can't open file %s", fname);

This is free software. See COPYING for details.
*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include "debug.h"
#include "diag.h"

/***
static void diagmsg(char *syscallname, char *fmt, va_list args) -- emit diag

Function used internally by the diag() family of functions. The first
argument is a pointer to a string containing the name of the system or
library function which failed (or NULL if this is an application-level
error). The second argument is a printf()-style format string. The
third argument is a list of additional args as required by the format.

If the syscallname (first argument) is non-NULL, an additional message is
printed containing the syscallname string and the text corresponding to the
current value of the global errno.
***/
static void diagmsg(const char *syscallname, const char *fmt, va_list args) {
   TSTA(fmt==NULL);

   vfprintf(stderr, fmt, args);
   fputc('\n', stderr);
   if(syscallname == NULL) return;
   fprintf(stderr, "%s(): %s\n", syscallname, strerror(errno));
}

/***
int diag(char *fmt, ...) -- emit diagnostic message

This function emits a diagnostic message. The arguments are as per
printf()(3). Always returns -1.
***/
int diag(const char *fmt, ...) {
   va_list args;

   TSTA(fmt==NULL);

   va_start(args,fmt);
   diagmsg(NULL, fmt, args);
   va_end(args);
   return -1;
}

void diag_cfg(FILE *f, char *fmt, ...) {
   va_list args;
   char str[1024];

   TSTA(fmt==NULL);

   va_start(args,fmt);
   vsnprintf(str, sizeof(str), fmt, args);
   va_end(args);

   fwrite(str, strlen(str), 1, f);
}

/***
void *diagp(char *fmt, ...) -- emit diagnostic message

Identical to diag(), above, but returns NULL instead of -1.
***/
void *diagp(char *fmt, ...) {
   va_list args;

   TSTA(fmt==NULL);

   va_start(args,fmt);
   diagmsg(NULL, fmt, args);
   va_end(args);
   return NULL;
}

/***
int sysdiag(char *syscallname, char *fmt, ...) -- emit diagnostic message

Emits a diagnostic message suitable for reporting the failure of a system
or library function which sets the global variable errno. The first argument
is a pointer to a string containing the name of the failed function. The
subsequent arguments are as per printf()(3). Always returns -1.
***/
int sysdiag(const char *syscallname, const char *fmt, ...) {
   va_list args;

   TSTA(syscallname==NULL); TSTA(fmt==NULL);

   va_start(args,fmt);
   diagmsg(syscallname, fmt, args);
   va_end(args);
   return -1;
}

/***
void *sysdiagp(char *syscallname, char *fmt, ...) -- emit diagnostic message

Identical to sysdiag(), above, but always returns NULL rather than -1.
***/
void *sysdiagp(char *syscallname, char *fmt, ...) {
   va_list args;

   TSTA(syscallname==NULL); TSTA(fmt==NULL);

   va_start(args,fmt);
   diagmsg(syscallname, fmt, args);
   va_end(args);
   return NULL;
}
