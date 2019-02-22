/* This is free software. See COPYING for details. */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <dirent.h>
#include <linux/i2c-dev.h>
#include "diag.h"
#include "debug.h"
#include "i2c_ops.h"

#define DEVDIR "/dev"

/*** try_dev() -- check if desired I2C device is on a given bus

Given an I2C device file (/dev/i2c-x), this function attempts to
determine if a specific I2C device is present on the associated bus.

Arguments:
   fname -- the path to the device-special file associated with the
      I2C bus to be tested
   addr -- the (unshifted) address of the I2C device we're looking for
   devname -- pointer to a nul-terminated string containing the human-
      readable name of the device (typically a part number like "DS1307")
   callback -- a callback function which should return 0 IFF the
      desired device is present

The callback function is called with a single argument: a file descriptor
open on the specified bus, which has been initialized to communicate with
a slave device at the specified address.

Note that in some cases it is possible to lock the I2C bus or crash the
whole system through ill-advised probing. The callback function should
only read or write registers which are generally safe for any of the devices
that use the given address. It should also 1) perform at least one read
or write operation -- not necessarily both -- to verify it can actually
talk to something and 2) attempt to distinguish amongst the various
devices that can use the address in question, if there are more than one.
Some devices have ID registers that are ideal for this purpose.

On success, returns a descriptor open and ready to talk to the intended
slave device (which has been verified as present by the callback).

On failure, returns a negative value.
***/
static int try_dev(const char * const fname, const uint8_t addr,
 const char * const devname, int(*callback)(const int fd)) {
   int fd = -1, rtn = 0;

   TSTA(!fname); TSTA(!devname); TSTA(!callback);

   if((fd = open(fname, O_RDWR)) < 0) {
      rtn = sysdiag((char *)"open", (char *)"can't open %s", fname); goto done;
   }

   if(ioctl(fd, I2C_SLAVE, addr)) {
      rtn = sysdiag((char *)"ioctl", (char *)"no I2C slave %02x on %s", addr, fname); goto done;
   }

   if(callback(fd)) {
      rtn = diag((char *)"no %s at address %02x on %s", devname, addr, fname);
      goto done;
   }

   diag((char *)"%s found at %02x on %s", devname, addr, fname);

   done:
   if(rtn && fd >= 0) {
      if(close(fd)) sysdiag((char *)"close",(char *)"can't close I2C device");
      fd = -1;
   }
   return fd;
}

/*** i2c_find() -- find and open an I2C device

This function attempts to find a specific I2C device connected to any one of
the available busses.

Arguments:
   addr -- the unshifted address of the desired I2C device
   devname -- pointer to a nul-terminated string containing the human-
      readable name of the device (typically a part number like "DS1307")
   callback -- pointer to a callback function used to test for the
      presence of the desired device; see try_dev() above for details

On success, returns an open file descriptor ready for communication with
the specified device. The caller must close()(2) this descriptor when it
is no longer needed.

On failure, returns a negative value.
***/
int i2c_find(const uint8_t addr, const char * const devname,
int(*callback)(const int fd)) {
   DIR *dir=NULL;
   int fd = -1, rtn = 0;
   struct dirent *ent;
   struct stat sbuf;

   TSTA(!devname); TSTA(!callback);

   if(chdir(DEVDIR)) {
      rtn = sysdiag((char *)"chdir", (char *)"can't chdir to %s", DEVDIR); goto done;
   }

   if((dir = opendir(".")) == NULL) {
      rtn = sysdiag((char *)"opendir", (char *)"can't open %s", DEVDIR); goto done;
   }

   while((ent = readdir(dir)) != NULL) {
      if(strlen(ent->d_name) < 5) continue;
      if(memcmp(ent->d_name, "i2c-", 4)) continue;
      if(stat(ent->d_name, &sbuf)) {
         sysdiag((char *)"stat", (char *)"can't stat %s", ent->d_name);
         continue;
      }
      if(!S_ISCHR(sbuf.st_mode)) continue;

      if((fd = try_dev(ent->d_name, addr, devname, callback)) >= 0) goto done;
   }

   rtn = diag((char *)"no I2C device found with address %02x", addr);

   done:
   if(dir && closedir(dir)) rtn = sysdiag((char *)"closedir", (char *)"can't close %s", DEVDIR);
   if(rtn && fd >= 0) {
      if(close(fd)) sysdiag((char *)"close", (char *)"can't close I2C device");
      fd = -1;
   }
   return fd;
}
