#ifndef DEBUG_H
#define DEBUG_H

#include "diag.h"

/* IMPORTANT: Never use these macros to perform a test that you expect
   might fail for an actual user. Use them only to test for program bugs
   and "never happens" conditions. */

/* This header defines a set of macros that allows the same (terse) code to
   use thorough and paranoid assertion checking everywhere, or to be as
   ruthlessly space- and time-efficient as possible, according to the
   definition of the PARANOIA_LEVEL preprocessor constant in effect.

   The general case is that you can write something like:

      TST(var=foo()+bar(), <= junk+2);

   If error checking is turned up, this expands to (approximately):

      if((var = foo() + bar()) <= junk + 2) { BOMB(); }

   where BOMB() is a notional function that emits an error message giving
   the module name and line number, then forces program termination. If
   error checking is off, the same line expands to:

      var = foo() + bar();

   In general, the first argument to TST can be any expression. If
   it has side-effects, they will happen exactly once regardless of
   the paranoia level.

   The second argument is text which can be appended to the parenthesized
   first expression to form a conditional which, if true, causes the
   program to abort. Side-effects here happen only if error checking is
   turned on, thus are best avoided.

   Some other shortcut macros are defined in terms of TST:

   TSTP(x); -- assert that x evaluates to non-NULL
   TSTR(x); -- assert that x evaluates to 0

   This is amazingly useful, as you can replace the usual:

      #if PARANOIA_LEVEL == 0
         (void)SomeDumbFunction(arg);
      #else
         if(SomeDumbFunction(arg)) bomb("unhappy thing");
      #endif

   with the equivalent:

      TSTR(SomeDumbFunction(arg));

   which is less than half as long and much easier to read. It also makes
   it harder to make mistakes which cause your program to work when paranoid
   but break when not, or to run more slowly than it needs to in production.

   A special assertion macro TSTA(x) is also defined. If error checking
   is enabled, it evaluates x as an expression which must be false or the
   program aborts. If error checking is off, it expands to nothing. Use
   this _only_ when x doesn't have side-effects. It is useful for things
   like sanity-checking function arguments.

   The macro UNREACHABLE, if error checking is enabled, always aborts
   with a message indicating a section of code marked unreachable was
   in fact reached. If error checking is disbaled, it expands to nothing.

   The astute reader will note that the TSTx macros do not let you specify
   a message. That is intentional; the idea is that you'll be given the
   module name and line number, and will have to RTFS anyway to fix the
   problem, so you might as well have to look there to see what went wrong.
*/

/* Note for advanced users: The various TSTx macros behave marginally
   differently under PARANOIA_SOME versus PARANOIA_UTMOST. The
   difference is in the message displayed: UTMOST gives the whole
   expression that evaluated true to cause the bomb, while SOME
   just says "oops". The UTMOST version is clearly more useful, but
   bloats your binary size. */

/* some preliminaries */
#define PARANOIA_NONE 0
#define PARANOIA_SOME 1
#define PARANOIA_UTMOST 2
#ifndef PARANOIA_LEVEL
#define PARANOIA_LEVEL PARANOIA_NONE
#endif /* ifndef PARANOIA_LEVEL */

#if PARANOIA_LEVEL == PARANOIA_UTMOST || PARANOIA_LEVEL == PARANOIA_SOME
#if PARANOIA_LEVEL == PARANOIA_UTMOST
#define TST(x,y) do{ if((x)y) BOMB("(" #x ")" #y); } while(0)
#else
#define TST(x,y) do{ if((x)y) BOMB("oops"); } while(0)
#endif
#define TSTP(x) TST(x,==NULL)
#define TSTR(x) TST(x,!=0)
#define TSTA(x) TST(x,!=0)
#define UNREACHABLE BOMB("unreachable block executed")
#elif PARANOIA_LEVEL == PARANOIA_NONE
#define TST(x,y) x
#define TSTP(x) x
#define TSTR(x) x
#define TSTA(x)
#define UNREACHABLE
#else /* if PARANOIA_LEVEL isn't one of _UTMOST, _SOME or _NONE */
#error unrecognized PARANOIA_LEVEL setting
#endif /* if PARANOIA_LEVEL == various things */

/* If they call BOMB() explicitly, they probably want it to do that even
   if running in non-paranoid mode. */
#define BOMB(x) do{diag(__FILE__":%d "x, __LINE__); abort();} while(0)

#endif /* ifndef DEBUG_H */
