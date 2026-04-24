#include "UnitTestConfig++.h"

#if defined UNITTEST_POSIX
    #include "Posix/TimeHelpersPosix.h"
#else
    #include "Win32/TimeHelpersWin.h"
#endif
