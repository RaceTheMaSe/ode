#ifndef UNITTEST_ASSERT_H
#define UNITTEST_ASSERT_H
#include "UnitTestConfig++.h"

namespace UnitTest {

NORETURN_VOID ReportAssert(char const* description, char const* filename, int lineNumber);
    
}

#endif
