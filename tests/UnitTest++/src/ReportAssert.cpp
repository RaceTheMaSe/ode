#include "ReportAssert.h"
#include "AssertException.h"

namespace UnitTest {

NORETURN_VOID ReportAssert(char const* description, char const* filename, int const lineNumber)
{
    throw AssertException(description, filename, lineNumber);
}

}
