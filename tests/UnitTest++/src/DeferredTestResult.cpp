#include "DeferredTestResult.h"

#include <cstdlib>

namespace UnitTest
{

DeferredTestResult::DeferredTestResult()
	: timeElapsed(0.0f)
	, failed(false)
{
}

DeferredTestResult::DeferredTestResult(char const* suite, char const* test)
	: suiteName(suite)
	, testName(test)
	, timeElapsed(0.0f)
	, failed(false)
{
}

}  // namespace UnitTest
