#ifndef UNITTEST_TESTREPORTERSTDOUT_H
#define UNITTEST_TESTREPORTERSTDOUT_H

#include "TestReporter.h"

namespace UnitTest {

class TestReporterStdout : public TestReporter
{
private:
    VIRTUAL void ReportTestStart(TestDetails const& test) OVERRIDE;
    VIRTUAL void ReportFailure(TestDetails const& test, char const* failure) OVERRIDE;
    VIRTUAL void ReportTestFinish(TestDetails const& test, float secondsElapsed) OVERRIDE;
    VIRTUAL void ReportSummary(int totalTestCount, int failedTestCount, int failureCount, float secondsElapsed) OVERRIDE;
};

}  // namespace UnitTest

#endif 
