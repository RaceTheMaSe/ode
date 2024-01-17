#ifndef UNITTEST_TESTREPORTERSTDOUT_H
#define UNITTEST_TESTREPORTERSTDOUT_H

#include "TestReporter.h"

namespace UnitTest {

class TestReporterStdout : public TestReporter
{
private:
    virtual void ReportTestStart(TestDetails const& test) OVERRIDE;
    virtual void ReportFailure(TestDetails const& test, char const* failure) OVERRIDE;
    virtual void ReportTestFinish(TestDetails const& test, float secondsElapsed) OVERRIDE;
    virtual void ReportSummary(int totalTestCount, int failedTestCount, int failureCount, float secondsElapsed) OVERRIDE;
};

}

#endif 
