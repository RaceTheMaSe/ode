#include "UnitTest++.h"
#include "TestResults.h"
#include "TimeHelpers.h"
#include "RecordingReporter.h"

using namespace UnitTest;

namespace
{

TEST(TimeConstraintSucceedsWithFastTest)
{
    TestResults result;
    {
        const TimeConstraint t(200, result, TestDetails("", "", "", 0));
        TimeHelpers::SleepMs(5);
    }
    CHECK_EQUAL(0, result.GetFailureCount());
}

TEST(TimeConstraintFailsWithSlowTest)
{
    TestResults result;
    {
        const TimeConstraint t(10, result, TestDetails("", "", "", 0));
        TimeHelpers::SleepMs(20);
    }
    CHECK_EQUAL(1, result.GetFailureCount());
}

TEST(TimeConstraintFailureIncludesCorrectData)
{
    RecordingReporter reporter;
    TestResults result(&reporter);
    {
        const TestDetails details("testname", "suitename", "filename", 10);
        const TimeConstraint t(10, result, details);
        TimeHelpers::SleepMs(20);
    }
    CHECK(std::strstr(reporter.lastFailedFile, "filename"));
    CHECK_EQUAL(10, reporter.lastFailedLine);
    CHECK(std::strstr(reporter.lastFailedTest, "testname"));
}

TEST(TimeConstraintFailureIncludesTimeoutInformation)
{
    RecordingReporter reporter;
    TestResults result(&reporter);
    {
        const TimeConstraint t(10, result, TestDetails("", "", "", 0));
        TimeHelpers::SleepMs(20);
    }
    CHECK(std::strstr(reporter.lastFailedMessage, "ime constraint"));
    CHECK(std::strstr(reporter.lastFailedMessage, "under 10ms"));
}

}  // namespace
