#include "UnitTest++.h"
#include "TestReporter.h"
#include "TimeHelpers.h"

using namespace UnitTest;

namespace {

TEST (PassingTestHasNoFailures)
{
    class PassingTest : public Test
    {
    public:
        PassingTest() : Test("passing") {}
    private:
        VIRTUAL void RunImpl(TestResults& testResults_) const OVERRIDE
        {
            CHECK(true);
        }
    };

    TestResults results;
    PassingTest().Run(results);
    CHECK_EQUAL(0, results.GetFailureCount());
}


TEST (FailingTestHasFailures)
{
    class FailingTest : public Test
    {
    public:
        FailingTest() : Test("failing") {}
    private:
        VIRTUAL void RunImpl(TestResults& testResults_) const OVERRIDE
        {
            CHECK(false);
        }
    };

    TestResults results;
    FailingTest().Run(results);
    CHECK_EQUAL(1, results.GetFailureCount());
}


TEST (ThrowingTestsAreReportedAsFailures)
{
    class CrashingTest : public Test
    {
    public:
        CrashingTest() : Test("throwing") {}
    private:
        VIRTUAL void RunImpl(TestResults& /*unsused*/) const OVERRIDE
        {
            throw "Blah";
        }
    };
 
    TestResults results;
    CrashingTest().Run(results);
    CHECK_EQUAL(1, results.GetFailureCount());
}


#ifndef UNITTEST_MINGW
TEST (CrashingTestsAreReportedAsFailures)
{
    class CrashingTest : public Test
    {
    public:
        CrashingTest() : Test("crashing") {}
    private:
        VIRTUAL void RunImpl(TestResults&) const OVERRIDE
        {
            reinterpret_cast< void (*)() >(0)();
        }
    };

    TestResults results;
    CrashingTest().Run(results);
    CHECK_EQUAL(1, results.GetFailureCount());
}
#endif

TEST (TestWithUnspecifiedSuiteGetsDefaultSuite)
{
    const Test test("test");
    CHECK(test.m_details.suiteName != NULL);
    CHECK_EQUAL("DefaultSuite", test.m_details.suiteName);
}

TEST (TestReflectsSpecifiedSuiteName)
{
    const Test test("test", "testSuite");
    CHECK(test.m_details.suiteName != NULL);
    CHECK_EQUAL("testSuite", test.m_details.suiteName);
}


}  // namespace
