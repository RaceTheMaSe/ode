#ifndef UNITTEST_DEFERREDTESTREPORTER_H
#define UNITTEST_DEFERREDTESTREPORTER_H

#include "TestReporter.h"
#include "DeferredTestResult.h"

#include <vector>

namespace UnitTest
{

class DeferredTestReporter : public TestReporter
{
public:
    VIRTUAL void ReportTestStart(TestDetails const& details) OVERRIDE;
    VIRTUAL void ReportFailure(TestDetails const& details, char const* failure) OVERRIDE;
    VIRTUAL void ReportTestFinish(TestDetails const& details, float secondsElapsed) OVERRIDE;

    typedef std::vector< DeferredTestResult > DeferredTestResultList;
    DeferredTestResultList& GetResults();

private:
    DeferredTestResultList m_results;
};

}

#endif
