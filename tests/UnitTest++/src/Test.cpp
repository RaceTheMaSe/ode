#include "UnitTestConfig++.h"
#include "Test.h"
#include "TestList.h"
#include "TestResults.h"
#include "AssertException.h"
#include "MemoryOutStream.h"

#ifdef UNITTEST_POSIX
    #include "Posix/SignalTranslator.h"
#endif

namespace UnitTest {

#if defined(_MSC_VER)
#pragma warning(disable: 4640) // construction of local static object is not thread-safe
#endif
TestList& Test::GetTestList()
{
    static TestList s_list;
    return s_list;
}

Test::Test(char const* testName, char const* suiteName, char const* filename, int const lineNumber)
    : m_details(testName, suiteName, filename, lineNumber)
    , next(NULL)
    , m_timeConstraintExempt(false)
{
}

Test::~Test()
{
}

void Test::Run(TestResults& testResults) const
{
    try
    {
#ifdef UNITTEST_POSIX
        UNITTEST_THROW_SIGNALS
#endif
        RunImpl(testResults);
    }
    catch (AssertException const& e)
    {
        testResults.OnTestFailure( TestDetails(m_details.testName, m_details.suiteName, e.Filename(), e.LineNumber()), e.what());
    }
    catch (std::exception const& e)
    {
        MemoryOutStream stream;
        stream << "Unhandled exception: " << e.what();
        testResults.OnTestFailure(m_details, stream.GetText());
    }
    catch (...)
    {
        testResults.OnTestFailure(m_details, "Unhandled exception: Crash!");
    }
}


void Test::RunImpl(TestResults& /**/) const
{
}


}  // namespace UnitTest
