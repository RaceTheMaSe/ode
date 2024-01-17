#include "AssertException.h"
#include <cstring>

namespace UnitTest {

AssertException::AssertException(char const* description, char const* filename, int const lineNumber)
    : m_lineNumber(lineNumber)
{
    std::strcpy(m_description, description);
    std::strcpy(m_filename, filename);
}

AssertException::~AssertException() NOEXCEPT
{
}

char const* AssertException::what() const NOEXCEPT
{
    return m_description;
}

char const* AssertException::Filename() const
{
    return m_filename;
}

int AssertException::LineNumber() const
{
    return m_lineNumber;
}

}
