#include "TestList.h"
#include "Test.h"

#include <cassert>
#include <cstddef>

namespace UnitTest {

TestList::TestList() 
    : m_head(NULL)
    , m_tail(NULL)
{
}

void TestList::Add(Test* test)
{
    if (m_tail == NULL)
    {
        assert(m_head == NULL);
        m_head = test;
        m_tail = test;
    }
    else
    {
        m_tail->next = test;
        m_tail = test;
    }
}

const Test* TestList::GetHead() const
{
    return m_head;
}

ListAdder::ListAdder(TestList& list, Test* test)
{
    list.Add(test);
}

}
