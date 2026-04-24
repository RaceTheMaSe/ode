// openode_UnitTest++.cpp : Defines the entry point for the console application.
//

#include <UnitTest++.h>
#include <ode/ode.h>

int main()
{
    dInitODE();
    const int res = UnitTest::RunAllTests();
    dCloseODE();
    return res;
}
