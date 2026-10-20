/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

#include <setjmp.h>
#include <ode/ode.h>

//****************************************************************************
// matrix sizes

#define dALIGN_SIZE(buf_size, alignment) (((size_t)(buf_size) + (alignment - 1)) & (size_t)(int)(~(alignment - 1))) // Casting the mask to int ensures sign-extension to larger integer sizes

#define MSIZE 21
#define MSIZE4 dALIGN_SIZE(MSIZE, 4)	// MSIZE rounded up to 4



//****************************************************************************
// matrix accessors

#define _A(i,j) A[(i)*4+(j)]
#define _I(i,j) I[(i)*4+(j)]
#define _R(i,j) R[(i)*4+(j)]

//****************************************************************************
// tolerances

#ifdef dDOUBLE
const double tol = 1e-10;
#endif

#ifdef dSINGLE
const float tol = 1e-5f;
#endif

//****************************************************************************
// misc messages and error handling

#ifdef __GNUC__
#define HEADER printf ("%s()\n", __FUNCTION__)
#else
#define HEADER printf ("%s:%d\n",__FILE__,__LINE__)
#endif

static jmp_buf jump_buffer;


static ODE_NORETURN_VOID myMessageFunction (int num, const char *msg, va_list ap)
{
    printf ("(Message %d: ",num);
    vprintf (msg,ap);
    printf (")");
    dSetMessageHandler (NULL);
    longjmp (jump_buffer,1);
}


#define TRAP_MESSAGE(do,ifnomsg,ifmsg) \
    dSetMessageHandler (&myMessageFunction); \
    if (setjmp (jump_buffer)) { \
        dSetMessageHandler (NULL); \
        ifmsg ; \
    } \
    else { \
        dSetMessageHandler (&myMessageFunction); \
        do ; \
        ifnomsg ; \
    } \
    dSetMessageHandler (NULL)

//****************************************************************************
// utility stuff

// compare two numbers, within a threshhold, return 1 if approx equal

static int cmp (dReal a, dReal b)
{
    return (fabs(a-b) < tol);
}

//****************************************************************************
// matrix utility stuff

// compare a 3x3 matrix with the identity

static int cmpIdentityMat3 (dMatrix3 A)
{
    return
        (cmp(_A(0,0),1.0) && cmp(_A(0,1),0.0) && cmp(_A(0,2),0.0) &&
        cmp(_A(1,0),0.0) && cmp(_A(1,1),1.0) && cmp(_A(1,2),0.0) &&
        cmp(_A(2,0),0.0) && cmp(_A(2,1),0.0) && cmp(_A(2,2),1.0));
}


// transpose a 3x3 matrix in-line

static void transpose3x3 (dMatrix3 A)
{
    dReal tmp;
    tmp=A[4]; A[4]=A[1]; A[1]=tmp;
    tmp=A[8]; A[8]=A[2]; A[2]=tmp;
    tmp=A[9]; A[9]=A[6]; A[6]=tmp;
}

//****************************************************************************
// test miscellaneous math functions

static void testRandomNumberGenerator()
{
    HEADER;
    if (dTestRand()) printf ("\tpassed\n");
    else printf ("\tFAILED\n");
}


static void testInfinity()
{
    HEADER;
    CONST_IF (REAL(1e10) < dInfinity && REAL(-1e10) > -dInfinity && -dInfinity < dInfinity)
        printf ("\tpassed\n");
    else printf ("\tFAILED\n");
}


static void testPad()
{
    HEADER;
    char s[100];
    s[0]=0;
    for (unsigned int i=0; i<=16; i++) sprintf (s+strlen(s),"%u ",dPAD(i));
    printf ("\t%s\n", strcmp(s,"0 1 4 4 4 8 8 8 8 12 12 12 12 16 16 16 16 ") ?
        "FAILED" : "passed");
}


static void testCrossProduct()
{
    HEADER;

    dVector3 a1,a2,b,c;
    dMatrix3 B;
    dMakeRandomVector (b,3,1.0);
    dMakeRandomVector (c,3,1.0);

    dCalcVectorCross3(a1,b,c);

    dSetZero (B,12);
    dSetCrossMatrixPlus(B,b,4);
    dMultiply0 (a2,B,c,3,3,1);

    dReal diff = dMaxDifference(a1,a2,3,1);
    printf ("\t%s\n", diff > tol ? "FAILED" : "passed");
}


static void testSetZero()
{
    HEADER;
    dReal a[100];
    dMakeRandomVector (a,100,1.0);
    dSetZero (a,100);
    for (int i=0; i<100; i++) if (a[i] != REAL(0.0)) {
        printf ("\tFAILED\n");
        return;
    }
    printf ("\tpassed\n");
}


static void testNormalize3()
{
    HEADER;
    int i,j,bad=0;
    dVector3 n1,n2;
    for (i=0; i<1000; i++) {
        dMakeRandomVector (n1,3,1.0);
        for (j=0; j<3; j++) n2[j]=n1[j];
        dNormalize3 (n2);
        if (dFabs(dCalcVectorDot3(n2,n2) - REAL(1.0)) > tol) bad |= 1;
        if (dFabs(n2[0]/n1[0] - n2[1]/n1[1]) > tol) bad |= 2;
        if (dFabs(n2[0]/n1[0] - n2[2]/n1[2]) > tol) bad |= 4;
        if (dFabs(n2[1]/n1[1] - n2[2]/n1[2]) > tol) bad |= 8;
        if (dFabs(dCalcVectorDot3(n2,n1) - dSqrt(dCalcVectorDot3(n1,n1))) > tol) bad |= 16;
        if (bad) {
            printf ("\tFAILED (code=%x)\n",(unsigned int)bad);
            return;
        }
    }
    printf ("\tpassed\n");
}


/*
static void testReorthonormalize()
{
HEADER;
dMatrix3 R,I;
dMakeRandomMatrix (R,3,3,1.0);
for (int i=0; i<30; i++) dReorthonormalize (R);
dMultiply2 (I,R,R,3,3,3);
printf ("\t%s\n",cmpIdentityMat3 (I) ? "passed" : "FAILED");
}
*/


static void testPlaneSpace()
{
    HEADER;
    dVector3 n,p,q;
    int bad = 0;
    for (int i=0; i<1000; i++) {
        dMakeRandomVector (n,3,1.0);
        dNormalize3 (n);
        dPlaneSpace (n,p,q);
        if (fabs(dCalcVectorDot3(n,p)) > tol) bad = 1;
        if (fabs(dCalcVectorDot3(n,q)) > tol) bad = 1;
        if (fabs(dCalcVectorDot3(p,q)) > tol) bad = 1;
        if (fabs(dCalcVectorDot3(p,p)-1) > tol) bad = 1;
        if (fabs(dCalcVectorDot3(q,q)-1) > tol) bad = 1;
    }
    printf ("\t%s\n", bad ? "FAILED" : "passed");
}

//****************************************************************************
// test matrix functions

static void testMatrixMultiply()
{
    // A is 2x3, B is 3x4, B2 is B except stored columnwise, C is 2x4
    dReal A[8],B[12],A2[12],B2[16],C[8];
    int i;

    HEADER;
    dSetZero (A,8);
    for (i=0; i<3; i++) A[i] = dReal(i+2);
    for (i=0; i<3; i++) A[i+4] = dReal(i+3+2);
    for (i=0; i<12; i++) B[i] = dReal(i+8);
    dSetZero (A2,12);
    for (i=0; i<6; i++) A2[i+2*(i/2)] = A[i+i/3];
    dSetZero (B2,16);
    for (i=0; i<12; i++) B2[i+i/3] = B[i];

    dMultiply0 (C,A,B,2,3,4);
    if (C[0] != 116 || C[1] != 125 || C[2] != 134 || C[3] != 143 ||
        C[4] != 224 || C[5] != 242 || C[6] != 260 || C[7] != 278)
        printf ("\tFAILED (1)\n"); else printf ("\tpassed (1)\n");

    dMultiply1 (C,A2,B,2,3,4);
    if (C[0] != 160 || C[1] != 172 || C[2] != 184 || C[3] != 196 ||
        C[4] != 196 || C[5] != 211 || C[6] != 226 || C[7] != 241)
        printf ("\tFAILED (2)\n"); else printf ("\tpassed (2)\n");

    dMultiply2 (C,A,B2,2,3,4);
    if (C[0] != 83 || C[1] != 110 || C[2] != 137 || C[3] != 164 ||
        C[4] != 164 || C[5] != 218 || C[6] != 272 || C[7] != 326)
        printf ("\tFAILED (3)\n"); else printf ("\tpassed (3)\n");
}


static void testSmallMatrixMultiply()
{
    dMatrix3 A,B,C,A2;
    dVector3 a,a2,x;

    HEADER;
    dMakeRandomMatrix (A,3,3,1.0);
    dMakeRandomMatrix (B,3,3,1.0);
    dMakeRandomMatrix (C,3,3,1.0);
    dMakeRandomMatrix (x,3,1,1.0);

    // dMultiply0_331()
    dMultiply0_331 (a,B,x);
    dMultiply0 (a2,B,x,3,3,1);
    printf ("\t%s (1)\n",(dMaxDifference (a,a2,3,1) > tol) ? "FAILED" :
        "passed");

    // dMultiply1_331()
    dMultiply1_331 (a,B,x);
    dMultiply1 (a2,B,x,3,3,1);
    printf ("\t%s (2)\n",(dMaxDifference (a,a2,3,1) > tol) ? "FAILED" :
        "passed");

    // dMultiply0_133
    dMultiply0_133 (a,x,B);
    dMultiply0 (a2,x,B,1,3,3);
    printf ("\t%s (3)\n",(dMaxDifference (a,a2,1,3) > tol) ? "FAILED" :
        "passed");

    // dMultiply0_333()
    dMultiply0_333 (A,B,C);
    dMultiply0 (A2,B,C,3,3,3);
    printf ("\t%s (4)\n",(dMaxDifference (A,A2,3,3) > tol) ? "FAILED" :
        "passed");

    // dMultiply1_333()
    dMultiply1_333 (A,B,C);
    dMultiply1 (A2,B,C,3,3,3);
    printf ("\t%s (5)\n",(dMaxDifference (A,A2,3,3) > tol) ? "FAILED" :
        "passed");

    // dMultiply2_333()
    dMultiply2_333 (A,B,C);
    dMultiply2 (A2,B,C,3,3,3);
    printf ("\t%s (6)\n",(dMaxDifference (A,A2,3,3) > tol) ? "FAILED" :
        "passed");
}


static void testCholeskyFactorization()
{
    dsizeint matrixSize = sizeof(dReal) * MSIZE4 * MSIZE;
    dReal *A = (dReal *)dAlloc(matrixSize), *B = (dReal *)dAlloc(matrixSize), *C = (dReal *)dAlloc(matrixSize), diff;

    HEADER;
    dMakeRandomMatrix (A,MSIZE,MSIZE,1.0);
    dMultiply2 (B,A,A,MSIZE,MSIZE,MSIZE);
    memcpy (A,B,MSIZE4*MSIZE*sizeof(dReal));
    if (dFactorCholesky (B,MSIZE)) printf ("\tpassed (1)\n");
    else printf ("\tFAILED (1)\n");
    dClearUpperTriangle (B,MSIZE);
    dMultiply2 (C,B,B,MSIZE,MSIZE,MSIZE);
    diff = dMaxDifference(A,C,MSIZE,MSIZE);
    printf ("\tmaximum difference = %.6e - %s (2)\n",(double)diff,
        diff > tol ? "FAILED" : "passed");

    dFree(C, matrixSize);
    dFree(B, matrixSize);
    dFree(A, matrixSize);
}


static void testCholeskySolve()
{
    dsizeint matrixSize = sizeof(dReal) * MSIZE4 * MSIZE, vectorSize = sizeof(dReal) * MSIZE;
    dReal *A = (dReal *)dAlloc(matrixSize), *L = (dReal *)dAlloc(matrixSize);
    dReal *b = (dReal *)dAlloc(vectorSize), *x = (dReal *)dAlloc(vectorSize), *btest = (dReal *)dAlloc(vectorSize), diff;

    HEADER;

    // get A,L = PD matrix
    dMakeRandomMatrix (A,MSIZE,MSIZE,1.0);
    dMultiply2 (L,A,A,MSIZE,MSIZE,MSIZE);
    memcpy (A,L,MSIZE4*MSIZE*sizeof(dReal));

    // get b,x = right hand side
    dMakeRandomMatrix (b,MSIZE,1,1.0);
    memcpy (x,b,MSIZE*sizeof(dReal));

    // factor L
    if (dFactorCholesky (L,MSIZE)) printf ("\tpassed (1)\n");
    else printf ("\tFAILED (1)\n");
    dClearUpperTriangle (L,MSIZE);

    // solve A*x = b
    dSolveCholesky (L,x,MSIZE);

    // compute A*x and compare it with b
    dMultiply2 (btest,A,x,MSIZE,MSIZE,1);
    diff = dMaxDifference(b,btest,MSIZE,1);
    printf ("\tmaximum difference = %.6e - %s (2)\n",(double)diff,
        diff > tol ? "FAILED" : "passed");

    dFree(btest, vectorSize);
    dFree(x, vectorSize);
    dFree(b, vectorSize);
    dFree(L, matrixSize);
    dFree(A, matrixSize);
}


static void testInvertPDMatrix()
{
    unsigned i,j,ok;
    dsizeint matrixSize = sizeof(dReal) * MSIZE4 * MSIZE;
    dReal *A = (dReal *)dAlloc(matrixSize), *Ainv = (dReal *)dAlloc(matrixSize), *I = (dReal *)dAlloc(matrixSize);

    HEADER;

    dMakeRandomMatrix (A,MSIZE,MSIZE,1.0);
    dMultiply2 (Ainv,A,A,MSIZE,MSIZE,MSIZE);
    memcpy (A,Ainv,MSIZE4*MSIZE*sizeof(dReal));
    dSetZero (Ainv,MSIZE4*MSIZE);

    if (dInvertPDMatrix (A,Ainv,MSIZE))
        printf ("\tpassed (1)\n"); else printf ("\tFAILED (1)\n");
    dMultiply0 (I,A,Ainv,MSIZE,MSIZE,MSIZE);

    // compare with identity
    ok = 1;
    for (i=0; i<MSIZE; i++) {
        for (j=0; j<MSIZE; j++) {
            if (i != j) if (cmp (I[i*MSIZE4+j],0.0)==0) ok = 0;
        }
    }
    for (i=0; i<MSIZE; i++) {
        if (cmp (I[i*MSIZE4+i],1.0)==0) ok = 0;
    }
    if (ok) printf ("\tpassed (2)\n"); else printf ("\tFAILED (2)\n");

    dFree(I, matrixSize);
    dFree(Ainv, matrixSize);
    dFree(A, matrixSize);
}


static void testIsPositiveDefinite()
{
    dsizeint matrixSize = sizeof(dReal) * MSIZE4 * MSIZE;
    dReal *A = (dReal *)dAlloc(matrixSize), *B = (dReal *)dAlloc(matrixSize);

    HEADER;

    dMakeRandomMatrix (A,MSIZE,MSIZE,1.0);
    dMultiply2 (B,A,A,MSIZE,MSIZE,MSIZE);
    printf ("\t%s\n",dIsPositiveDefinite(A,MSIZE) ? "FAILED (1)":"passed (1)");
    printf ("\t%s\n",dIsPositiveDefinite(B,MSIZE) ? "passed (2)":"FAILED (2)");

    dFree(B, matrixSize);
    dFree(A, matrixSize);
}


static void testFastLDLTFactorization()
{
    unsigned i,j;
    dsizeint matrixSize = sizeof(dReal) * MSIZE4 * MSIZE, vectorSize = sizeof(dReal) * MSIZE;
    dReal *A = (dReal *)dAlloc(matrixSize), *L = (dReal *)dAlloc(matrixSize), *DL = (dReal *)dAlloc(matrixSize),
        *ATEST = (dReal *)dAlloc(matrixSize), *d = (dReal *)dAlloc(vectorSize), diff;

    HEADER;

    dMakeRandomMatrix (A,MSIZE,MSIZE,1.0);
    dMultiply2 (L,A,A,MSIZE,MSIZE,MSIZE);
    memcpy (A,L,MSIZE4*MSIZE*sizeof(dReal));

    dFactorLDLT (L,d,MSIZE,MSIZE4);

    dClearUpperTriangle (L,MSIZE);
    for (i=0; i<MSIZE; i++) L[i*MSIZE4+i] = 1.0;

    dSetZero (DL,MSIZE4*MSIZE);
    for (i=0; i<MSIZE; i++) {
        for (j=0; j<MSIZE; j++) DL[i*MSIZE4+j] = L[i*MSIZE4+j] / d[j];
    }

    dMultiply2 (ATEST,L,DL,MSIZE,MSIZE,MSIZE);
    diff = dMaxDifference(A,ATEST,MSIZE,MSIZE);
    printf ("\tmaximum difference = %.6e - %s\n",(double)diff,
        diff > tol ? "FAILED" : "passed");

    dFree(d, vectorSize);
    dFree(ATEST, matrixSize);
    dFree(DL, matrixSize);
    dFree(L, matrixSize);
    dFree(A, matrixSize);
}


static void testCoopLDLTFactorization()
{
    unsigned int i,j;

    const dsizeint COOP_MSIZE = MSIZE * 51, COOP_MSIZE4 = (dsizeint)dALIGN_SIZE(COOP_MSIZE, 4);

    dsizeint matrixSize = sizeof(dReal) * COOP_MSIZE4 * COOP_MSIZE, vectorSize = sizeof(dReal) * COOP_MSIZE;
    dReal *A = (dReal *)dAlloc(matrixSize), *L = (dReal *)dAlloc(matrixSize), *DL = (dReal *)dAlloc(matrixSize),
        *ATEST = (dReal *)dAlloc(matrixSize), *d = (dReal *)dAlloc(vectorSize), diff;

    const unsigned threadCountMaximum = 8;
    dThreadingImplementationID threading = dThreadingAllocateMultiThreadedImplementation();
    dCooperativeID cooperative = dCooperativeCreate(dThreadingImplementationGetFunctions(threading), threading);
    dThreadingThreadPoolID pool = dThreadingAllocateThreadPool(threadCountMaximum, 0, dAllocateFlagBasicData, NULL);
    dThreadingThreadPoolServeMultiThreadedImplementation(pool, threading);

    dResourceRequirementsID requirements = dResourceRequirementsCreate(cooperative);
    dEstimateCooperativelyFactorLDLTResourceRequirements(requirements, threadCountMaximum, COOP_MSIZE);
    dResourceContainerID resources = dResourceContainerAcquire(requirements);

    HEADER;

    for (unsigned int pass = 0; pass != 4; ++pass)
    {
        dTimerStart ("Factoring LDLT");

        const unsigned allowedThreads = 4;
        const unsigned PASS_MSIZE = COOP_MSIZE - pass, PASS_MSIZE4 = (unsigned int)dALIGN_SIZE(PASS_MSIZE, 4);

        dTimerNow ("Preparing data");
        dMakeRandomMatrix (L, (int)PASS_MSIZE, (int)PASS_MSIZE, 1.0);
        dMultiply2 (A, L, L, (int)PASS_MSIZE, (int)PASS_MSIZE, (int)PASS_MSIZE);
        memcpy (L, A, sizeof(dReal) * PASS_MSIZE4 * PASS_MSIZE);

        dTimerNow ("Factoring multi threaded");
        dCooperativelyFactorLDLT (resources, allowedThreads, L, d, PASS_MSIZE, PASS_MSIZE4);

        dTimerNow ("Verifying");
        dClearUpperTriangle (L, (int)PASS_MSIZE);
        for (i = 0; i < PASS_MSIZE; i++) L[i * PASS_MSIZE4 + i] = 1.0;

        dSetZero (DL, (int)(PASS_MSIZE4 * PASS_MSIZE));
        for (i = 0; i < PASS_MSIZE; i++) {
            for (j = 0; j < PASS_MSIZE; j++) DL[i * PASS_MSIZE4 + j] = L[i * PASS_MSIZE4 + j] / d[j];
        }

        dMultiply2 (ATEST, L, DL, (int)PASS_MSIZE, (int)PASS_MSIZE, (int)PASS_MSIZE);
        diff = dMaxDifference(A, ATEST, (int)PASS_MSIZE, (int)PASS_MSIZE);
        printf ("\tN=%u: maximum difference = %.6e - %s\n", PASS_MSIZE, (double)diff, diff > REAL(1e2) * tol ? "FAILED" : "passed");

        dTimerEnd();
        dTimerReport(stdout, 0);
    }

    dResourceContainerDestroy(resources);
    dResourceRequirementsDestroy(requirements);

    dThreadingImplementationShutdownProcessing(threading);
    dThreadingFreeThreadPool(pool);
    dCooperativeDestroy(cooperative);
    dThreadingFreeImplementation(threading);

    dFree(d, vectorSize);
    dFree(ATEST, matrixSize);
    dFree(DL, matrixSize);
    dFree(L, matrixSize);
    dFree(A, matrixSize);
}


static void testSolveLDLT()
{
    dsizeint matrixSize = sizeof(dReal) * MSIZE4 * MSIZE, vectorSize = sizeof(dReal) * MSIZE;
    dReal *A = (dReal *)dAlloc(matrixSize), *L = (dReal *)dAlloc(matrixSize), 
        *d = (dReal *)dAlloc(vectorSize), *x = (dReal *)dAlloc(vectorSize), 
        *b = (dReal *)dAlloc(vectorSize), *btest = (dReal *)dAlloc(vectorSize), diff;

    HEADER;

    dMakeRandomMatrix (A,MSIZE,MSIZE,1.0);
    dMultiply2 (L,A,A,MSIZE,MSIZE,MSIZE);
    memcpy (A,L,MSIZE4*MSIZE*sizeof(dReal));

    dMakeRandomMatrix (b,MSIZE,1,1.0);
    memcpy (x,b,MSIZE*sizeof(dReal));

    dFactorLDLT (L,d,MSIZE,MSIZE4);
    dSolveLDLT (L,d,x,MSIZE,MSIZE4);

    dMultiply2 (btest,A,x,MSIZE,MSIZE,1);
    diff = dMaxDifference(b,btest,MSIZE,1);
    printf ("\tmaximum difference = %.6e - %s\n",(double)diff,
        diff > tol ? "FAILED" : "passed");

    dFree(btest, vectorSize);
    dFree(b, vectorSize);
    dFree(x, vectorSize);
    dFree(d, vectorSize);
    dFree(L, matrixSize);
    dFree(A, matrixSize);
}

static void testCoopSolveLDLT()
{
    const dsizeint COOP_MSIZE = MSIZE * 51, COOP_MSIZE4 = (dsizeint)dALIGN_SIZE(COOP_MSIZE, 4);

    dsizeint matrixSize = sizeof(dReal) * COOP_MSIZE4 * COOP_MSIZE, vectorSize = sizeof(dReal) * COOP_MSIZE;
    dReal *A = (dReal *)dAlloc(matrixSize), *L = (dReal *)dAlloc(matrixSize), 
        *d = (dReal *)dAlloc(vectorSize), *x = (dReal *)dAlloc(vectorSize), 
        *b = (dReal *)dAlloc(vectorSize), *btest = (dReal *)dAlloc(vectorSize), diff;

    const unsigned threadCountMaximum = 8;
    dThreadingImplementationID threading = dThreadingAllocateMultiThreadedImplementation();
    dCooperativeID cooperative = dCooperativeCreate(dThreadingImplementationGetFunctions(threading), threading);
    dThreadingThreadPoolID pool = dThreadingAllocateThreadPool(threadCountMaximum, 0, dAllocateFlagBasicData, NULL);
    dThreadingThreadPoolServeMultiThreadedImplementation(pool, threading);

    dResourceRequirementsID requirements = dResourceRequirementsCreate(cooperative);
    dEstimateCooperativelySolveLDLTResourceRequirements(requirements, threadCountMaximum, COOP_MSIZE);
    dResourceContainerID resources = dResourceContainerAcquire(requirements);

    HEADER;

    for (unsigned int pass = 0; pass != 4; ++pass)
    {
        dTimerStart ("Solving LDLT");

        const unsigned allowedThreads = 4;
        const unsigned PASS_MSIZE = COOP_MSIZE - pass, PASS_MSIZE4 = (unsigned int)dALIGN_SIZE(PASS_MSIZE, 4);

        dTimerNow ("Preparing data");
        dMakeRandomMatrix (b, (int)PASS_MSIZE, 1, 1.0);

        dMakeRandomMatrix (L, (int)PASS_MSIZE, (int)PASS_MSIZE, 1.0);
        dMultiply2 (A, L, L, (int)PASS_MSIZE, (int)PASS_MSIZE, (int)PASS_MSIZE);

        memcpy (x, b, PASS_MSIZE * sizeof(dReal));
        memcpy (L, A, sizeof(dReal) * PASS_MSIZE4 * PASS_MSIZE);

        dTimerNow ("Factoring");
        dFactorLDLT (L, d, (int)PASS_MSIZE, (int)PASS_MSIZE4);

        dTimerNow ("Solving multi-threaded");
        dCooperativelySolveLDLT(resources, allowedThreads, L, d, x, PASS_MSIZE, PASS_MSIZE4);

        dTimerNow ("Verifying solution");
        dMultiply2 (btest, A, x, (int)PASS_MSIZE, (int)PASS_MSIZE, 1);
        diff = dMaxDifference(b, btest, (int)PASS_MSIZE, 1);
        printf ("\tN=%u: maximum difference = %.6e - %s\n", PASS_MSIZE, (double)diff, diff > REAL(1e2) * tol ? "FAILED" : "passed");

        dTimerEnd();
        dTimerReport(stdout, 0);
    }

    dResourceContainerDestroy(resources);
    dResourceRequirementsDestroy(requirements);

    dThreadingImplementationShutdownProcessing(threading);
    dThreadingFreeThreadPool(pool);
    dCooperativeDestroy(cooperative);
    dThreadingFreeImplementation(threading);

    dFree(btest, vectorSize);
    dFree(b, vectorSize);
    dFree(x, vectorSize);
    dFree(d, vectorSize);
    dFree(L, matrixSize);
    dFree(A, matrixSize);
}


static void testLDLTAddTL()
{
    unsigned i,j;
    dsizeint matrixSize = sizeof(dReal) * MSIZE4 * MSIZE, vectorSize = sizeof(dReal) * MSIZE;
    dReal *A = (dReal *)dAlloc(matrixSize), *L = (dReal *)dAlloc(matrixSize), 
        *DL = (dReal *)dAlloc(matrixSize), *ATEST = (dReal *)dAlloc(matrixSize), 
        *d = (dReal *)dAlloc(vectorSize), *a = (dReal *)dAlloc(vectorSize), diff;

    HEADER;

    dMakeRandomMatrix (A,MSIZE,MSIZE,1.0);
    dMultiply2 (L,A,A,MSIZE,MSIZE,MSIZE);
    memcpy (A,L,MSIZE4*MSIZE*sizeof(dReal));
    dFactorLDLT (L,d,MSIZE,MSIZE4);

    // delete first row and column of factorization
    for (i=0; i<MSIZE; i++) a[i] = -A[i*MSIZE4];
    a[0] += 1;
    dLDLTAddTL (L,d,a,MSIZE,MSIZE4);
    for (i=1; i<MSIZE; i++) L[i*MSIZE4] = 0;
    d[0] = 1;

    // get modified L*D*L'
    dClearUpperTriangle (L,MSIZE);
    for (i=0; i<MSIZE; i++) L[i*MSIZE4+i] = 1.0;
    dSetZero (DL,MSIZE4*MSIZE);
    for (i=0; i<MSIZE; i++) {
        for (j=0; j<MSIZE; j++) DL[i*MSIZE4+j] = L[i*MSIZE4+j] / d[j];
    }
    dMultiply2 (ATEST,L,DL,MSIZE,MSIZE,MSIZE);

    // compare it to A with its first row/column removed
    for (i=1; i<MSIZE; i++) A[i*MSIZE4] = A[i] = 0;
    A[0] = 1;
    diff = dMaxDifference(A,ATEST,MSIZE,MSIZE);
    printf ("\tmaximum difference = %.6e - %s\n",(double)diff,
        diff > tol ? "FAILED" : "passed");

    dFree(a, vectorSize);
    dFree(d, vectorSize);
    dFree(ATEST, matrixSize);
    dFree(DL, matrixSize);
    dFree(L, matrixSize);
    dFree(A, matrixSize);
}


static void testLDLTRemove()
{
    unsigned i,j,r;
    dsizeint intVectorSize = sizeof(int) * MSIZE, matrixSize = sizeof(dReal) * MSIZE4 * MSIZE, vectorSize = sizeof(dReal) * MSIZE;
    int *p = (int *)dAlloc(intVectorSize);
    dReal *A = (dReal *)dAlloc(matrixSize), *L = (dReal *)dAlloc(matrixSize), 
        *L2 = (dReal *)dAlloc(matrixSize), *DL2 = (dReal *)dAlloc(matrixSize), 
        *Atest1 = (dReal *)dAlloc(matrixSize), *Atest2 = (dReal *)dAlloc(matrixSize), 
        *d = (dReal *)dAlloc(vectorSize), *d2 = (dReal *)dAlloc(vectorSize), diff, maxdiff;

    HEADER;

    // make array of A row pointers
    dReal *Arows[MSIZE];
    for (i=0; i<MSIZE; i++) Arows[i] = A+i*MSIZE4;

    // fill permutation vector
    for (i=0; i<MSIZE; i++) p[i]=(int)i;

    dMakeRandomMatrix (A,MSIZE,MSIZE,1.0);
    dMultiply2 (L,A,A,MSIZE,MSIZE,MSIZE);
    memcpy (A,L,MSIZE4*MSIZE*sizeof(dReal));
    dFactorLDLT (L,d,MSIZE,MSIZE4);

    maxdiff = 1e10;
    for (r=0; r<MSIZE; r++) {
        // get Atest1 = A with row/column r removed
        memcpy (Atest1,A,MSIZE4*MSIZE*sizeof(dReal));
        dRemoveRowCol (Atest1,MSIZE,MSIZE4,(int)r);

        // test that the row/column removal worked
        unsigned bad = 0;
        for (i=0; i<MSIZE; i++) {
            for (j=0; j<MSIZE; j++) {
                if (i != r && j != r) {
                    unsigned ii = i;
                    unsigned jj = j;
                    if (ii >= r) ii--;
                    if (jj >= r) jj--;
                    if (A[i*MSIZE4+j] != Atest1[ii*MSIZE4+jj]) bad = 1;
                }
            }
        }
        if (bad) printf ("\trow/col removal FAILED for row %u\n",r);

        // zero out last row/column of Atest1
        for (i=0; i<MSIZE; i++) {
            Atest1[(MSIZE-1)*MSIZE4+i] = 0;
            Atest1[i*MSIZE4+MSIZE-1] = 0;
        }    

        // get L2*D2*L2' = adjusted factorization to remove that row
        memcpy (L2,L,MSIZE4*MSIZE*sizeof(dReal));
        memcpy (d2,d,MSIZE*sizeof(dReal));
        dLDLTRemove (/*A*/ Arows,p,L2,d2,MSIZE,MSIZE,(int)r,MSIZE4);

        // get Atest2 = L2*D2*L2'
        dClearUpperTriangle (L2,MSIZE);
        for (i=0; i<(MSIZE-1); i++) L2[i*MSIZE4+i] = 1.0;
        for (i=0; i<MSIZE; i++) L2[(MSIZE-1)*MSIZE4+i] = 0;
        d2[MSIZE-1] = 1;
        dSetZero (DL2,MSIZE4*MSIZE);
        for (i=0; i<(MSIZE-1); i++) {
            for (j=0; j<MSIZE-1; j++) DL2[i*MSIZE4+j] = L2[i*MSIZE4+j] / d2[j];
        }

        dMultiply2 (Atest2,L2,DL2,MSIZE,MSIZE,MSIZE);

        diff = dMaxDifference(Atest1,Atest2,MSIZE,MSIZE);
        if (diff < maxdiff) maxdiff = diff;

        /*
        dPrintMatrix (Atest1,MSIZE,MSIZE);
        printf ("\n");
        dPrintMatrix (Atest2,MSIZE,MSIZE);
        printf ("\n");
        */
    }
    printf ("\tmaximum difference = %.6e - %s\n",(double)maxdiff,
        maxdiff > tol ? "FAILED" : "passed");

    dFree(d2, vectorSize);
    dFree(d, vectorSize);
    dFree(Atest2, matrixSize);
    dFree(Atest1, matrixSize);
    dFree(DL2, matrixSize);
    dFree(L2, matrixSize);
    dFree(L, matrixSize);
    dFree(A, matrixSize);
}

//****************************************************************************
// test mass stuff

#define NUMP 10		// number of particles

#if 0
static void printMassParams (dMass *m)
{
    printf ("mass = %.4f\n",m->mass);
    printf ("com  = (%.4f,%.4f,%.4f)\n",m->c[0],m->c[1],m->c[2]);
    printf ("I    = [ %10.4f %10.4f %10.4f ]\n"
        "       [ %10.4f %10.4f %10.4f ]\n"
        "       [ %10.4f %10.4f %10.4f ]\n",
        m->_I(0,0),m->_I(0,1),m->_I(0,2),
        m->_I(1,0),m->_I(1,1),m->_I(1,2),
        m->_I(2,0),m->_I(2,1),m->_I(2,2));
}
#endif

static void compareMassParams (dMass *m1, dMass *m2, const char *msg)
{
    int i,j,ok = 1;
    if (!(cmp(m1->mass,m2->mass) && cmp(m1->c[0],m2->c[0]) &&
        cmp(m1->c[1],m2->c[1]) && cmp(m1->c[2],m2->c[2])))
        ok = 0;
    for (i=0; i<3; i++) for (j=0; j<3; j++)
        if (cmp (m1->_I(i,j),m2->_I(i,j))==0) ok = 0;
    if (ok) printf ("\tpassed (%s)\n",msg); else printf ("\tFAILED (%s)\n",msg);
}


// compute the mass parameters of a particle set

static void computeMassParams (dMass *m, dReal q[NUMP][3], dReal pm[NUMP])
{
    int i,j;
    dMassSetZero (m);
    for (i=0; i<NUMP; i++) {
        m->mass += pm[i];
        for (j=0; j<3; j++) m->c[j] += pm[i]*q[i][j];
        m->_I(0,0) += pm[i]*(q[i][1]*q[i][1] + q[i][2]*q[i][2]);
        m->_I(1,1) += pm[i]*(q[i][0]*q[i][0] + q[i][2]*q[i][2]);
        m->_I(2,2) += pm[i]*(q[i][0]*q[i][0] + q[i][1]*q[i][1]);
        m->_I(0,1) -= pm[i]*(q[i][0]*q[i][1]);
        m->_I(0,2) -= pm[i]*(q[i][0]*q[i][2]);
        m->_I(1,2) -= pm[i]*(q[i][1]*q[i][2]);
    }
    for (j=0; j<3; j++) m->c[j] /= m->mass;
    m->_I(1,0) = m->_I(0,1);
    m->_I(2,0) = m->_I(0,2);
    m->_I(2,1) = m->_I(1,2);
}


static void testMassFunctions()
{
    dMass m;
    int i,j;
    dReal q[NUMP][3];		// particle positions
    dReal pm[NUMP];		// particle masses
    dMass m1,m2;
    dMatrix3 R;

    HEADER;

    printf ("\t");
    dMassSetZero (&m);
    TRAP_MESSAGE (dMassSetParameters (&m,10, 0,0,0, 1,2,3, 4,5,6),
        printf (" FAILED (1)\n"), printf (" passed (1)\n"));

    printf ("\t");
    dMassSetZero (&m);
    TRAP_MESSAGE (dMassSetParameters (&m,10, REAL(0.1),REAL(0.2),REAL(0.15), 3,5,14, REAL(3.1),REAL(3.2),4),
        printf ("passed (2)\n") , printf (" FAILED (2)\n"));
    if (m.mass==10 && m.c[0]==REAL(0.1) && m.c[1]==REAL(0.2) &&
        m.c[2]==REAL(0.15) && m._I(0,0)==3 && m._I(1,1)==5 && m._I(2,2)==14 &&
        m._I(0,1)==REAL(3.1) && m._I(0,2)==REAL(3.2) && m._I(1,2)==4 &&
        m._I(1,0)==REAL(3.1) && m._I(2,0)==REAL(3.2) && m._I(2,1)==4)
        printf ("\tpassed (3)\n"); else printf ("\tFAILED (3)\n");

    dMassSetZero (&m);
    dMassSetSphere (&m,REAL(1.4), REAL(0.86));
    if (cmp(m.mass,REAL(3.73002719949386)) && m.c[0]==0 && m.c[1]==0 && m.c[2]==0 &&
        cmp(m._I(0,0),REAL(1.10349124669826)) &&
        cmp(m._I(1,1),REAL(1.10349124669826)) &&
        cmp(m._I(2,2),REAL(1.10349124669826)) &&
        m._I(0,1)==0 && m._I(0,2)==0 && m._I(1,2)==0 &&
        m._I(1,0)==0 && m._I(2,0)==0 && m._I(2,1)==0)
        printf ("\tpassed (4)\n"); else printf ("\tFAILED (4)\n");

    dMassSetZero (&m);
    dMassSetCapsule (&m,REAL(1.3),1,REAL(0.76),REAL(1.53));
    if (cmp(m.mass,REAL(5.99961928996029)) && m.c[0]==0 && m.c[1]==0 && m.c[2]==0 &&
        cmp(m._I(0,0),REAL(1.59461986077384)) &&
        cmp(m._I(1,1),REAL(4.21878433864904)) &&
        cmp(m._I(2,2),REAL(4.21878433864904)) &&
        m._I(0,1)==0 && m._I(0,2)==0 && m._I(1,2)==0 &&
        m._I(1,0)==0 && m._I(2,0)==0 && m._I(2,1)==0)
        printf ("\tpassed (5)\n"); else printf ("\tFAILED (5)\n");

    dMassSetZero (&m);
    dMassSetBox (&m,REAL(0.27),3,4,5);
    if (cmp(m.mass,REAL(16.2)) && m.c[0]==0 && m.c[1]==0 && m.c[2]==0 &&
        cmp(m._I(0,0),REAL(55.35)) && cmp(m._I(1,1),REAL(45.9)) && cmp(m._I(2,2),REAL(33.75)) &&
        m._I(0,1)==0 && m._I(0,2)==0 && m._I(1,2)==0 &&
        m._I(1,0)==0 && m._I(2,0)==0 && m._I(2,1)==0)
        printf ("\tpassed (6)\n"); else printf ("\tFAILED (6)\n");

    // test dMassAdjust?

    // make random particles and compute the mass, COM and inertia, then
    // translate and repeat.
    for (i=0; i<NUMP; i++) {
        pm[i] = dRandReal()+REAL(0.5);
        for (j=0; j<3; j++) {
            q[i][j] = REAL(2.0)*(dRandReal()-REAL(0.5));
        }
    }
    computeMassParams (&m1,q,pm);
    memcpy (&m2,&m1,sizeof(dMass));
    dMassTranslate (&m2,1,2,-3);
    for (i=0; i<NUMP; i++) {
        q[i][0] += 1;
        q[i][1] += 2;
        q[i][2] -= 3;
    }
    computeMassParams (&m1,q,pm);
    compareMassParams (&m1,&m2,"7");

    // rotate the masses
    _R(0,0) = REAL(-0.87919618797635);
    _R(0,1) = REAL( 0.15278881840384);
    _R(0,2) = REAL(-0.45129772879842);
    _R(1,0) = REAL(-0.47307856232664);
    _R(1,1) = REAL(-0.39258064912909);
    _R(1,2) = REAL( 0.78871864932708);
    _R(2,0) = REAL(-0.05666336483842);
    _R(2,1) = REAL( 0.90693771059546);
    _R(2,2) = REAL( 0.41743652473765);
    dMassRotate (&m2,R);
    for (i=0; i<NUMP; i++) {
        dReal a[3];
        dMultiply0 (a,&_R(0,0),&q[i][0],3,3,1);
        q[i][0] = a[0];
        q[i][1] = a[1];
        q[i][2] = a[2];
    }
    computeMassParams (&m1,q,pm);
    compareMassParams (&m1,&m2,"8");
}

//****************************************************************************
// test rotation stuff

static void makeRandomRotation (dMatrix3 R)
{
    dReal *u1 = R, *u2=R+4, *u3=R+8;
    dMakeRandomVector (u1,3,1.0);
    dNormalize3 (u1);
    dMakeRandomVector (u2,3,1.0);
    dReal d = dCalcVectorDot3(u1,u2);
    u2[0] -= d*u1[0];
    u2[1] -= d*u1[1];
    u2[2] -= d*u1[2];
    dNormalize3(u2);
    dCalcVectorCross3(u3,u1,u2);
}


static void testRtoQandQtoR()
{
    HEADER;
    dMatrix3 R,I,R2;
    dQuaternion q;
    int i;

    // test makeRandomRotation()
    makeRandomRotation (R);
    dMultiply2 (I,R,R,3,3,3);
    printf ("\tmakeRandomRotation() - %s (1)\n",
        cmpIdentityMat3(I) ? "passed" : "FAILED");

    // test QtoR() on random normalized quaternions
    int ok = 1;
    for (i=0; i<100; i++) {
        dMakeRandomVector (q,4,1.0);
        dNormalize4 (q);
        dQtoR (q,R);
        dMultiply2 (I,R,R,3,3,3);
        if (cmpIdentityMat3(I)==0) ok = 0;
    }
    printf ("\tQtoR() orthonormality %s (2)\n", ok ? "passed" : "FAILED");

    // test R -> Q -> R works
    dReal maxdiff=0;
    for (i=0; i<100; i++) {
        makeRandomRotation (R);
        dRtoQ (R,q);
        dQtoR (q,R2);
        dReal diff = dMaxDifference (R,R2,3,3);
        if (diff > maxdiff) maxdiff = diff;
    }
    printf ("\tmaximum difference = %e - %s (3)\n",(double)maxdiff,
        (maxdiff > tol) ? "FAILED" : "passed");
}


static void testQuaternionMultiply()
{
    HEADER;
    dMatrix3 RA,RB,RC,Rtest;
    dQuaternion qa,qb,qc;
    dReal diff,maxdiff=0;

    for (int i=0; i<100; i++) {
        makeRandomRotation (RB);
        makeRandomRotation (RC);
        dRtoQ (RB,qb);
        dRtoQ (RC,qc);

        dMultiply0 (RA,RB,RC,3,3,3);
        dQMultiply0 (qa,qb,qc);
        dQtoR (qa,Rtest);
        diff = dMaxDifference (Rtest,RA,3,3);
        if (diff > maxdiff) maxdiff = diff;

        dMultiply1 (RA,RB,RC,3,3,3);
        dQMultiply1 (qa,qb,qc);
        dQtoR (qa,Rtest);
        diff = dMaxDifference (Rtest,RA,3,3);
        if (diff > maxdiff) maxdiff = diff;

        dMultiply2 (RA,RB,RC,3,3,3);
        dQMultiply2 (qa,qb,qc);
        dQtoR (qa,Rtest);
        diff = dMaxDifference (Rtest,RA,3,3);
        if (diff > maxdiff) maxdiff = diff;

        dMultiply0 (RA,RC,RB,3,3,3);
        transpose3x3 (RA);
        dQMultiply3 (qa,qb,qc);
        dQtoR (qa,Rtest);
        diff = dMaxDifference (Rtest,RA,3,3);
        if (diff > maxdiff) maxdiff = diff;
    }
    printf ("\tmaximum difference = %e - %s\n",(double)maxdiff,
        (maxdiff > tol) ? "FAILED" : "passed");
}


static void testRotationFunctions()
{
    dMatrix3 R1;
    HEADER;

    printf ("\tdRSetIdentity - ");
    dMakeRandomMatrix (R1,3,3,1.0);
    dRSetIdentity (R1);
    if (cmpIdentityMat3(R1)) printf ("passed\n"); else printf ("FAILED\n");

    printf ("\tdRFromAxisAndAngle - ");

    printf ("\n");
    printf ("\tdRFromEulerAngles - ");

    printf ("\n");
    printf ("\tdRFrom2Axes - ");

    printf ("\n");
}

//****************************************************************************

#include <assert.h>

template<class T>
class simplevector
{
private:
    int n;
    int max;
    T* data;

public:
    simplevector() { initialize(); }
    ~simplevector() { finalize(); }
    T& operator[](int i) { assert(i>=0 && i<n); return data[i]; }
    const T& operator[](int i) const { assert(i>=0 && i<n); return data[i]; }
    void push_back(const T& elem)
    {
        if (n == max)
        {
            max *= 2;
            T* newdata = new T[(size_t)max];
            memcpy(newdata, data, sizeof(T)*(size_t)n);
            delete[] data;
            data = newdata;
        }
        data[n++] = elem;
    }
    int size() const { return n; }
    void clear() { finalize(); initialize(); }

private:
    void finalize() { delete[] data; }
    void initialize() { data = new T[32]; max = 32; n = 0; }
};

// matrix header on the stack

class dMatrixComparison {
    struct dMatInfo;
    simplevector<dMatInfo*> mat;
    int afterfirst,index;

public:
    dMatrixComparison();
    ~dMatrixComparison();

    dReal nextMatrix (dReal *A, int n, int m, int lower_tri, const char *name, ...);
    // add a new n*m matrix A to the sequence. the name of the matrix is given
    // by the printf-style arguments (name,...). if this is the first sequence
    // then this object will simply record the matrices and return 0.
    // if this the second or subsequent sequence then this object will compare
    // the matrices with the first sequence, and report any differences.
    // the matrix error will be returned. if `lower_tri' is 1 then only the
    // lower triangle of the matrix (including the diagonal) will be compared
    // (the matrix must be square).

    void end();
    // end a sequence.

    void reset();
    // restarts the object, so the next sequence will be the first sequence.

    void dump();
    // print out info about all the matrices in the sequence
};

struct dMatrixComparison::dMatInfo {
    int n,m;		// size of matrix
    char name[128];	// name of the matrix
    dReal *data;		// matrix data
    int size;		// size of `data'
};



dMatrixComparison::dMatrixComparison()
{
    afterfirst = 0;
    index = 0;
}


dMatrixComparison::~dMatrixComparison()
{
    reset();
}


dReal dMatrixComparison::nextMatrix (dReal *A, int n, int m, int lower_tri,
                                     const char *name, ...)
{
    if (A==NULL || n < 1 || m < 1 || name==NULL) dDebug (false,"bad args to nextMatrix");
    unsigned num = (unsigned)n*dPAD(m);

    if (afterfirst==0) {
        dMatInfo *mi = (dMatInfo*) dAlloc (sizeof(dMatInfo));
        mi->n = n;
        mi->m = m;
        mi->size = (int)(num * sizeof(dReal));
        mi->data = (dReal*) dAlloc ((dsizeint)mi->size);
        memcpy (mi->data,A,(size_t)mi->size);

        va_list ap;
        va_start (ap,name);
        vsprintf (mi->name,name,ap);
        va_end (ap);
        if (strlen(mi->name) >= sizeof (mi->name)) dDebug (0,"name too long");

        mat.push_back(mi);
        return 0;
    }
    else {
        if (lower_tri && n != m)
            dDebug (0,"dMatrixComparison, lower triangular matrix must be square");
        if (index >= mat.size()) dDebug (0,"dMatrixComparison, too many matrices");
        dMatInfo *mp = mat[index];
        index++;

        dMatInfo mi;
        va_list ap;
        va_start (ap,name);
        vsprintf (mi.name,name,ap);
        va_end (ap);
        if (strlen(mi.name) >= sizeof (mi.name)) dDebug (0,"name too long");

        if (strcmp(mp->name,mi.name) != 0)
            dDebug (0,"dMatrixComparison, name mismatch (\"%s\" and \"%s\")",
            mp->name,mi.name);
        if (mp->n != n || mp->m != m)
            dDebug (0,"dMatrixComparison, size mismatch (%dx%d and %dx%d)",
            mp->n,mp->m,n,m);

        dReal maxdiff;
        if (lower_tri) {
            maxdiff = dMaxDifferenceLowerTriangle (A,mp->data,n);
        }
        else {
            maxdiff = dMaxDifference (A,mp->data,n,m);
        }
        if (maxdiff > tol)
            dDebug (0,"dMatrixComparison, matrix error (size=%dx%d, name=\"%s\", "
            "error=%.4e)",n,m,mi.name,(double)maxdiff);
        return maxdiff;
    }
}


void dMatrixComparison::end()
{
    if (mat.size() <= 0) dDebug (0,"no matrices in sequence");
    afterfirst = 1;
    index = 0;
}


void dMatrixComparison::reset()
{
    for (int i=0; i<mat.size(); i++) {
        dFree (mat[i]->data,(dsizeint)mat[i]->size);
        dFree (mat[i],sizeof(dMatInfo));
    }
    mat.clear();
    afterfirst = 0;
    index = 0;
}


void dMatrixComparison::dump()
{
    for (int i=0; i<mat.size(); i++)
        printf ("%d: %s (%dx%d)\n",i,mat[i]->name,mat[i]->n,mat[i]->m);
}

//****************************************************************************
// unit test


// static jmp_buf jump_buffer;

static ODE_NORETURN_VOID myDebug (int /*num*/, const char* /*msg*/, va_list /*ap*/)
{
    // printf ("(Error %d: ",num);
    // vprintf (msg,ap);
    // printf (")\n");
    longjmp (jump_buffer,1);
}


static void dTestMatrixComparison()
{
    int i;
    printf ("dTestMatrixComparison()\n");
    dMessageFunction *orig_debug = dGetDebugHandler();

    dMatrixComparison mc;
    dReal A[50*50];

    // make first sequence
    unsigned long seed = dRandGetSeed();
    for (i=1; i<49;) {
        dMakeRandomMatrix (A,i,i+1,1.0);
        mc.nextMatrix (A,i,i+1,0,"A%d",i);
        i=i+1; // in loop body because of C++20 volatile deprecation
    }
    mc.end();

    //mc.dump();

    // test identical sequence
    dSetDebugHandler (&myDebug);
    dRandSetSeed (seed);
    if (setjmp (jump_buffer)) {
        printf ("\tFAILED (1)\n");
    }
    else {
        for (i=1; i<49;) {
            dMakeRandomMatrix (A,i,i+1,1.0);
            mc.nextMatrix (A,i,i+1,0,"A%d",i);
            i=i+1; // in loop body because of C++20 volatile deprecation
        }
        mc.end();
        printf ("\tpassed (1)\n");
    }
    dSetDebugHandler (orig_debug);

    // test broken sequences (with matrix error)
    dRandSetSeed (seed);
    volatile unsigned passcount = 0;
    for (i=1; i<49;) {
        if (setjmp (jump_buffer)) {
            passcount = passcount + 1;
        }
        else {
            dSetDebugHandler (&myDebug);
            dMakeRandomMatrix (A,i,i+1,1.0);
            A[((unsigned int)i-1)*dPAD(i+1)+(unsigned int)i] += REAL(0.01);
            mc.nextMatrix (A,i,i+1,0,"A%d",i);
            dSetDebugHandler (orig_debug);
        }
        i=i+1; // in loop body because of C++20 volatile deprecation
    }
    mc.end();
    printf ("\t%s (2)\n",(passcount == 48) ? "passed" : "FAILED");

    // test broken sequences (with name error)
    dRandSetSeed (seed);
    passcount = 0;
    for (i=1; i<49; ) {
        if (setjmp (jump_buffer)) {
            passcount = passcount + 1;
        }
        else {
            dSetDebugHandler (&myDebug);
            dMakeRandomMatrix (A,i,i+1,1.0);
            mc.nextMatrix (A,i,i+1,0,"B%d",i);
            dSetDebugHandler (orig_debug);
        }
        i=i+1; // in loop body because of C++20 volatile deprecation
    }
    mc.end();
    printf ("\t%s (3)\n",(passcount == 48) ? "passed" : "FAILED");

    // test identical sequence again
    dSetDebugHandler (&myDebug);
    dRandSetSeed (seed);
    if (setjmp (jump_buffer)) {
        printf ("\tFAILED (4)\n");
    }
    else {
        for (i=1; i<49; ) {
            dMakeRandomMatrix (A,i,i+1,1.0);
            mc.nextMatrix (A,i,i+1,0,"A%d",i);
            i=i+1; // in loop body because of C++20 volatile deprecation
        }
        mc.end();
        printf ("\tpassed (4)\n");
    }
    dSetDebugHandler (orig_debug);
}

//****************************************************************************

// internal unit tests
extern "C" void dTestDataStructures();
extern "C" int dTestSolveLCP();


int main()
{
    dInitODE();
    testRandomNumberGenerator();
    testInfinity();
    testPad();
    testCrossProduct();
    testSetZero();
    testNormalize3();
    //testReorthonormalize();     ... not anymore
    testPlaneSpace();
    testMatrixMultiply();
    testSmallMatrixMultiply();
    testCholeskyFactorization();
    testCholeskySolve();
    testInvertPDMatrix();
    testIsPositiveDefinite();
    testFastLDLTFactorization();
    testCoopLDLTFactorization();
    testSolveLDLT();
    testCoopSolveLDLT();
    testLDLTAddTL();
    testLDLTRemove();
    testMassFunctions();
    testRtoQandQtoR();
    testQuaternionMultiply();
    testRotationFunctions();
    dTestMatrixComparison();
    dTestSolveLCP();
    dTestDataStructures();
    dCloseODE();
    return 0;
}
