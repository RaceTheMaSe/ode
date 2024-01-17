
// Should be included by Opcode.h if needed

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __OPC_ICEHOOK_H__
#define __OPC_ICEHOOK_H__

	#define ICE_DONT_CHECK_COMPILER_OPTIONS

	// From Windows...
	typedef int                 BOOL;
	#ifndef FALSE
	#define FALSE               0
	#endif

	#ifndef TRUE
	#define TRUE                1
	#endif

	#include <stdio.h>
	#include <stdlib.h>
	#include <assert.h>
	#include <string.h>
	#include <float.h>
	#include <math.h>

	#ifndef ASSERT
		#define	ASSERT(exp)	{} do{} while(0)
	#endif
	#define ICE_COMPILE_TIME_ASSERT(exp)	extern char ICE_Dummy[ (exp) ? 1 : -1 ]

	#define	Log				{}
	#define	SetIceError(a,b)	false
	#define	EC_OUTOFMEMORY	"Out of memory"

#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wheader-hygiene" // using namespace directive in global context in header
#endif

	extern OPCODE_NORETURN_VOID IceAbort();

	#include "Ice/IcePreprocessor.h"

	#undef ICECORE_API
	#define ICECORE_API	OPCODE_API

	#include "Ice/IceTypes.h"
	#include "Ice/IceFPU.h"
	#include "Ice/IceMemoryMacros.h"

	namespace IceCore
	{
		#include "Ice/IceUtils.h"
		#include "Ice/IceContainer.h"
		#include "Ice/IcePairs.h"
		#include "Ice/IceRevisitedRadix.h"
		#include "Ice/IceRandom.h"
	}
	using namespace IceCore;

	#define ICEMATHS_API	OPCODE_API
	namespace IceMaths
	{
		#include "Ice/IceAxes.h"
		#include "Ice/IcePoint.h"
		#include "Ice/IceHPoint.h"
		#include "Ice/IceMatrix3x3.h"
		#include "Ice/IceMatrix4x4.h"
		#include "Ice/IcePlane.h"
		#include "Ice/IceRay.h"
		#include "Ice/IceIndexedTriangle.h"
		#include "Ice/IceTriangle.h"
		#include "Ice/IceTriList.h"
		#include "Ice/IceAABB.h"
		#include "Ice/IceOBB.h"
		#include "Ice/IceBoundingSphere.h"
		#include "Ice/IceSegment.h"
		#include "Ice/IceLSS.h"
	}
	using namespace IceMaths;

#if defined(__clang__)
#pragma clang diagnostic pop
#endif

#endif // __OPC_ICEHOOK_H__
