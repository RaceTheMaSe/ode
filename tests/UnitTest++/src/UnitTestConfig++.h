#ifndef UNITTEST_CONFIG_H
#define UNITTEST_CONFIG_H

// Standard defines documented here: http://predef.sourceforge.net

#if defined _MSC_VER
    #pragma warning(disable:4127) // conditional expression is constant
	#pragma warning(disable:4702) // unreachable code
	#pragma warning(disable:4722) // destructor never returns, potential memory leak
#endif

#if defined(unix) || defined(__unix__) || defined(__unix) || defined(linux) || \
    defined(__APPLE__) || defined (__NetBSD__) || defined (__OpenBSD__) || defined (__FreeBSD__)        
    #define UNITTEST_POSIX
#endif

#if defined (__MINGW32__)
    #define UNITTEST_MINGW
#endif

// by default, MemoryOutStream is implemented in terms of std::ostringstream.
// uncomment this line to use the custom MemoryOutStream (no deps on std::ostringstream).
//#define UNITTEST_USE_CUSTOM_STREAMS

#if defined(__GNUC__)
  #define NORETURN_VOID void __attribute__((noreturn))
#elif defined(_MSC_VER)
  #define NORETURN_VOID __declspec(noreturn) void
#else
  #define NORETURN_VOID void
#endif

#if defined(__cplusplus) && __cplusplus >= 201103L
#define OVERRIDE override
#define NOEXCEPT noexcept
#else
#define OVERRIDE
#define NOEXCEPT throw()
#endif

#endif
