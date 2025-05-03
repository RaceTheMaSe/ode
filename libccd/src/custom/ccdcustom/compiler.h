#pragma once
/* this file should not be needed, issues with including the system libccd in custom build */
#if !defined(UNUSED_FUNCTION)
#ifdef __GNUC__
#define UNUSED_FUNCTION __attribute__((unused))
#else
#define UNUSED_FUNCTION
#endif
#endif /* UNUSED_FUNCTION */
