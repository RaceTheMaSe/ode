/***
 * libccd
 * ---------------------------------
 * Copyright (c)2010 Daniel Fiser <danfis@danfis.cz>
 *
 *
 *  This file is part of libccd.
 *
 *  Distributed under the OSI-approved BSD License (the "License");
 *  see accompanying file BDS-LICENSE for details or see
 *  <http://www.opensource.org/licenses/bsd-license.php>.
 *
 *  This software is distributed WITHOUT ANY WARRANTY; without even the
 *  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *  See the License for more information.
 */

#ifndef __CCD_DBG_H__
#define __CCD_DBG_H__

/**
 * Some macros which can be used for printing debug info to stderr if macro
 * NDEBUG not defined.
 *
 * DBG_PROLOGUE can be specified as string and this string will be
 * prepended to output text
 */
#ifndef NDEBUG

#include <stdio.h>

#ifndef DBG_PROLOGUE
# define DBG_PROLOGUE
#endif

#if defined(__STDC__) && defined(__STDC_VERSION__) && __STDC_VERSION__ >= 199901L
# define DBG(format, ...) do { \
    fprintf(stderr, DBG_PROLOGUE "%s :: " format "\n", __func__, ## __VA_ARGS__); \
    fflush(stderr); \
    } while (0)
#else
#include "compiler.h"
#include <stdarg.h>
#include <string.h>
/* no association to code line prior to C99 implemented */
_ccd_inline UNUSED_FUNCTION void DBG(const char* format, ...)
{
    char buffer[1024] = {0};
    va_list ap;
    va_start(ap, format);
    strcat(buffer,"%s :: ");
    strcat(buffer,format);
    strcat(buffer,"\n");
    fprintf(stderr, buffer, ap);
    fflush(stderr);
    va_end(ap);
}
#endif

# define DBG2(str) do { \
    fprintf(stderr, DBG_PROLOGUE "%s :: " str "\n", __func__); \
    fflush(stderr); \
    } while (0)

# define DBG_VEC3(vec, prefix) do {\
    fprintf(stderr, DBG_PROLOGUE "%s :: %s[%lf %lf %lf]\n", \
            __func__, prefix, ccdVec3X(vec), ccdVec3Y(vec), ccdVec3Z(vec)); \
    fflush(stderr); \
    } while (0)
/*
# define DBG_VEC3(vec, prefix) do {\
    fprintf(stderr, DBG_PROLOGUE "%s :: %s[%.20lf %.20lf %.20lf]\n", \
            __func__, prefix, ccdVec3X(vec), ccdVec3Y(vec), ccdVec3Z(vec)); \
    fflush(stderr); \
    } while (0)
*/

#else
#if defined(__STDC__) && defined(__STDC_VERSION__) && __STDC_VERSION__ >= 199901L
# define DBG(format, ...)
#else
_ccd_inline UNUSED_FUNCTION void DBG(const char* format, ...) { (void)format; }
#endif
# define DBG2(str)
# define DBG_VEC3(v, prefix)
#endif

#endif /* __CCD_DBG_H__ */
