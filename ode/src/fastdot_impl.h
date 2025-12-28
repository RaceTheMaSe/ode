

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

#ifndef ODE_FASTDOT_IMPL_H
#define ODE_FASTDOT_IMPL_H

#include "ode/common.h"

template<unsigned b_stride>
dReal calculateLargeVectorDot (const dReal *a, const dReal *b, unsigned n)
{
    dReal sum = 0;
    const dReal *a_end = a + (n & (unsigned int)(~3));
    for (; a != a_end; b += 4 * (size_t)b_stride, a += 4) {
        const dReal p0 = a[0], p1 = a[1], p2 = a[2], p3 = a[3];
        const dReal q0 = b[0 * (size_t)b_stride], q1 = b[1 * (size_t)b_stride], q2 = b[2 * (size_t)b_stride], q3 = b[3 * (size_t)b_stride];
        const dReal m0 = p0 * q0;
        const dReal m1 = p1 * q1;
        const dReal m2 = p2 * q2;
        const dReal m3 = p3 * q3;
        sum += m0 + m1 + m2 + m3;
    }
    a_end += (n & 3);
    for (; a != a_end; b += b_stride, ++a) {
        sum += (*a) * (*b);
    }
    return sum;
}


#endif
