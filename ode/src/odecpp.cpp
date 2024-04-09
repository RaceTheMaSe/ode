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

/* C++ interface for non-collision stuff */

#ifdef __cplusplus
#include <ode/odecpp.h>

dWorldDynamicIDContainer::~dWorldDynamicIDContainer() {}

void dBodySimpleIDContainer::destroy() 
{ 
  if (_id) {
    dBodyDestroy(_id); 
    _id = NULL;
  }
}

dBodyDynamicIDContainer::~dBodyDynamicIDContainer() {}

void dJointGroupSimpleIDContainer::destroy()
{
  if (_id) {
    dJointGroupDestroy(_id); 
    _id = NULL;
  }
}

dJointGroupDynamicIDContainer::~dJointGroupDynamicIDContainer() {}

void dJointSimpleIDContainer::destroy()
{
  if (_id) {
    dJointDestroy (_id); 
    _id = NULL;
  }
}

dJointDynamicIDContainer::~dJointDynamicIDContainer() {}

#endif // __cplusplus
