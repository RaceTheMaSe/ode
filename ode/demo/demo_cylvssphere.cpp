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

// Test for cylinder vs sphere, by Bram Stolk

#include <ode/odeconfig.h>
#include <cassert>
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"


// dynamics and collision objects (chassis, 3 wheels, environment)

static dWorldID world;
static dSpaceID space;

static dBodyID cylbody;
static dGeomID cylgeom;

static dBodyID sphbody;
static dGeomID sphgeom;

static dJointGroupID contactgroup;

static bool show_contacts = true;

#define CYLRADIUS    REAL(0.6)
#define CYLLENGTH    REAL(2.0)
#define SPHERERADIUS REAL(0.5)

    
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawLine dsDrawLineD
#endif



// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  assert(o1);
  assert(o2);

  if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
  {
      (void)fprintf(stderr,"testing space %p %p\n", (void*)o1, (void*)o2);
    // colliding a space with something
    dSpaceCollide2(o1,o2,data,&nearCallback);
    // Note we do not want to test intersections within a space,
    // only between spaces.
    return;
  }

  const int N = 32;
  dContact contact[N];
  const int n = dCollide (o1,o2,N,&(contact[0].geom),sizeof(dContact));
  if (n > 0) 
  {
    for (int i=0; i<n; i++) 
    {
      contact[i].surface.mode = 0;
      contact[i].surface.mu = REAL(50.0); // was: dInfinity
      dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
      dJointAttach (c, dGeomGetBody(contact[i].geom.g1), dGeomGetBody(contact[i].geom.g2));
      if (show_contacts) 
      {
        dMatrix3 RI;
        dRSetIdentity (RI);
        const dReal ss[3] = {REAL(0.12),REAL(0.12),REAL(0.12)};
        dsSetColorAlpha (0,0,1,0.5);
        dsDrawBox (contact[i].geom.pos,RI,ss);
        const dReal *pos  = contact[i].geom.pos;
        const dReal depth = contact[i].geom.depth;
        const dReal *norm = contact[i].geom.normal;
        dReal endp[3] = {pos[0]+depth*norm[0], pos[1]+depth*norm[1], pos[2]+depth*norm[2]};
        dsSetColorAlpha (1,1,1,1);
        dsDrawLine (contact[i].geom.pos, endp);
      }
    }
  }
}


// start simulation - set viewpoint

static void start()
{
  dAllocateODEDataForThread((unsigned int)dAllocateMaskAll);

  float xyz[3] = {-8,-9,3};
  float hpr[3] = {45.0000f,-27.5000f,0.0000f};
  dsSetViewpoint (xyz,hpr);
}


// called when a key pressed

static void command (int cmd)
{
  (void)cmd;
}



// simulation loop

static void simLoop (int pause)
{
  dSpaceCollide (space,NULL,&nearCallback);
  if (!pause)
  {
    dWorldQuickStep (world, REAL(0.01)); // 100 Hz
  }
  dJointGroupEmpty (contactgroup);

  dsSetColorAlpha (1,1,0,0.5);

  const dReal *CPos = dBodyGetPosition(cylbody);
  const dReal *CRot = dBodyGetRotation(cylbody);
  float cpos[3] = {static_cast<float>(CPos[0]), static_cast<float>(CPos[1]), static_cast<float>(CPos[2])};
  float crot[12] = { static_cast<float>(CRot[0]), static_cast<float>(CRot[1]), static_cast<float>(CRot[2]), static_cast<float>(CRot[3]), static_cast<float>(CRot[4]), static_cast<float>(CRot[5]), static_cast<float>(CRot[6]), static_cast<float>(CRot[7]), static_cast<float>(CRot[8]), static_cast<float>(CRot[9]), static_cast<float>(CRot[10]), static_cast<float>(CRot[11]) };
  dsDrawCylinder
  (
    cpos,
    crot,
    (float)CYLLENGTH,
    (float)CYLRADIUS
  ); // single precision

  const dReal *SPos = dBodyGetPosition(sphbody);
  const dReal *SRot = dBodyGetRotation(sphbody);
  float spos[3] = {static_cast<float>(SPos[0]), static_cast<float>(SPos[1]), static_cast<float>(SPos[2])};
  float srot[12] = { static_cast<float>(SRot[0]), static_cast<float>(SRot[1]), static_cast<float>(SRot[2]), static_cast<float>(SRot[3]), static_cast<float>(SRot[4]), static_cast<float>(SRot[5]), static_cast<float>(SRot[6]), static_cast<float>(SRot[7]), static_cast<float>(SRot[8]), static_cast<float>(SRot[9]), static_cast<float>(SRot[10]), static_cast<float>(SRot[11]) };
  dsDrawSphere
  (
    spos,
    srot,
    SPHERERADIUS
  ); // single precision
}


int main (int argc, char **argv)
{
  dMass m;

  // setup pointers to drawstuff callback functions
  dsFunctions fn;
  fn.version = DS_VERSION;
  fn.start = &start;
  fn.step = &simLoop;
  fn.command = &command;
  fn.stop = NULL;
  fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

  // create world
  dInitODE2(0);
  world = dWorldCreate();
  space = dHashSpaceCreate (NULL);
  contactgroup = dJointGroupCreate (0);
  dWorldSetGravity (world,0,0,REAL(-9.8));
  dWorldSetQuickStepNumIterations (world, 32);

  dCreatePlane (space,0,0,1, 0.0);

  cylbody = dBodyCreate (world);
  dQuaternion q;
#if 0
  dQFromAxisAndAngle (q,1,0,0,dReal(M_PI*0.5));
#else
//  dQFromAxisAndAngle (q,1,0,0, dReal(M_PI * 1.0));
  dQFromAxisAndAngle (q,1,0,0, dReal(M_PI * -0.77));
#endif
  dBodySetQuaternion (cylbody,q);
  dMassSetCylinder (&m,1.0,3,CYLRADIUS,CYLLENGTH);
  dBodySetMass (cylbody,&m);
  cylgeom = dCreateCylinder(NULL, CYLRADIUS, CYLLENGTH);
  dGeomSetBody (cylgeom,cylbody);
  dBodySetPosition (cylbody, 0, 0, 3);
  dSpaceAdd (space, cylgeom);

  sphbody = dBodyCreate (world);
  dMassSetSphere (&m,1,SPHERERADIUS);
  dBodySetMass (sphbody,&m);
  sphgeom = dCreateSphere(NULL, SPHERERADIUS);
  dGeomSetBody (sphgeom,sphbody);
  dBodySetPosition (sphbody, 0, 0, REAL(5.5));
  dSpaceAdd (space, sphgeom);

  // run simulation
  dsSimulationLoop (argc, argv, DS_SIMULATION_DEFAULT_WIDTH, DS_SIMULATION_DEFAULT_HEIGHT, &fn);

  dJointGroupEmpty (contactgroup);
  dJointGroupDestroy (contactgroup);

  dGeomDestroy(sphgeom);
  dGeomDestroy (cylgeom);

  dSpaceDestroy (space);
  dWorldDestroy (world);
  dCloseODE();
  return 0;
}



