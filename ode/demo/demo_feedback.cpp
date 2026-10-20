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

// Test for breaking joints, by Bram Stolk

#include <ode/odeconfig.h>
#include <assert.h>
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"


#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#endif


// dynamics and collision objects (chassis, 3 wheels, environment)

static dWorldID world;
static dSpaceID space;

static const int   STACKCNT=10;	// nr of weights on bridge
static const int   SEGMCNT=16;	// nr of segments in bridge
static const dReal SEGMDIM[3] = { REAL(0.9), REAL(4.0), REAL(0.1) };

static dGeomID  groundgeom;
static dBodyID  segbodies[SEGMCNT];
static dGeomID  seggeoms[SEGMCNT];
static dBodyID  stackbodies[STACKCNT];
static dGeomID  stackgeoms[STACKCNT];
static dJointID hinges[SEGMCNT-1];
static dJointID sliders[2];
static dJointFeedback jfeedbacks[SEGMCNT-1];
static dReal colours[SEGMCNT];
static int stress[SEGMCNT-1];

static dJointGroupID contactgroup;


// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

static void nearCallback (void *data, dGeomID o1, dGeomID o2)
{
  assert(o1);
  assert(o2);

  if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
  {
      fprintf(stderr,"testing space %p %p\n", (void*)o1, (void*)o2);
    // colliding a space with something
    dSpaceCollide2(o1,o2,data,&nearCallback);
    // Note we do not want to test intersections within a space,
    // only between spaces.
    return;
  }

  const int N = 32;
  dContact contact[N];
  int n = dCollide (o1,o2,N,&(contact[0].geom),sizeof(dContact));
  if (n > 0) 
  {
    for (int i=0; i<n; i++) 
    {
      contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1;
      contact[i].surface.mu = 100.0;
      contact[i].surface.soft_erp = REAL(0.96);
      contact[i].surface.soft_cfm = REAL(0.02);
      dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
      dJointAttach (c,
		    dGeomGetBody(contact[i].geom.g1),
		    dGeomGetBody(contact[i].geom.g2));
    }
  }
}


// start simulation - set viewpoint

static void start()
{
  dAllocateODEDataForThread((unsigned int)dAllocateMaskAll);

  float xyz[3] = { -6, 8, 6};
  float hpr[3] = { -65.0f, -27.0f, 0.0f};
  dsSetViewpoint (xyz,hpr);
}


// called when a key pressed

static void command (int /*cmd*/)
{}


static void drawGeom (dGeomID g)
{
  const dReal *pos = dGeomGetPosition(g);
  const dReal *R   = dGeomGetRotation(g);

  int type = dGeomGetClass (g);
  if (type == dBoxClass)
  {
    dVector3 sides;
    dGeomBoxGetLengths (g, sides);
    dsDrawBox (pos,R,sides);
  }
  if (type == dCylinderClass)
  {
    dReal r,l;
    dGeomCylinderGetParams(g, &r, &l);
    dsDrawCylinder (pos, R, (float)l, (float)r);
  }
}


static void inspectJoints(void)
{
  const dReal forcelimit = 5000.0;
  int i;
  for (i=0; i<SEGMCNT-1; i++)
  {
    if (dJointGetBody(hinges[i], 0))
    {
      // This joint has not snapped already... inspect it.
      dReal l0 = dCalcVectorLength3(jfeedbacks[i].f1);
      dReal l1 = dCalcVectorLength3(jfeedbacks[i].f2);
      colours[i+0] = REAL(0.95)*colours[i+0] + REAL(0.05) * l0/forcelimit;
      colours[i+1] = REAL(0.95)*colours[i+1] + REAL(0.05) * l1/forcelimit;
      if (l0 > forcelimit || l1 > forcelimit)
        stress[i]++;
      else
        stress[i]=0;
      if (stress[i]>4)
      {
        // Low-pass filter the noisy feedback data.
        // Only after 4 consecutive timesteps with excessive load, snap.
        fprintf(stderr,"SNAP! (that was the sound of joint %d breaking)\n", i);
        dJointAttach (hinges[i], NULL, NULL);
      }
    }
  }
}


// simulation loop

static void simLoop (int pause)
{
  int i;

  double simstep = 0.002; // 2ms simulation steps
  double dt = dsElapsedTime();
  int nrofsteps = (int) ceilf((float)(dt/simstep));
  for (i=0; i<nrofsteps && !pause; i++)
  {
    dSpaceCollide (space,NULL,&nearCallback);
    dWorldQuickStep (world, (dReal)simstep);
    dJointGroupEmpty (contactgroup);
    inspectJoints();
  }

  for (i=0; i<SEGMCNT; i++)
  {
    float r=0,g=0,b=0.2f;
    float v = (float)colours[i];
    if (v>1.0f) v=1.0f;
    if (v<0.5f) 
    {
      r=2*v;
      g=1.0;
    }
    else
    {
      r=1.0;
      g=2*(1-v);
    }
    dsSetColor (r,g,b);
    drawGeom(seggeoms[i]);
  }
  dsSetColor (1,1,1);
  for (i=0; i<STACKCNT; i++)
    drawGeom(stackgeoms[i]);
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
  dWorldSetQuickStepNumIterations (world, 20);

  int i;
  for (i=0; i<SEGMCNT; i++)
  {
    segbodies[i] = dBodyCreate (world);
    dBodySetPosition(segbodies[i], dReal(i) - SEGMCNT/REAL(2.0), 0, 5);
    dMassSetBox (&m, 1, SEGMDIM[0], SEGMDIM[1], SEGMDIM[2]);
    dBodySetMass (segbodies[i], &m);
    seggeoms[i] = dCreateBox (NULL, SEGMDIM[0], SEGMDIM[1], SEGMDIM[2]);
    dGeomSetBody (seggeoms[i], segbodies[i]);
    dSpaceAdd (space, seggeoms[i]);
  }

  for (i=0; i<SEGMCNT-1; i++)
  {
    hinges[i] = dJointCreateHinge (world,NULL);
    dJointAttach (hinges[i], segbodies[i],segbodies[i+1]);
    dJointSetHingeAnchor (hinges[i], (dReal)i + REAL(0.5) - SEGMCNT/REAL(2.0), 0, 5);
    dJointSetHingeAxis (hinges[i], 0,1,0);
    dJointSetHingeParam (hinges[i],dParamFMax,  8000.0);
    // NOTE:
    // Here we tell ODE where to put the feedback on the forces for this hinge
    dJointSetFeedback (hinges[i], jfeedbacks+i);
    stress[i]=0;
  }

  for (i=0; i<STACKCNT; i++)
  {
    stackbodies[i] = dBodyCreate(world);
    dMassSetBox (&m, 2.0, 2, 2, REAL(0.6));
    dBodySetMass(stackbodies[i],&m);

    stackgeoms[i] = dCreateBox(NULL, 2, 2, REAL(0.6));
    dGeomSetBody(stackgeoms[i], stackbodies[i]);
    dBodySetPosition(stackbodies[i], 0,0,REAL(8.0)+REAL(2.0)*(dReal)i);
    dSpaceAdd(space, stackgeoms[i]);
  }

  sliders[0] = dJointCreateSlider (world,NULL);
  dJointAttach(sliders[0], segbodies[0], NULL);
  dJointSetSliderAxis (sliders[0], 1,0,0);
  dJointSetSliderParam (sliders[0],dParamFMax,  REAL(4000.0));
  dJointSetSliderParam (sliders[0],dParamLoStop,        0.0);
  dJointSetSliderParam (sliders[0],dParamHiStop,REAL(   0.2));

  sliders[1] = dJointCreateSlider (world,NULL);
  dJointAttach(sliders[1], segbodies[SEGMCNT-1], NULL);
  dJointSetSliderAxis (sliders[1], 1,0,0);
  dJointSetSliderParam (sliders[1],dParamFMax,  REAL(4000.0));
  dJointSetSliderParam (sliders[1],dParamLoStop,        0.0);
  dJointSetSliderParam (sliders[1],dParamHiStop,REAL(  -0.2));

  groundgeom = dCreatePlane(space, 0,0,1,0);

  for (i=0; i<SEGMCNT; i++)
    colours[i]=0.0;

  // run simulation
  dsSimulationLoop (argc, argv, DS_SIMULATION_DEFAULT_WIDTH, DS_SIMULATION_DEFAULT_HEIGHT, &fn);

  dJointGroupEmpty(contactgroup);
  dJointGroupDestroy (contactgroup);

  // First destroy seggeoms, then space, then the world.
  for (i=0; i<SEGMCNT; i++)
    dGeomDestroy (seggeoms[i]);
  for (i=0; i<STACKCNT; i++)
    dGeomDestroy (stackgeoms[i]);

  dSpaceDestroy(space);
  dWorldDestroy (world);
  dCloseODE();
  return 0;
}



