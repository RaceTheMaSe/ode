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

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawLine dsDrawLineD
#endif


static dWorldID world;
static dSpaceID space;
static dBodyID body1;
static dBodyID body2;
static dJointID joint1, joint2;
static bool applyForce = false;

static void start()
{
    world = dWorldCreate();
    dWorldSetGravity (world,0,0,REAL(-9.8));

    dWorldSetDamping(world, REAL(1e-4), REAL(1e-5));
//    dWorldSetERP(world, 1);

    space = dSimpleSpaceCreate (NULL);
    
    body1 = dBodyCreate(world);
    body2 = dBodyCreate(world);

    dBodySetPosition(body1, 0, 0, 3);
    dBodySetPosition(body2, 0, 0, 1);


    dGeomID g = {};
    dMass mass;
    
    g = dCreateBox(space, REAL(0.2), REAL(0.2), 1);
    dGeomSetBody(g, body1);
    dMassSetBox(&mass, 1, REAL(0.2), REAL(0.2), 1);
    dBodySetMass(body1, &mass);
    
    g = dCreateBox(space, REAL(0.2), REAL(0.2), 1);
    dGeomSetBody(g, body2);
    dMassSetBox(&mass, 1, REAL(0.2), REAL(0.2), 1);
    dBodySetMass(body2, &mass);

#if 1
    joint1 = dJointCreateDHinge(world, NULL);
    dJointAttach(joint1, body1, NULL);
    dJointSetDHingeAxis(joint1, 0, 1, 0);
    dJointSetDHingeAnchor1(joint1, 0, 0, 3.5);
    dJointSetDHingeAnchor2(joint1, 0, 0, 4.5);
#endif

#if 1
    joint2 = dJointCreateDHinge(world, NULL);
    dJointAttach(joint2, body1, body2);
    dJointSetDHingeAxis(joint2, 1, 0, 0);
    dJointSetDHingeAnchor1(joint2, 0, 0, 2.5);
    dJointSetDHingeAnchor2(joint2, 0, 0, 1.5);
#else
    joint2 = dJointCreateDBall(world, 0);
    dJointAttach(joint2, body1, body2);
    dJointSetDBallAnchor1(joint2, 0, 0, 2.5);
    dJointSetDBallAnchor2(joint2, 0, 0, 1.5);
#endif

    //dBodyAddForce(body1, 20, 0, 0);


    // initial camera position
    float xyz[3] = {3.8966f, -2.0614f, 4.0300f};
    float hpr[3] = {153.5, -16.5, 0};
    dsSetViewpoint (xyz,hpr);
}

static void stop()
{
    dSpaceDestroy(space);

    dWorldDestroy(world);
}


static void drawGeom(dGeomID g)
{
    const int gclass = dGeomGetClass(g);
    const dReal *pos = dGeomGetPosition(g);
    const dReal *rot = dGeomGetRotation(g);

    if(gclass == dBoxClass) {
        dVector3 lengths;
        if (applyForce)
            dsSetColor(1, .5, 0);
        else
            dsSetColor(1, 1, 0);
        dsSetTexture (DS_WOOD);
        dGeomBoxGetLengths(g, lengths);
        dsDrawBox(pos, rot, lengths);
    }
}


static void simLoop(int pause)
{
    if (!pause) {

        static dReal t = 0;

        const dReal step = REAL(0.005);
        const unsigned nsteps = 2;

        for (unsigned i=0; i<nsteps; ++i) {

            applyForce = fmodf((float)t, 3.f) > 2.f;

            if (applyForce) {
                const dReal f = REAL(0.3) * dSin(t*REAL(1.2));
                dBodyAddForceAtRelPos(body1,
                                      f, 0, 0, 
                                      0, 0, -0.5); // at the lower end
                
                const dReal g = REAL(0.3) * dSin(t*REAL(0.7));
                dBodyAddForceAtRelPos(body2,
                                      0, g, 0, 
                                      0, 0, -0.5); // at the lower end
            }
            
            t += step;
            if (t > REAL(20.))
                t = 0.;

            dWorldQuickStep(world, step);
        }
    }
    
    // now we draw everything
    const unsigned ngeoms = (unsigned)dSpaceGetNumGeoms(space);
    for (unsigned i=0; i<ngeoms; ++i) {
        dGeomID g = dSpaceGetGeom(space, (int)i);

        drawGeom(g);
    }

#if 1
    dVector3 a11, a12;
    dJointGetDHingeAnchor1(joint1, a11);
    dJointGetDHingeAnchor2(joint1, a12);
    dsSetColor(1, 0, 0);
    dsDrawLine(a11, a12);
    //printf("Error 1: %f\n", fabs(dJointGetDHingeDistance(joint1) - dCalcPointsDistance3(a11, a12)));
#endif

#if 1
    dVector3 a21, a22;
    dJointGetDHingeAnchor1(joint2, a21);
    dJointGetDHingeAnchor2(joint2, a22);
    dsSetColor(0, 1, 0);
    dsDrawLine(a21, a22);

    //printf("Error 2: %f\n", fabs(dJointGetDHingeDistance(joint2) - dCalcPointsDistance3(a21, a22)));
#endif
}



int main(int argc, char **argv)
{
    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.command = NULL;
    fn.stop = stop;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;
    
    // create world
    dInitODE();

    // run demo
    dsSimulationLoop (argc, argv, DS_SIMULATION_DEFAULT_WIDTH, DS_SIMULATION_DEFAULT_HEIGHT, &fn);

    dCloseODE();
    return 0;
}
