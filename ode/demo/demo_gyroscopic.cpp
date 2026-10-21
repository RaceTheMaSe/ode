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

// select correct drawing functions

#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#endif

static bool write_world = false;
static bool show_contacts = false;
static dWorld * world;
static dBody *top1, *top2;
static dSpace *space;
static dJointGroup* contactgroup;

static const dReal pinradius = REAL(0.05);
static const dReal pinlength = REAL(1.5);
static const dReal topradius = REAL(1.0);
static const dReal toplength = REAL(0.25);
// const dReal topmass = 1.0f;

#define MAX_CONTACTS 4

static void nearCallback (void * /*data*/, dGeomID o1, dGeomID o2)
{
    // for drawing the contact points
    dMatrix3 RI;
    dRSetIdentity (RI);
    const dReal ss[3] = {REAL(0.02),REAL(0.02),REAL(0.02)};

    int i;
    dBodyID b1 = dGeomGetBody(o1);
    dBodyID b2 = dGeomGetBody(o2);

    dContact contact[MAX_CONTACTS];
    const int numc = dCollide (o1,o2,MAX_CONTACTS,&contact[0].geom,
			    sizeof(dContact));

    for (i=0; i<numc; i++) {
        contact[i].surface.mode = dContactApprox1;
        contact[i].surface.mu = 2;

        dJointID c = dJointCreateContact (*world,*contactgroup,contact+i);
        dJointAttach (c,b1,b2);
        if (show_contacts)
            dsDrawBox (contact[i].geom.pos, RI, ss);

    }
}


// start simulation - set viewpoint

static void start()
{
  float xyz[3] = {4.777f, -2.084f, 2.18f};
  float hpr[3] = {153.0f, -14.5f, 0.0f};
  dsSetViewpoint (xyz,hpr);
  printf ("Orange top approximates conservation of angular momentum\n");
  printf ("Green top uses conservation of angular velocity\n");
  printf ("---\n");
  printf ("SPACE to reset\n");
  printf ("A to tilt the tops.\n");
  printf ("T to toggle showing the contact points.\n");
  printf ("1 to save the current state to 'state.dif'.\n");
}


static char locase (char c)
{
  if (c >= 'A' && c <= 'Z') return c - ('a'-'A');
  else return c;
}


// called when a key pressed
static void reset();
static void tilt();

static void command (int cmdIn)
{
    char cmd = locase ((char)cmdIn);
    if (cmd == ' ')
    {
        reset();       
    }
    else if (cmd == 'a') {
        tilt();
    }
    else if (cmd == 't') {
        show_contacts = !show_contacts;
    }
    else if (cmd == '1') {
        write_world = true;
    }
}

// simulation loop

static void simLoop (int pause)
{
    dsSetColor (0,0,2);
    space->collide(NULL,&nearCallback);
    if (!pause)
        //world->quickStep(REAL(0.02));
        world->step(REAL(0.02));

    if (write_world) {
        FILE *f = fopen ("state.dif","wt");
        if (f) {
            dWorldExportDIF (*world,f,"X");
            (void)fclose (f);
        }
        write_world = false;
    }

    // remove all contact joints
    dJointGroupEmpty (*contactgroup);

    dsSetTexture (DS_WOOD);

    dsSetColor (1,0.5f,0);
    dsDrawCylinder(top1->getPosition(),
                    top1->getRotation(),
                    (float)toplength, (float)topradius);
    dsDrawCapsule(top1->getPosition(),
                    top1->getRotation(),
                    (float)pinlength, (float)pinradius);

    dsSetColor (0.5f,1,0);
    dsDrawCylinder(top2->getPosition(),
                    top2->getRotation(),
                    (float)toplength, (float)topradius);
    dsDrawCapsule(top2->getPosition(),
                    top2->getRotation(),
                    (float)pinlength, (float)pinradius);

}


static void reset()
{
    dMatrix3 R;
    dRSetIdentity(R);

    top1->setRotation(R);
    top2->setRotation(R);

    top1->setPosition(REAL(0.8), -2, 2);
    top2->setPosition(REAL(0.8), 2, 2);
    
    top1->setAngularVel(1,0,7);
    top2->setAngularVel(1,0,7);
    
    top1->setLinearVel(0,REAL(0.2),0);
    top2->setLinearVel(0,REAL(0.2),0);
}

static void tilt()
{
    top1->addTorque(0, 10, 0);
    top2->addTorque(0, 10, 0);
}

int main (int argc, char **argv)
{
    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.command = &command;
    fn.stop = NULL;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;


    // create world
    dInitODE();
    world = new dWorld();
    world->setGravity(0,0,REAL(-0.5));
    world->setCFM(REAL(1e-5));
    world->setLinearDamping(REAL(0.00001));
    world->setAngularDamping(REAL(0.0001));
    
    space = new dSimpleSpace(NULL);

    dPlane *floor = new dPlane(*space, 0,0,1,0);

    top1 = new dBody(*world);
    top2 = new dBody(*world);

    dMass m;
    m.setCylinderTotal(1, 3, topradius, toplength);
    top1->setMass(m);
    top2->setMass(m);
    
    dGeom *g1, *g2, *pin1, *pin2;
    g1 = new dCylinder(*space, topradius, toplength);
    g1->setBody(*top1);
    g2 = new dCylinder(*space, topradius, toplength);
    g2->setBody(*top2);
    
    pin1 = new dCapsule(*space, pinradius, pinlength);
    pin1->setBody(*top1);
    pin2 = new dCapsule(*space, pinradius, pinlength);
    pin2->setBody(*top2);
    
    top2->setGyroscopicMode(false);

    contactgroup = new dJointGroup();

    reset();

    // run simulation
    dsSimulationLoop (argc, argv, DS_SIMULATION_DEFAULT_WIDTH, DS_SIMULATION_DEFAULT_HEIGHT, &fn);

    delete g1; g1 = NULL;
    delete g2; g2 = NULL;
    delete pin1; pin1 = NULL;
    delete pin2; pin2 = NULL;
    delete floor; floor = NULL;
    contactgroup->empty();
    delete contactgroup; contactgroup = NULL;
    delete top1; top1 = NULL;
    delete top2; top2 = NULL;
    delete space; space = NULL;
    delete world; world = NULL;
    dCloseODE();
}
