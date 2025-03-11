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
 * Created by:    Remi Ricard                                            *
 *                (remi.ricard@simlog.com or papaDoc@videotron.ca)       *
 * Creation date: 2007/05/04                                             *
 *************************************************************************/

/*
  This program demonstrates how the Piston joint works.

  A Piston joint enables the sliding of a body with respect to another body
  and the 2 bodies are free to rotate about the sliding axis.

  - The yellow body is fixed to the world.
  - The yellow body and the blue body are attached by a Piston joint with
    the axis along the x direction.
  - The purple object is a geometry obstacle.
  - The red line is the representation of the prismatic axis
  - The orange line is the representation of the rotoide axis
  - The light blue ball is the anchor position

  N.B. Many command options are available type -h to print them.
*/

#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <iostream>
#include <cmath>
#include "texturepath.h"

// select correct drawing functions
#ifdef dDOUBLE
#define dsDrawBox dsDrawBoxD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule dsDrawCapsuleD
#define dsDrawSphere dsDrawSphereD
#endif


const dReal VEL_INC = REAL(0.01); // Velocity increment

// physics parameters
const dReal PI = REAL(3.14159265358979323846264338327950288419716939937510);
const dReal BODY1_LENGTH = REAL(1.5);    // Size along the X axis

const dReal RADIUS = REAL(0.2);
const dReal AXIS_RADIUS = REAL(0.01);


#define X 0
#define Y 1
#define Z 2

enum INDEX
{
    BODY1 = 0,
    BODY2,
    RECT,
    BOX,
    OBS,
    GROUND,
    NUM_PARTS,
    ALL = NUM_PARTS
};

const int catBits[NUM_PARTS+1] =
{
    0x0001, ///< Ext Cylinder category
    0x0002, ///< Int Cylinder category
    0x0004, ///< Int_Rect Cylinder category
    0x0008, ///< Box category
    0x0010, ///< Obstacle category
    0x0020, ///< Ground category
    ~0      ///< All categories
};

#define Mass1 10
// #define Mass2 8


//camera view
static const float xyz[3] = {2.0f,-5.5f,2.0000f};
static const float hpr[3] = {90.000f,-15.5000f,0.0000f};


//world,space,body & geom
static dWorldID         world;
static dSpaceID         space;
static dJointGroupID    contactgroup;
static dBodyID          body[NUM_PARTS];
static dGeomID          geom[NUM_PARTS];

// Default Positions and anchor of the 2 bodies
static dVector3 pos1;
static dVector3 pos2;
static dVector3 anchor;

static dJoint *joint;


const dReal BODY2_SIDES[3] = {REAL(0.4), REAL(0.4), REAL(0.4)};
const dReal OBS_SIDES[3]  = {1,1,1};
const dReal RECT_SIDES[3] = {REAL(0.3), REAL(0.1), REAL(0.2)};


static int type = dJointTypePiston;

//#pragma message("tc to be changed to 0")

static int tc = 0; // The test case choice;


//collision detection
static void nearCallback (void * /*data*/, dGeomID o1, dGeomID o2)
{
    int n;

    dBodyID b1 = dGeomGetBody (o1);
    dBodyID b2 = dGeomGetBody (o2);
    if (b1 && b2 && dAreConnectedExcluding (b1,b2,dJointTypeContact) ) return;
    const int N = 10;
    dContact contact[N];
    n = dCollide (o1,o2,N,&contact[0].geom,sizeof (dContact) );
    if (n > 0)
    {
        for  (int i=0; i<n; i++)
        {
            contact[i].surface.mode = (dContactSlip1 | dContactSlip2 |
                                       dContactSoftERP | dContactSoftCFM |
                                       dContactApprox1);
            contact[i].surface.mu = REAL(0.1);
            contact[i].surface.slip1 = REAL(0.02);
            contact[i].surface.slip2 = REAL(0.02);
            contact[i].surface.soft_erp = REAL(0.1);
            contact[i].surface.soft_cfm = REAL(0.0001);
            dJointID c = dJointCreateContact (world,contactgroup,&contact[i]);
            dJointAttach (c,dGeomGetBody (contact[i].geom.g1),dGeomGetBody (contact[i].geom.g2) );
        }
    }
}

static void printKeyBoardShortCut()
{
    printf ("Press 'h' for this help.\n");
    printf ("Press 'q' to add force on BLUE body along positive x direction.\n");
    printf ("Press 'w' to add force on BLUE body along negative x direction.\n");

    printf ("Press 'a' to add force on BLUE body along positive y direction.\n");
    printf ("Press 's' to add force on BLUE body along negative y direction.\n");

    printf ("Press 'z' to add force on BLUE body along positive z direction.\n");
    printf ("Press 'x' to add force on BLUE body along negative z direction.\n");

    printf ("Press 'e' to add torque on BLUE body around positive x direction \n");
    printf ("Press 'r' to add torque on BLUE body around negative x direction \n");

    printf ("Press 'd' to add torque on BLUE body around positive y direction \n");
    printf ("Press 'f' to add torque on BLUE body around negative y direction \n");

    printf ("Press 'c' to add torque on BLUE body around positive z direction \n");
    printf ("Press 'v' to add torque on BLUE body around negative z direction \n");

    printf ("Press 't' to add force on prismatic joint in the positive axis direction\n");
    printf ("Press 'y' to add force on prismatic joint in the negative axis direction\n");

    printf ("Press 'i' to add limits on the prismatic joint (0 to 0) \n");
    printf ("Press 'o' to add limits on the rotoide joint (0 to 0)\n");
    printf ("Press 'k' to add limits on the rotoide joint (-45 to 45deg) \n");
    printf ("Press 'l' to remove limits on the rotoide joint \n");


    printf ("Press '.' to increase joint velocity along the prismatic direction.\n");
    printf ("Press ',' to decrease joint velocity along the prismatic direction.\n");

    printf ("Press 'p' to print the Position of the joint.\n");

    printf ("Press '+' Go to the next test case.\n");
    printf ("Press '-' Go to the previous test case.\n");

    printf ("Press '8' To remove one of the body. The blue body and the world will be\n");
    printf ("          attached to the joint (blue body at position 1)\n");
    printf ("Press '9' To remove one of the body. The blue body and the world will be\n");
    printf ("          attached to the joint (body body at position 2)\n");


}


// start simulation - set viewpoint
static void start()
{
    dAllocateODEDataForThread((unsigned int)dAllocateMaskAll);

    dsSetViewpoint (xyz,hpr);
    printf ("This program demonstrates how the Piston joint works.\n");
    printf ("A Piston joint enables the sliding of a body with respect to another body\n");
    printf ("and the 2 bodies are free to rotate about the sliding axis.\n\n");
    printf ("The yellow body is fixed to the world\n");
    printf ("The yellow body and the blue body are attached by a Piston joint with\n");
    printf ("the axis along the x direction.\n");
    printf ("The purple object is a geometry obstacle.\n");

    printKeyBoardShortCut();
}


static void setPositionBodies (int val)
{
    const dVector3 POS1 = {0,0,REAL(1.5),0};
    const dVector3 POS2 = {0,0,REAL(1.5),0};
    const dVector3 ANCHOR = {0,0,REAL(1.5),0};

    for (int i=0; i<3; ++i)
    {
        pos1[i] = POS1[i];
        pos2[i] = POS2[i];
        anchor[i] = ANCHOR[i];
    }

    if (body[BODY1])
    {
        dBodySetLinearVel (body[BODY1], 0,0,0);
        dBodySetAngularVel (body[BODY1], 0,0,0);
    }

    if (body[BODY2])
    {
        dBodySetLinearVel (body[BODY2], 0,0,0);
        dBodySetAngularVel (body[BODY2], 0,0,0);
    }

    switch (val)
    {
    case 3:
        pos1[Z] += REAL(-0.5);
        anchor[Z] -= REAL(0.25);
        break;
    case 2:
        pos1[Z] -= REAL(0.5);
        anchor[Z] -= REAL(0.5);
        break;
    case 1:
        pos1[Z] += REAL(-0.5);
        break;
    default: // This is also case 0
        // Nothing to be done
        break;
    }

    const dMatrix3 R =
    {
        1,0,0,0,
        0,1,0,0,
        0,0,1,0
    };

    if (body[BODY1])
    {
        dBodySetPosition (body[BODY1], pos1[X], pos1[Y], pos1[Z]);
        dBodySetRotation (body[BODY1], R);
    }

    if (body[BODY2])
    {
        dBodySetPosition (body[BODY2], pos2[X], pos2[Y], pos2[Z]);
        dBodySetRotation (body[BODY2], R);
    }



    if (joint)
    {
        joint->attach (body[BODY1], body[BODY2]);
        if (joint->getType() == dJointTypePiston)
            dJointSetPistonAnchor(joint->id(), anchor[X], anchor[Y], anchor[Z]);
    }

}


// function to update camera position at each step.
static void update()
{
//   static FILE *file = fopen("x:/sim/src/libode/tstsrcSF/export.dat", "w");

//   static int cnt = 0;
//   char str[24];
//   sprintf(str, "%06d",cnt++);

//   dWorldExportDIF(world, file, str);
}


// called when a key pressed
static void command (int cmd)
{
    switch (cmd)
    {
    case 'h' :
    case 'H' :
    case '?' :
        printKeyBoardShortCut();
        break;

        // Force
    case 'q' :
    case 'Q' :
        dBodyAddForce (body[BODY1],4,0,0);
        break;
    case 'w' :
    case 'W' :
        dBodyAddForce (body[BODY1],-4,0,0);
        break;

    case 'a' :
    case 'A' :
        dBodyAddForce (body[BODY1],0,40,0);
        break;
    case 's' :
    case 'S' :
        dBodyAddForce (body[BODY1],0,-40,0);
        break;

    case 'z' :
    case 'Z' :
        dBodyAddForce (body[BODY1],0,0,4);
        break;
    case 'x' :
    case 'X' :
        dBodyAddForce (body[BODY1],0,0,-4);
        break;

        // Torque
    case 'e':
    case 'E':
        dBodyAddTorque (body[BODY1],REAL( 0.1),0,0);
        break;
    case 'r':
    case 'R':
        dBodyAddTorque (body[BODY1],REAL(-0.1),0,0);
        break;

    case 'd':
    case 'D':
        dBodyAddTorque (body[BODY1],0,REAL( 0.1),0);
        break;
    case 'f':
    case 'F':
        dBodyAddTorque (body[BODY1],0,REAL(-0.1),0);
        break;

    case 'c':
    case 'C':
        dBodyAddTorque (body[BODY1],REAL( 0.1),0,0);
        break;
    case 'v':
    case 'V':
        dBodyAddTorque (body[BODY1],REAL(-0.1),0,0);
        break;

    case 't':
    case 'T':
        if (joint->getType() == dJointTypePiston)
            dJointAddPistonForce (joint->id(),1);
        else
            dJointAddSliderForce (joint->id(),1);
        break;
    case 'y':
    case 'Y':
        if (joint->getType() == dJointTypePiston)
            dJointAddPistonForce (joint->id(),-1);
        else
            dJointAddSliderForce (joint->id(),-1);
        break;


    case '8' :
        dJointAttach(joint->id(), body[0], NULL);
        break;
    case '9' :
        dJointAttach(joint->id(), NULL, body[0]);
        break;

    case 'i':
    case 'I' :
        joint->setParam (dParamLoStop, 0);
        joint->setParam (dParamHiStop, 0);
        break;

    case 'o':
    case 'O' :
        joint->setParam (dParamLoStop2, 0);
        joint->setParam (dParamHiStop2, 0);
        break;

    case 'k':
    case 'K':
        joint->setParam (dParamLoStop2, dReal(-45.0*3.14159267/180.0));
        joint->setParam (dParamHiStop2, dReal( 45.0*3.14159267/180.0));
        break;
    case 'l':
    case 'L':
        joint->setParam (dParamLoStop2, -dInfinity);
        joint->setParam (dParamHiStop2,  dInfinity);
        break;

        // Velocity of joint
    case ',':
    case '<' :
    {
        const dReal vel = joint->getParam (dParamVel) - VEL_INC;
        joint->setParam (dParamVel, vel);
        std::cout<<"Velocity = "<<vel<<"  FMax = 2"<<'\n';
    }
    break;

    case '.':
    case '>' :
    {
        const dReal vel = joint->getParam (dParamVel) + VEL_INC;
        joint->setParam (dParamVel, vel);
        std::cout<<"Velocity = "<<vel<<"  FMax = 2"<<'\n';
    }
    break;

    case 'p' :
    case 'P' :
    {
        switch (joint->getType() )
        {
        case dJointTypeSlider :
        {
            const dSliderJoint *sj = static_cast<dSliderJoint *> (joint);
            std::cout<<"Position ="<<sj->getPosition() <<"\n";
        }
        break;
        case dJointTypePiston :
        {
            const dPistonJoint *rj = static_cast<dPistonJoint *> (joint);
            std::cout<<"Position ="<<rj->getPosition() <<"\n";
        }
        break;
        // keep the compiler happy
        case dJointTypeNone:
        case dJointTypeBall:
        case dJointTypeHinge:
        case dJointTypeContact:
        case dJointTypeUniversal:
        case dJointTypeHinge2:
        case dJointTypeFixed:
        case dJointTypeNull:
        case dJointTypeAMotor:
        case dJointTypeLMotor:
        case dJointTypePlane2D:
        case dJointTypePR:
        case dJointTypePU:
        case dJointTypeDBall:
        case dJointTypeDHinge:
        case dJointTypeTransmission:
        {}
        }
    }
    break;

    case '+' :
        (++tc) %= 4;
        setPositionBodies (tc);
        break;
    case '-' :
        (--tc) %= 4;
        setPositionBodies (tc);
        break;

    default:
        break;
    }
}

static void drawBox (dGeomID id, int R, int G, int B)
{
    if (!id)
        return;

    const dReal *pos = dGeomGetPosition (id);
    const dReal *rot = dGeomGetRotation (id);
    dsSetColor ((float)R,(float)G,(float)B);

    dVector3 l;
    dGeomBoxGetLengths (id, l);
    dsDrawBox (pos, rot, l);
}


// simulation loop
static void simLoop (int pause)
{
    const dReal *rot;
    dVector3 ax = {};
    dReal l=0;

    switch (joint->getType() )
    {
    case dJointTypeSlider :
        ( (dSliderJoint *) joint)->getAxis (ax);
        l = ( (dSliderJoint *) joint)->getPosition();
        break;
    case dJointTypePiston :
        ( (dPistonJoint *) joint)->getAxis (ax);
        l = ( (dPistonJoint *) joint)->getPosition();
        break;
    // keep the compiler happy
    case dJointTypeNone:
    case dJointTypeBall:
    case dJointTypeHinge:
    case dJointTypeContact:
    case dJointTypeUniversal:
    case dJointTypeHinge2:
    case dJointTypeFixed:
    case dJointTypeNull:
    case dJointTypeAMotor:
    case dJointTypeLMotor:
    case dJointTypePlane2D:
    case dJointTypePR:
    case dJointTypePU:
    case dJointTypeDBall:
    case dJointTypeDHinge:
    case dJointTypeTransmission:
        {}
    }


    if (!pause)
    {
        const double simstep = 0.01; // 1ms simulation steps
        const double dt = dsElapsedTime();

        int nrofsteps = (int) ceilf ((float)(dt/simstep));
        if (!nrofsteps)
            nrofsteps = 1;

        for (int i=0; i<nrofsteps && !pause; i++)
        {
            dSpaceCollide (space,NULL,&nearCallback);
            dWorldStep (world, (dReal)simstep);

            dJointGroupEmpty (contactgroup);
        }

        update();


        dReal radius, length;

        dsSetTexture (DS_WOOD);

        drawBox (geom[BODY2], 1,1,0);

        drawBox (geom[RECT], 0,0,1);

        if ( geom[BODY1] )
        {
            const dReal *pos = dGeomGetPosition (geom[BODY1]);
            rot = dGeomGetRotation (geom[BODY1]);
            dsSetColor (0,0,1);

            dGeomCapsuleGetParams (geom[BODY1], &radius, &length);
            dsDrawCapsule (pos, rot, (float)length, (float)radius);
        }


        drawBox (geom[OBS], 1,0,1);


        // Draw the prismatic axis
        if ( geom[BODY1] )
        {
            const dReal *pos = dGeomGetPosition (geom[BODY1]);
            rot = dGeomGetRotation (geom[BODY2]);
            dVector3 p;
            p[X] = pos[X] - l*ax[X];
            p[Y] = pos[Y] - l*ax[Y];
            p[Z] = pos[Z] - l*ax[Z];
            dsSetColor (1,0,0);
            dsDrawCylinder (p, rot, 3.75f, (float)(REAL(1.05)*AXIS_RADIUS));
        }


        if (joint->getType() == dJointTypePiston )
        {
            dVector3 pistonAnchor;
            dJointGetPistonAnchor(joint->id(), pistonAnchor);

            // Draw the rotoide axis
            rot = dGeomGetRotation (geom[BODY2]);
            dsSetColor (1,0.5,0);
            dsDrawCylinder (pistonAnchor, rot, 4, (float)AXIS_RADIUS);


            dsSetColor (0,1,1);
            rot = dGeomGetRotation (geom[BODY1]);
            dsDrawSphere (pistonAnchor, rot, (float)(REAL(1.5)*RADIUS));
        }

    }
}


static ODE_NORETURN_VOID Help (char **argv)
{
    printf ("%s ", argv[0]);
    printf (" -h | --help   : print this help\n");
    printf (" -s | --slider : Set the joint as a slider\n");
    printf (" -p | --piston : Set the joint as a Piston. (Default joint)\n");
    printf (" -1 | --offset1 : Create an offset between the 2 bodies\n");
    printf ("                  Offset one of the body by z=-0.5 and keep the anchor\n");
    printf ("                  point in the middle of the fixed body\n");
    printf (" -2 | --offset2 : Create an offset between the 2 bodies\n");
    printf ("                  Offset one of the body by z=-0.5 and set the anchor\n");
    printf ("                  point in the middle of the movable body\n");
    printf (" -3 | --offset3 : Create an offset between the 2 bodies\n");
    printf ("                  Offset one of the body by z=-0.5 and set the anchor\n");
    printf ("                  point in the middle of the 2 bodies\n");
    printf (" -t | --texture-path path  : Path to the texture.\n");
    printf ("                             Default = %s\n", DRAWSTUFF_TEXTURE_PATH);
    printf (" -n | --notFixed : In free space with no gravity mode");
    printf ("-notex          : Don't use texture\n");
    printf ("-noshadow       : No shadow\n");
    printf ("-noshadows      : No shadows\n");
    printf ("-pause          : Initial pause\n");
    printf ("--------------------------------------------------\n");
    printf ("Hit any key to continue:");
    getchar();

    exit (0);
}

int main (int argc, char **argv)
{
    dInitODE2(0);
    bool fixed  = true;

    // setup pointers to drawstuff callback functions
    dsFunctions fn;
    fn.version = DS_VERSION;
    fn.start = &start;
    fn.step = &simLoop;
    fn.command = &command;
    fn.stop = NULL;
    fn.path_to_textures = DRAWSTUFF_TEXTURE_PATH;

    dVector3 offset;
    dSetZero (offset, 4);

    // Default test case

    if (argc >= 2 )
    {
        for (int i=1; i < argc; ++i)
        {
            //static int tata = 0;

            if (1)
            {
                if ( 0 == strcmp ("-h", argv[i]) || 0 == strcmp ("--help", argv[i]) )
                    Help (argv);

                if ( 0 == strcmp ("-s", argv[i]) || 0 == strcmp ("--slider", argv[i]) )
                    type = dJointTypeSlider;

                if ( 0 == strcmp ("-t", argv[i]) || 0 == strcmp ("--texture-path", argv[i]) )
                {
                    const int j = i+1;
                    if ( j >= argc      ||  // Check if we have enough arguments
                            argv[j][0] == '\0' ||  // We should have a path here
                            argv[j][0] == '-' ) // We should have a path not a command line
                        Help (argv);
                    else
                        fn.path_to_textures = argv[++i]; // Increase i since we use this argument
                }
            }


            if ( 0 == strcmp ("-1", argv[i]) || 0 == strcmp ("--offset1", argv[i]) )
                tc = 1;

            if ( 0 == strcmp ("-2", argv[i]) || 0 == strcmp ("--offset2", argv[i]) )
                tc = 2;

            if ( 0 == strcmp ("-3", argv[i]) || 0 == strcmp ("--offset3", argv[i]) )
                tc = 3;

            if (0 == strcmp ("-n", argv[i]) || 0 == strcmp ("--notFixed", argv[i]) )
                fixed = false;
        }
    }

    world = dWorldCreate();
    dWorldSetERP (world, REAL(0.8));

    space = dSimpleSpaceCreate (NULL);
    contactgroup = dJointGroupCreate (0);
    geom[GROUND] = dCreatePlane (space, 0,0,1,0);
    dGeomSetCategoryBits (geom[GROUND], (unsigned long)catBits[GROUND]);
    dGeomSetCollideBits (geom[GROUND], (unsigned long)catBits[ALL]);

    dMass m;
    dMatrix3 R;


    // Create the Obstacle
    geom[OBS] = dCreateBox (space, OBS_SIDES[0], OBS_SIDES[1], OBS_SIDES[2]);
    dGeomSetCategoryBits (geom[OBS], (unsigned long)catBits[OBS]);
    dGeomSetCollideBits (geom[OBS], (unsigned long)catBits[ALL]);
    //Rotation of 45deg around y
    dRFromAxisAndAngle (R, 1,1,0, dReal(REAL(-0.25)*PI));
    dGeomSetRotation (geom[OBS], R);
    dGeomSetPosition (geom[OBS], REAL(1.95), REAL(-0.2), REAL(0.5));


    //Rotation of 90deg around y
    // Will orient the Z axis along X
    dRFromAxisAndAngle (R, 0,1,0, dReal(REAL(-0.5)*PI));


    // Create Body2 (Wiil be attached to the world)
    body[BODY2] = dBodyCreate (world);
    // Main axis of cylinder is along X=1
    dMassSetBox (&m, 1, BODY2_SIDES[0], BODY2_SIDES[1], BODY2_SIDES[2]);
    dMassAdjust (&m, Mass1);
    geom[BODY2] = dCreateBox (space, BODY2_SIDES[0], BODY2_SIDES[1], BODY2_SIDES[2]);
    dGeomSetBody (geom[BODY2], body[BODY2]);
    dGeomSetOffsetRotation (geom[BODY2], R);
    dGeomSetCategoryBits (geom[BODY2], (unsigned long)catBits[BODY2]);
    dGeomSetCollideBits (geom[BODY2], (unsigned long)(catBits[ALL] & (~catBits[BODY1])) );
    dBodySetMass (body[BODY2], &m);


    // Create Body 1 (Slider on the prismatic axis)
    body[BODY1] = dBodyCreate (world);
    // Main axis of capsule is along X=1
    dMassSetCapsule (&m, 1, 1, RADIUS, BODY1_LENGTH);
    dMassAdjust (&m, Mass1);
    geom[BODY1] = dCreateCapsule (space, RADIUS, BODY1_LENGTH);
    dGeomSetBody (geom[BODY1], body[BODY1]);
    dGeomSetOffsetRotation (geom[BODY1], R);
    dGeomSetCategoryBits (geom[BODY1], (unsigned long)catBits[BODY1]);
    dGeomSetCollideBits (geom[BODY1], (unsigned long)(catBits[ALL] & ~catBits[BODY2] & ~catBits[RECT]));

    dMass mRect;
    dMassSetBox (&mRect, 1, RECT_SIDES[0], RECT_SIDES[1], RECT_SIDES[2]);
    dMassAdd (&m, &mRect);
    // TODO: translate m?
    geom[RECT] = dCreateBox (space, RECT_SIDES[0], RECT_SIDES[1], RECT_SIDES[2]);
    dGeomSetBody (geom[RECT], body[BODY1]);
    dGeomSetOffsetPosition (geom[RECT],
                            (BODY1_LENGTH-RECT_SIDES[0]) /REAL(2.0),
                            0.0,
                            -RADIUS -RECT_SIDES[2]/REAL(2.0));
    dGeomSetCategoryBits (geom[RECT], (unsigned long)catBits[RECT]);
    dGeomSetCollideBits (geom[RECT], (unsigned long)(catBits[ALL] & (~catBits[BODY1])) );

    dBodySetMass (body[BODY1], &m);



    setPositionBodies (tc);


    if ( fixed )
    {
        // Attache external cylinder to the world
        dJointID fixedJoint = dJointCreateFixed (world,NULL);
        dJointAttach (fixedJoint , NULL, body[BODY2]);
        dJointSetFixed (fixedJoint );
        dWorldSetGravity (world,0,0,REAL(-0.8));
    }
    else
    {
        dWorldSetGravity (world,0,0,0);
    }




    // The static is here only to help debugging
    switch (type)
    {
    case dJointTypeSlider :
    {
        dSliderJoint *sj = new dSliderJoint (world, NULL);
        sj->attach (body[BODY1], body[BODY2]);
        sj->setAxis (1, 0, 0);
        joint = sj;
    }
    break;

    case dJointTypePiston : // fall through default
    default:
    {
        dPistonJoint *pj = new dPistonJoint (world, NULL);
        pj->attach (body[BODY1], body[BODY2]);
        pj->setAxis (1, 0, 0);

        dJointSetPistonAnchor(pj->id(), anchor[X], anchor[Y], anchor[Z]);

        joint = pj;
    }
    break;
    }


    // run simulation
    dsSimulationLoop (argc, argv, DS_SIMULATION_DEFAULT_WIDTH, DS_SIMULATION_DEFAULT_HEIGHT, &fn);

    delete joint; joint = NULL;
    dJointGroupDestroy (contactgroup);
    dSpaceDestroy (space);
    dWorldDestroy (world);
    dCloseODE();
    return 0;
}




