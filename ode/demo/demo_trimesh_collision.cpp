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

 // TriMesh collision demo.
 // Serves as a test for the collision of trimesh geometries.
 // By Davy (Dawei) Chen.

#include <cassert>
#ifdef HAVE_UNISTD_H
#	include <unistd.h>
#endif
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include "texturepath.h"

#ifdef dDOUBLE
#define dsDrawTriangle dsDrawTriangleD
#endif
#define dGeomTriMeshDataBuildReal dGeomTriMeshDataBuildSingle


static dWorldID world;
static dSpaceID space;
static dJointGroupID contactgroup;

// 2D Convex hulls to create the TriMesh geometries
const dReal HULL1[] = { -1.000000, -35.000000, 1.000000, -34.000000, 5.000000, -31.000000, 16.000000, -20.000000, 16.000000, -11.000000, 5.000000, 0.000000, -1.000000, 3.000000, -7.000000, 4.000000, -13.000000, 3.000000, -16.000000, 2.000000, -20.000000, 0.000000, -24.000000, -4.000000, -26.000000, -8.000000, -27.000000, -12.000000, -27.000000, -15.000000, -27.000000, -19.000000, -26.000000, -23.000000, -24.000000, -27.000000, -20.000000, -31.000000, -16.000000, -34.000000 };
const dReal HULL2[] = { 23.000000, 28.000000, 28.000000, 29.000000, 30.000000, 30.000000, 34.000000, 33.000000, 35.000000, 34.000000, 38.000000, 39.000000, 39.000000, 42.000000, 40.000000, 48.000000, 39.000000, 53.000000, 38.000000, 56.000000, 35.000000, 61.000000, 34.000000, 62.000000, 30.000000, 65.000000, 28.000000, 66.000000, 25.000000, 67.000000, 20.000000, 68.000000, 19.000000, 68.000000, 14.000000, 67.000000, 11.000000, 66.000000, 9.000000, 65.000000, 5.000000, 62.000000, 4.000000, 61.000000, 1.000000, 56.000000, 0.000000, 53.000000, 0.000000, 48.000000, 0.000000, 42.000000, 1.000000, 39.000000, 4.000000, 34.000000, 5.000000, 33.000000, 9.000000, 30.000000, 11.000000, 29.000000 };
const dReal HULL3[] = { -1.000000, -35.000000, 1.000000, -34.000000, 5.000000, -31.000000, 8.000000, -28.000000, 11.000000, -22.000000, 12.000000, -19.000000, 12.000000, -12.000000, 11.000000, -9.000000, 8.000000, -3.000000, 5.000000, 0.000000, -1.000000, 3.000000, -7.000000, 4.000000, -13.000000, 3.000000, -16.000000, 2.000000, -20.000000, 0.000000, -23.000000, -3.000000, -25.000000, -6.000000, -26.000000, -8.000000, -27.000000, -11.000000, -27.000000, -15.000000, -27.000000, -20.000000, -26.000000, -23.000000, -25.000000, -25.000000, -23.000000, -28.000000, -20.000000, -31.000000, -16.000000, -34.000000 };
const dReal HULL4[] = { -2.000000, -35.000000, 6.000000, -31.000000, 9.000000, -27.000000, 10.000000, -25.000000, 11.000000, -22.000000, 11.000000, -9.000000, 10.000000, -6.000000, 9.000000, -4.000000, 6.000000, 0.000000, -2.000000, 3.000000, -7.000000, 4.000000, -19.000000, 3.000000, -20.000000, 2.000000, -25.000000, -6.000000, -26.000000, -8.000000, -27.000000, -11.000000, -27.000000, -15.000000, -27.000000, -20.000000, -26.000000, -23.000000, -25.000000, -25.000000, -20.000000, -34.000000 };
const dReal HULL5[] = { -2.000000, -35.000000, 4.000000, -32.000000, 9.000000, -27.000000, 11.000000, -23.000000, 12.000000, -20.000000, 12.000000, -11.000000, 11.000000, -8.000000, 9.000000, -4.000000, 5.000000, 0.000000, -2.000000, 3.000000, -7.000000, 4.000000, -12.000000, 3.000000, -15.000000, 2.000000, -20.000000, 0.000000, -23.000000, -3.000000, -25.000000, -6.000000, -26.000000, -8.000000, -27.000000, -11.000000, -27.000000, -15.000000, -27.000000, -20.000000, -26.000000, -23.000000, -25.000000, -25.000000, -23.000000, -28.000000, -19.000000, -32.000000, -15.000000, -34.000000 };
const dReal HULL6[] = { -1.000000, -35.000000, 25.000000, -29.000000, 26.000000, -28.000000, 30.000000, -18.000000, 30.000000, -14.000000, 26.000000, -6.000000, 25.000000, -5.000000, -1.000000, 3.000000, -7.000000, 4.000000, -13.000000, 3.000000, -20.000000, 0.000000, -23.000000, -3.000000, -25.000000, -6.000000, -26.000000, -8.000000, -27.000000, -12.000000, -27.000000, -15.000000, -27.000000, -19.000000, -26.000000, -23.000000, -25.000000, -25.000000, -23.000000, -28.000000, -20.000000, -31.000000, -17.000000, -33.000000 };

// Center points of the 2D Convex hulls
const dReal CENTER1[] = { -5.500000, -15.500000 };
const dReal CENTER2[] = { 20.000000, 48.000000 };
const dReal CENTER3[] = { -7.500000, -15.500000 };
const dReal CENTER4[] = { -8.000000, -15.500000 };
const dReal CENTER5[] = { -7.500000, -15.500000 };
const dReal CENTER6[] = { 1.500000, -15.500000 };

// Where to position the TriMeshes on the Ground plane
// In (X, Y) coordinates pairs
const dReal BODY_POSITIONS[][2] = {
    {-60.0, -30.0},
    {  0.0, -30.0},
    { 60.0, -30.0},
    {-60.0,  30.0},
    {  0.0,  30.0},
    { 60.0,  30.0}
};

static const dReal *HULLS[] = { HULL1, HULL2, HULL3, HULL4, HULL5, HULL6 };
const unsigned int HULL_SIZES[] = {
    sizeof(HULL1) / sizeof(dReal) / 2,
    sizeof(HULL2) / sizeof(dReal) / 2,
    sizeof(HULL3) / sizeof(dReal) / 2,
    sizeof(HULL4) / sizeof(dReal) / 2,
    sizeof(HULL5) / sizeof(dReal) / 2,
    sizeof(HULL6) / sizeof(dReal) / 2
};
static const dReal *CENTERS[] = { CENTER1, CENTER2, CENTER3, CENTER4, CENTER5, CENTER6 };
const unsigned int HULLS_COUNT = sizeof(HULLS) / sizeof(dReal *);

const dReal TRIMESH_HEIGHT = REAL(2.0);

static dReal *odeVerts[HULLS_COUNT];
static dTriIndex *odeInds[HULLS_COUNT];
static unsigned int odeIndsCount[HULLS_COUNT];

static dTriMeshDataID triMeshDataId[HULLS_COUNT];
static dGeomID triMeshId[HULLS_COUNT];
static dBodyID bodyId[HULLS_COUNT];

// this is called by dSpaceCollide when two objects in space are
// potentially colliding.

static void nearCallback(void *data, dGeomID o1, dGeomID o2)
{
    assert(o1);
    assert(o2);
    if (dGeomIsSpace(o1) || dGeomIsSpace(o2))
    {
        // colliding a space with something
        dSpaceCollide2(o1, o2, data, &nearCallback);
        // Note we do not want to test intersections within a space,
        // only between spaces.
        return;
    }

    const int N = 32;
    dContact contact[N];
    const int n = dCollide(o1, o2, N, &(contact[0].geom), sizeof(dContact));
    if (n > 0)
    {
        for (int i = 0; i<n; i++)
        {
            contact[i].surface.slip1 = REAL(0.7);
            contact[i].surface.slip2 = REAL(0.7);
            contact[i].surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1 | dContactSlip1 | dContactSlip2;
            // Friction effect, if set to dInfinity, objects would be unmovable 
            contact[i].surface.mu = REAL(0.0);
            contact[i].surface.soft_erp = REAL(0.50);
            contact[i].surface.soft_cfm = REAL(0.03);
            dJointID c = dJointCreateContact(world, contactgroup, &contact[i]);
            dJointAttach
            (
                c,
                dGeomGetBody(contact[i].geom.g1),
                dGeomGetBody(contact[i].geom.g2)
            );
        }
    }
}

// start simulation - set viewpoint

static void start()
{
    dAllocateODEDataForThread((unsigned int)dAllocateMaskAll);
    float xyz[3] = { -8,0,5 };
    float hpr[3] = { 0.0f,-29.5000f,0.0000f };
    dsSetViewpoint(xyz, hpr);
    (void)fprintf(stderr, "Press SPACE to reset the simulation1.\n");
}

static void reset()
{
    for (unsigned int i = 0; i < HULLS_COUNT; i++)
    {
        dBodySetPosition(bodyId[i],
            dReal(BODY_POSITIONS[i][0]) * REAL(0.05),
            dReal(BODY_POSITIONS[i][1]) * REAL(0.05),
            REAL(0.0));

        dMatrix3 R;
        dRFromAxisAndAngle(R,
            REAL(1.0),
            REAL(1.0),
            REAL(1.0),
            REAL(0.0));
        dBodySetRotation(bodyId[i], R);

        // Enable the body as it might have been auto-disabled
        dBodyEnable(bodyId[i]);
    }
}

// called when a key pressed

static void command(int cmd)
{
    switch (cmd)
    {
        case ' ':
            reset();
            break;
        default:
            break;
    }
}

static void simLoop(int pause)
{
    const double simstep = 1/240.0;
    const double dt = dsElapsedTime();

    int nrofsteps = (int)ceilf((float)(dt/simstep));
    nrofsteps = nrofsteps > 8 ? 8 : nrofsteps;

    for (int i = 0; i<nrofsteps && !pause; i++)
    {
        // Add force to TriMesh bodies, and make them gather towards world center
        for (unsigned int j = 0; j < HULLS_COUNT; j++)
        {
            const dReal *pos = dBodyGetPosition(bodyId[j]);
            // Calculate force tensity according to distance from world center
            const dReal length = dCalcVectorLengthSquare3(pos);
            dVector3 pos1;
            dCopyVector3(pos1, pos);
            dNegateVector3(pos1);
            dNormalize3(pos1);
            dScaleVector3(pos1, REAL(5.0) * length);

            dBodyAddForce(bodyId[j], pos1[0], pos1[1], pos1[2]);
        }

        dSpaceCollide(space, NULL, &nearCallback);
        dWorldQuickStep(world, (dReal)simstep);
        dJointGroupEmpty(contactgroup);
    }

    dsSetColor(1, 1, 1);
    for (unsigned int i = 0; i < HULLS_COUNT; i++)
    {
        const dReal *Pos = dBodyGetPosition(bodyId[i]);
        const dReal *Rot = dBodyGetRotation(bodyId[i]);

        // Draw TriMeshes
        if (odeVerts[i])
        {
            for (unsigned int j = 0; j < odeIndsCount[i] / 3; j++)
            {
                const dVector3 v0 = {
                    dReal(odeVerts[i][odeInds[i][j * 3 + 0] * 3]),
                    dReal(odeVerts[i][odeInds[i][j * 3 + 0] * 3 + 1]),
                    dReal(odeVerts[i][odeInds[i][j * 3 + 0] * 3 + 2])
                };
                const dVector3 v1 = {
                    dReal(odeVerts[i][odeInds[i][j * 3 + 1] * 3]),
                    dReal(odeVerts[i][odeInds[i][j * 3 + 1] * 3 + 1]),
                    dReal(odeVerts[i][odeInds[i][j * 3 + 1] * 3 + 2])
                };
                const dVector3 v2 = {
                    dReal(odeVerts[i][odeInds[i][j * 3 + 2] * 3]),
                    dReal(odeVerts[i][odeInds[i][j * 3 + 2] * 3 + 1]),
                    dReal(odeVerts[i][odeInds[i][j * 3 + 2] * 3 + 2])
                };
                dsDrawTriangle(Pos, Rot, v0, v1, v2, 1);
            }
        }
    }
}

int main(int argc, char **argv)
{
    const dMass m;

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
    space = dHashSpaceCreate(NULL);
    dHashSpaceSetLevels(space, -3, 5);
    dCreatePlane(space, 0, 0, 1, 0);	// Add a ground plane.

    contactgroup = dJointGroupCreate(0);
    dWorldSetGravity(world, 0, 0, -1);
    dWorldSetQuickStepNumIterations(world, 32);
    dWorldSetContactMaxCorrectingVel(world, 40);
    dWorldSetMaxAngularSpeed(world, REAL(62.8));
    dWorldSetERP(world, REAL(0.7));
    dWorldSetQuickStepW(world, REAL(0.75)); // For increased stability.

    dWorldSetAutoDisableFlag(world, true);
    dWorldSetAutoDisableLinearThreshold(world, REAL(0.01));
    dWorldSetAutoDisableAngularThreshold(world, REAL(0.03));
    dWorldSetAutoDisableTime(world, REAL(0.15));

    // Generate TriMesh geometries and bodies
    for (unsigned int i = 0; i < HULLS_COUNT; i++)
    {
        const unsigned int hullSize = HULL_SIZES[i];
        // Vertices
        const unsigned int odeVertsCount1 = hullSize * 3 * 2;
        const unsigned int odeVertsCount = odeVertsCount1 + 2 * 3;
        odeVerts[i] = new dReal[odeVertsCount];
        for (unsigned int j = 0; j < hullSize; j++)
        {
            // Bottom layer
            odeVerts[i][j * 3] = (HULLS[i][j * 2] - CENTERS[i][0]) * REAL(0.05);
            odeVerts[i][j * 3 + 1] = (HULLS[i][j * 2 + 1] - CENTERS[i][1]) * REAL(0.05);
            odeVerts[i][j * 3 + 2] = REAL(0.0);
            // Top layer
            odeVerts[i][(hullSize + j) * 3] = (HULLS[i][j * 2] - CENTERS[i][0]) * REAL(0.05);
            odeVerts[i][(hullSize + j) * 3 + 1] = (HULLS[i][j * 2 + 1] - CENTERS[i][1]) * REAL(0.05);
            odeVerts[i][(hullSize + j) * 3 + 2] = TRIMESH_HEIGHT;
        }
        // Center vertex on bottom plane
        odeVerts[i][odeVertsCount1] = REAL(0.0);
        odeVerts[i][odeVertsCount1 + 1] = REAL(0.0);
        odeVerts[i][odeVertsCount1 + 2] = REAL(0.0);
        // Center vertex on top plane
        odeVerts[i][odeVertsCount1 + 3] = REAL(0.0);
        odeVerts[i][odeVertsCount1 + 3 + 1] = REAL(0.0);
        odeVerts[i][odeVertsCount1 + 3 + 2] = TRIMESH_HEIGHT;
        // Indices
        const unsigned int odeIndsCount1 = hullSize * 6;
        odeIndsCount[i] = odeIndsCount1 * 2;
        odeInds[i] = new dTriIndex[odeIndsCount[i]];
        for (unsigned int j = 0; j < hullSize; j++)
        {
            // Wall triangles
            // Wrap around index
            const unsigned int n1 = j + 1 < hullSize ? j + 1 : 0;
            const unsigned int n2 = hullSize + n1;
            odeInds[i][j * 6] = (dTriIndex)j;
            odeInds[i][j * 6 + 1] = (dTriIndex)n1;
            odeInds[i][j * 6 + 2] = (dTriIndex)(hullSize + j);
            odeInds[i][j * 6 + 3] = (dTriIndex)(hullSize + j);
            odeInds[i][j * 6 + 4] = (dTriIndex)n1;
            odeInds[i][j * 6 + 5] = (dTriIndex)n2;
            // Bottom and Top triangles
            odeInds[i][odeIndsCount1 + j * 6] = (dTriIndex)j;
            odeInds[i][odeIndsCount1 + j * 6 + 1] = (dTriIndex)n1;
            odeInds[i][odeIndsCount1 + j * 6 + 2] = (dTriIndex)(hullSize * 2);
            odeInds[i][odeIndsCount1 + j * 6 + 3] = (dTriIndex)(hullSize + j);
            odeInds[i][odeIndsCount1 + j * 6 + 4] = (dTriIndex)n2;
            odeInds[i][odeIndsCount1 + j * 6 + 5] = (dTriIndex)(hullSize * 2 + 1);
        }

        bodyId[i] = dBodyCreate(world);
        dBodySetPosition(bodyId[i],
            BODY_POSITIONS[i][0] * REAL(0.05),
            BODY_POSITIONS[i][1] * REAL(0.05),
            REAL(0.0));

        dMatrix3 R;
        dRFromAxisAndAngle(R,
            REAL(1.0),
            REAL(1.0),
            REAL(1.0),
            REAL(0.0));
        dBodySetRotation(bodyId[i], R);

        const size_t index = 0;
        dBodySetData(bodyId[i], (void *)index);

        dMass m1;
        dReal sides[3];
        sides[0] = REAL(3.0);
        sides[1] = REAL(3.0);
        sides[2] = REAL(3.0);
        const dReal DENSITY = REAL(1.0);
        dMassSetBox(&m1, DENSITY, sides[0], sides[1], sides[2]);

        dBodySetMass(bodyId[i], &m1);
        // Make bodies less bouncy
        dBodySetLinearDamping(bodyId[i], REAL(0.1));
        dBodySetAngularDamping(bodyId[i], REAL(0.1));

        triMeshDataId[i] = dGeomTriMeshDataCreate();
        dGeomTriMeshDataBuildReal(triMeshDataId[i], odeVerts[i], 3 * sizeof(float), (int)odeVertsCount, odeInds[i], (int)odeIndsCount[i], 3 * sizeof(dTriIndex));
        dGeomTriMeshDataPreprocess2(triMeshDataId[i], (1U << dTRIDATAPREPROCESS_BUILD_FACE_ANGLES), NULL);

        triMeshId[i] = dCreateTriMesh(space, triMeshDataId[i], NULL, NULL, NULL);
        dGeomSetBody(triMeshId[i], bodyId[i]);
    }

    // run simulation
    dsSimulationLoop(argc, argv, DS_SIMULATION_DEFAULT_WIDTH, DS_SIMULATION_DEFAULT_HEIGHT, &fn);

    for (unsigned int i = 0; i < HULLS_COUNT; i++)
    {
        dGeomTriMeshDataDestroy(triMeshDataId[i]);
        dGeomDestroy(triMeshId[i]);
        dBodyDestroy(bodyId[i]);

        delete[] odeVerts[i];
        odeVerts[i] = NULL;
        delete[] odeInds[i];
        odeInds[i] = NULL;
    }

    dJointGroupEmpty(contactgroup);
    dJointGroupDestroy(contactgroup);
    dSpaceDestroy(space);
    dWorldDestroy(world);
    dCloseODE();

    return 0;
}
