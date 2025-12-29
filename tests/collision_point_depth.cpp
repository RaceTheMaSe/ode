#include <UnitTest++.h>
#include <ode/ode.h>
#include "common_impl.h"


#undef PROXIMITY_TOLERANCE
#if defined(dSINGLE)
#define PROXIMITY_TOLERANCE     dReal(1e-7)
#elif defined(dDOUBLE)
#define PROXIMITY_TOLERANCE     dReal(1e-12)
#else
#error "No precision defined"
#endif

TEST(test_collision_sphere_point_depth)
{
    // Test case: sphere at the origin.
    {
        const dReal radius = 1;
        dGeomID sphere = dCreateSphere(0, radius);

        dGeomSetPosition(sphere, 0,0,0);

        // depth at center should equal radius
        CHECK_EQUAL(radius, dGeomSpherePointDepth(sphere, 0,0,0));

        // half-radius depth
        CHECK_EQUAL(dReal(0.5) * radius, dGeomSpherePointDepth(sphere, dReal(0.5),   0,   0));
        CHECK_EQUAL(dReal(0.5) * radius, dGeomSpherePointDepth(sphere,   0, dReal(0.5),   0));
        CHECK_EQUAL(dReal(0.5) * radius, dGeomSpherePointDepth(sphere,   0,   0, dReal(0.5)));
        CHECK_EQUAL(dReal(0.5) * radius, dGeomSpherePointDepth(sphere, dReal(-0.5),    0,    0));
        CHECK_EQUAL(dReal(0.5) * radius, dGeomSpherePointDepth(sphere,    0, dReal(-0.5),    0));
        CHECK_EQUAL(dReal(0.5) * radius, dGeomSpherePointDepth(sphere,    0,    0, dReal(-0.5)));
        CHECK_CLOSE(dReal(0.5) * radius, dGeomSpherePointDepth(sphere, dReal(0.3), dReal(0.4),   0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.5) * radius, dGeomSpherePointDepth(sphere,   0, dReal(0.3), dReal(0.4)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.5) * radius, dGeomSpherePointDepth(sphere, dReal(0.4),   0, dReal(0.3)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.5) * radius, dGeomSpherePointDepth(sphere, dReal(-0.3),  dReal(0.4),    0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.5) * radius, dGeomSpherePointDepth(sphere,    0, dReal(-0.3),  dReal(0.4)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.5) * radius, dGeomSpherePointDepth(sphere,  dReal(0.4),    0, dReal(-0.3)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.5) * radius, dGeomSpherePointDepth(sphere,  dReal(0.3), dReal(-0.4),    0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.5) * radius, dGeomSpherePointDepth(sphere,    0,  dReal(0.3), dReal(-0.4)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.5) * radius, dGeomSpherePointDepth(sphere, dReal(-0.4),    0,  dReal(0.3)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.5) * radius, dGeomSpherePointDepth(sphere, dReal(-0.3), dReal(-0.4),    0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.5) * radius, dGeomSpherePointDepth(sphere,    0, dReal(-0.3), dReal(-0.4)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.5) * radius, dGeomSpherePointDepth(sphere, dReal(-0.4),    0, dReal(-0.3)), PROXIMITY_TOLERANCE);

        // 0.1 radius depth
        CHECK_CLOSE(dReal(0.1) * radius, dGeomSpherePointDepth(sphere, dReal(0.9),   0,   0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.1) * radius, dGeomSpherePointDepth(sphere,   0, dReal(0.9),   0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.1) * radius, dGeomSpherePointDepth(sphere,   0,   0, dReal(0.9)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.1) * radius, dGeomSpherePointDepth(sphere, dReal(-0.9),    0,    0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.1) * radius, dGeomSpherePointDepth(sphere,    0, dReal(-0.9),    0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.1) * radius, dGeomSpherePointDepth(sphere,    0,    0, dReal(-0.9)), PROXIMITY_TOLERANCE);

        // on surface (zero depth)
        CHECK_EQUAL(dReal(0), dGeomSpherePointDepth(sphere, dReal(1.0),   0,   0));
        CHECK_EQUAL(dReal(0), dGeomSpherePointDepth(sphere,   0, dReal(1.0),   0));
        CHECK_EQUAL(dReal(0), dGeomSpherePointDepth(sphere,   0,   0, dReal(1.0)));
        CHECK_EQUAL(dReal(0), dGeomSpherePointDepth(sphere, dReal(-1.0),    0,    0));
        CHECK_EQUAL(dReal(0), dGeomSpherePointDepth(sphere,    0, dReal(-1.0),    0));
        CHECK_EQUAL(dReal(0), dGeomSpherePointDepth(sphere,    0,    0, dReal(-1.0)));
        CHECK_CLOSE(dReal(0), dGeomSpherePointDepth(sphere, dReal(0.6), dReal(0.8),   0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0), dGeomSpherePointDepth(sphere,   0, dReal(0.6), dReal(0.8)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0), dGeomSpherePointDepth(sphere, dReal(0.8),   0, dReal(0.6)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0), dGeomSpherePointDepth(sphere, dReal(-0.6),  dReal(0.8),    0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0), dGeomSpherePointDepth(sphere,    0, dReal(-0.6),  dReal(0.8)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0), dGeomSpherePointDepth(sphere,  dReal(0.8),    0, dReal(-0.6)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0), dGeomSpherePointDepth(sphere,  dReal(0.6), dReal(-0.8),    0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0), dGeomSpherePointDepth(sphere,    0,  dReal(0.6), dReal(-0.8)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0), dGeomSpherePointDepth(sphere, dReal(-0.8),    0,  dReal(0.6)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0), dGeomSpherePointDepth(sphere, dReal(-0.6), dReal(-0.8),    0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0), dGeomSpherePointDepth(sphere,    0, dReal(-0.6), dReal(-0.8)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0), dGeomSpherePointDepth(sphere, dReal(-0.8),    0, dReal(-0.6)), PROXIMITY_TOLERANCE);

        // 0.1 radius from surface (negative depth)
        CHECK_CLOSE(dReal(-0.1) * radius, dGeomSpherePointDepth(sphere, dReal(1.1),   0,   0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.1) * radius, dGeomSpherePointDepth(sphere,   0, dReal(1.1),   0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.1) * radius, dGeomSpherePointDepth(sphere,   0,   0, dReal(1.1)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.1) * radius, dGeomSpherePointDepth(sphere, dReal(-1.1),    0,    0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.1) * radius, dGeomSpherePointDepth(sphere,    0, dReal(-1.1),    0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.1) * radius, dGeomSpherePointDepth(sphere,    0,    0, dReal(-1.1)), PROXIMITY_TOLERANCE);

        // half-radius from surface (negative depth)
        CHECK_EQUAL(dReal(-0.5) * radius, dGeomSpherePointDepth(sphere, dReal(1.5),   0,   0));
        CHECK_EQUAL(dReal(-0.5) * radius, dGeomSpherePointDepth(sphere,   0, dReal(1.5),   0));
        CHECK_EQUAL(dReal(-0.5) * radius, dGeomSpherePointDepth(sphere,   0,   0, dReal(1.5)));
        CHECK_EQUAL(dReal(-0.5) * radius, dGeomSpherePointDepth(sphere, dReal(-1.5),    0,    0));
        CHECK_EQUAL(dReal(-0.5) * radius, dGeomSpherePointDepth(sphere,    0, dReal(-1.5),    0));
        CHECK_EQUAL(dReal(-0.5) * radius, dGeomSpherePointDepth(sphere,    0,    0, dReal(-1.5)));
        CHECK_CLOSE(dReal(-0.5), dGeomSpherePointDepth(sphere, dReal(0.9), dReal(1.2),   0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.5), dGeomSpherePointDepth(sphere,   0, dReal(0.9), dReal(1.2)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.5), dGeomSpherePointDepth(sphere, dReal(1.2),   0, dReal(0.9)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.5), dGeomSpherePointDepth(sphere, dReal(-0.9),  dReal(1.2),    0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.5), dGeomSpherePointDepth(sphere,    0, dReal(-0.9),  dReal(1.2)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.5), dGeomSpherePointDepth(sphere,  dReal(1.2),    0, dReal(-0.9)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.5), dGeomSpherePointDepth(sphere,  dReal(0.9), dReal(-1.2),    0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.5), dGeomSpherePointDepth(sphere,    0,  dReal(0.9), dReal(-1.2)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.5), dGeomSpherePointDepth(sphere, dReal(-1.2),    0,  dReal(0.9)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.5), dGeomSpherePointDepth(sphere, dReal(-0.9), dReal(-1.2),    0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.5), dGeomSpherePointDepth(sphere,    0, dReal(-0.9), dReal(-1.2)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.5), dGeomSpherePointDepth(sphere, dReal(-1.2),    0, dReal(-0.9)), PROXIMITY_TOLERANCE);

        dGeomDestroy(sphere);
    }
}

TEST(test_collision_box_point_depth)
{
    // Test case: cube at the origin.
    {
        const dReal length = 1;
        dGeomID cube = dCreateBox(0, 2*length, 2*length, 2*length);

        dGeomSetPosition(cube, 0,0,0);

        // depth at center should equal half length
        CHECK_EQUAL(length, dGeomBoxPointDepth(cube, 0,0,0));

        // half-length depth
        CHECK_EQUAL(dReal(0.5) * length, dGeomBoxPointDepth(cube, dReal(0.5),   0,   0));
        CHECK_EQUAL(dReal(0.5) * length, dGeomBoxPointDepth(cube,   0, dReal(0.5),   0));
        CHECK_EQUAL(dReal(0.5) * length, dGeomBoxPointDepth(cube,   0,   0, dReal(0.5)));
        CHECK_EQUAL(dReal(0.5) * length, dGeomBoxPointDepth(cube, dReal(-0.5),    0,    0));
        CHECK_EQUAL(dReal(0.5) * length, dGeomBoxPointDepth(cube,    0, dReal(-0.5),    0));
        CHECK_EQUAL(dReal(0.5) * length, dGeomBoxPointDepth(cube,    0,    0, dReal(-0.5)));

        // closest point 0.6 * length
        CHECK_CLOSE(dReal(0.6) * length, dGeomBoxPointDepth(cube, dReal(0.3), dReal(0.4),   0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.6) * length, dGeomBoxPointDepth(cube,   0, dReal(0.3), dReal(0.4)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.6) * length, dGeomBoxPointDepth(cube, dReal(0.4),   0, dReal(0.3)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.6) * length, dGeomBoxPointDepth(cube, dReal(-0.3),  dReal(0.4),    0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.6) * length, dGeomBoxPointDepth(cube,    0, dReal(-0.3),  dReal(0.4)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.6) * length, dGeomBoxPointDepth(cube,  dReal(0.4),    0, dReal(-0.3)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.6) * length, dGeomBoxPointDepth(cube,  dReal(0.3), dReal(-0.4),    0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.6) * length, dGeomBoxPointDepth(cube,    0,  dReal(0.3), dReal(-0.4)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.6) * length, dGeomBoxPointDepth(cube, dReal(-0.4),    0,  dReal(0.3)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.6) * length, dGeomBoxPointDepth(cube, dReal(-0.3), dReal(-0.4),    0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.6) * length, dGeomBoxPointDepth(cube,    0, dReal(-0.3), dReal(-0.4)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.6) * length, dGeomBoxPointDepth(cube, dReal(-0.4),    0, dReal(-0.3)), PROXIMITY_TOLERANCE);

        // 0.1 length depth
        CHECK_CLOSE(dReal(0.1) * length, dGeomBoxPointDepth(cube, dReal(0.9),   0,   0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.1) * length, dGeomBoxPointDepth(cube,   0, dReal(0.9),   0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.1) * length, dGeomBoxPointDepth(cube,   0,   0, dReal(0.9)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.1) * length, dGeomBoxPointDepth(cube, dReal(-0.9),    0,    0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.1) * length, dGeomBoxPointDepth(cube,    0, dReal(-0.9),    0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(0.1) * length, dGeomBoxPointDepth(cube,    0,    0, dReal(-0.9)), PROXIMITY_TOLERANCE);

        // on surface (zero depth)
        CHECK_EQUAL(dReal(0), dGeomBoxPointDepth(cube, dReal(1.0),   0,   0));
        CHECK_EQUAL(dReal(0), dGeomBoxPointDepth(cube,   0, dReal(1.0),   0));
        CHECK_EQUAL(dReal(0), dGeomBoxPointDepth(cube,   0,   0, dReal(1.0)));
        CHECK_EQUAL(dReal(0), dGeomBoxPointDepth(cube, dReal(-1.0),    0,    0));
        CHECK_EQUAL(dReal(0), dGeomBoxPointDepth(cube,    0, dReal(-1.0),    0));
        CHECK_EQUAL(dReal(0), dGeomBoxPointDepth(cube,    0,    0, dReal(-1.0)));
        CHECK_EQUAL(dReal(0), dGeomBoxPointDepth(cube, dReal(0.3), dReal(1.0),   0));
        CHECK_EQUAL(dReal(0), dGeomBoxPointDepth(cube,   0, dReal(0.3), dReal(1.0)));
        CHECK_EQUAL(dReal(0), dGeomBoxPointDepth(cube, dReal(1.0),   0, dReal(0.3)));
        CHECK_EQUAL(dReal(0), dGeomBoxPointDepth(cube, dReal(-0.3),  dReal(1.0),    0));
        CHECK_EQUAL(dReal(0), dGeomBoxPointDepth(cube,    0, dReal(-0.3),  dReal(1.0)));
        CHECK_EQUAL(dReal(0), dGeomBoxPointDepth(cube,  dReal(1.0),    0, dReal(-0.3)));
        CHECK_EQUAL(dReal(0), dGeomBoxPointDepth(cube,  dReal(0.3), dReal(-1.0),    0));
        CHECK_EQUAL(dReal(0), dGeomBoxPointDepth(cube,    0,  dReal(0.3), dReal(-1.0)));
        CHECK_EQUAL(dReal(0), dGeomBoxPointDepth(cube, dReal(-1.0),    0,  dReal(0.3)));
        CHECK_EQUAL(dReal(0), dGeomBoxPointDepth(cube, dReal(-0.3), dReal(-1.0),    0));
        CHECK_EQUAL(dReal(0), dGeomBoxPointDepth(cube,    0, dReal(-0.3), dReal(-1.0)));
        CHECK_EQUAL(dReal(0), dGeomBoxPointDepth(cube, dReal(-1.0),    0, dReal(-0.3)));

        // 0.1 length from surface (negative depth)
        CHECK_CLOSE(dReal(-0.1) * length, dGeomBoxPointDepth(cube, dReal(1.1),   0,   0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.1) * length, dGeomBoxPointDepth(cube,   0, dReal(1.1),   0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.1) * length, dGeomBoxPointDepth(cube,   0,   0, dReal(1.1)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.1) * length, dGeomBoxPointDepth(cube, dReal(-1.1),    0,    0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.1) * length, dGeomBoxPointDepth(cube,    0, dReal(-1.1),    0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.1) * length, dGeomBoxPointDepth(cube,    0,    0, dReal(-1.1)), PROXIMITY_TOLERANCE);

        // half-length from surface face (negative depth)
        CHECK_EQUAL(dReal(-0.5) * length, dGeomBoxPointDepth(cube, dReal(1.5),   0,   0));
        CHECK_EQUAL(dReal(-0.5) * length, dGeomBoxPointDepth(cube,   0, dReal(1.5),   0));
        CHECK_EQUAL(dReal(-0.5) * length, dGeomBoxPointDepth(cube,   0,   0, dReal(1.5)));
        CHECK_EQUAL(dReal(-0.5) * length, dGeomBoxPointDepth(cube, dReal(-1.5),    0,    0));
        CHECK_EQUAL(dReal(-0.5) * length, dGeomBoxPointDepth(cube,    0, dReal(-1.5),    0));
        CHECK_EQUAL(dReal(-0.5) * length, dGeomBoxPointDepth(cube,    0,    0, dReal(-1.5)));
        CHECK_EQUAL(dReal(-0.5) * length, dGeomBoxPointDepth(cube, dReal(0.3), dReal(1.5),   0));
        CHECK_EQUAL(dReal(-0.5) * length, dGeomBoxPointDepth(cube,   0, dReal(0.3), dReal(1.5)));
        CHECK_EQUAL(dReal(-0.5) * length, dGeomBoxPointDepth(cube, dReal(1.5),   0, dReal(0.3)));
        CHECK_EQUAL(dReal(-0.5) * length, dGeomBoxPointDepth(cube, dReal(-0.3),  dReal(1.5),    0));
        CHECK_EQUAL(dReal(-0.5) * length, dGeomBoxPointDepth(cube,    0, dReal(-0.3),  dReal(1.5)));
        CHECK_EQUAL(dReal(-0.5) * length, dGeomBoxPointDepth(cube,  dReal(1.5),    0, dReal(-0.3)));
        CHECK_EQUAL(dReal(-0.5) * length, dGeomBoxPointDepth(cube,  dReal(0.3), dReal(-1.5),    0));
        CHECK_EQUAL(dReal(-0.5) * length, dGeomBoxPointDepth(cube,    0,  dReal(0.3), dReal(-1.5)));
        CHECK_EQUAL(dReal(-0.5) * length, dGeomBoxPointDepth(cube, dReal(-1.5),    0,  dReal(0.3)));
        CHECK_EQUAL(dReal(-0.5) * length, dGeomBoxPointDepth(cube, dReal(-0.3), dReal(-1.5),    0));
        CHECK_EQUAL(dReal(-0.5) * length, dGeomBoxPointDepth(cube,    0, dReal(-0.3), dReal(-1.5)));
        CHECK_EQUAL(dReal(-0.5) * length, dGeomBoxPointDepth(cube, dReal(-1.5),    0, dReal(-0.3)));
        // half-length from surface edge (negative depth)
        CHECK_CLOSE(dReal(-0.5) * length, dGeomBoxPointDepth(cube, dReal(1.3), dReal(1.4),   0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.5) * length, dGeomBoxPointDepth(cube,   0, dReal(1.3), dReal(1.4)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.5) * length, dGeomBoxPointDepth(cube, dReal(1.4),   0, dReal(1.3)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.5) * length, dGeomBoxPointDepth(cube, dReal(-1.3),  dReal(1.4),    0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.5) * length, dGeomBoxPointDepth(cube,    0, dReal(-1.3),  dReal(1.4)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.5) * length, dGeomBoxPointDepth(cube,  dReal(1.4),    0, dReal(-1.3)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.5) * length, dGeomBoxPointDepth(cube,  dReal(1.3), dReal(-1.4),    0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.5) * length, dGeomBoxPointDepth(cube,    0,  dReal(1.3), dReal(-1.4)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.5) * length, dGeomBoxPointDepth(cube, dReal(-1.4),    0,  dReal(1.3)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.5) * length, dGeomBoxPointDepth(cube, dReal(-1.3), dReal(-1.4),    0), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.5) * length, dGeomBoxPointDepth(cube,    0, dReal(-1.3), dReal(-1.4)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.5) * length, dGeomBoxPointDepth(cube, dReal(-1.4),    0, dReal(-1.3)), PROXIMITY_TOLERANCE);
        // 0.6 length from corner (negative depth)
        CHECK_CLOSE(dReal(-0.6) * length, dGeomBoxPointDepth(cube, dReal(1.2), dReal(1.4), dReal(1.4)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.6) * length, dGeomBoxPointDepth(cube, dReal(1.4), dReal(1.2), dReal(1.4)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.6) * length, dGeomBoxPointDepth(cube, dReal(1.4), dReal(1.4), dReal(1.2)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.6) * length, dGeomBoxPointDepth(cube, dReal(-1.2),  dReal(1.4),  dReal(1.4)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.6) * length, dGeomBoxPointDepth(cube,  dReal(1.4), dReal(-1.2),  dReal(1.4)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.6) * length, dGeomBoxPointDepth(cube,  dReal(1.4),  dReal(1.4), dReal(-1.2)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.6) * length, dGeomBoxPointDepth(cube,  dReal(1.2), dReal(-1.4),  dReal(1.4)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.6) * length, dGeomBoxPointDepth(cube,  dReal(1.4),  dReal(1.2), dReal(-1.4)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.6) * length, dGeomBoxPointDepth(cube, dReal(-1.4),  dReal(1.4),  dReal(1.2)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.6) * length, dGeomBoxPointDepth(cube, dReal(-1.2), dReal(-1.4),  dReal(1.4)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.6) * length, dGeomBoxPointDepth(cube,  dReal(1.4), dReal(-1.2), dReal(-1.4)), PROXIMITY_TOLERANCE);
        CHECK_CLOSE(dReal(-0.6) * length, dGeomBoxPointDepth(cube, dReal(-1.4),  dReal(1.4), dReal(-1.2)), PROXIMITY_TOLERANCE);

        dGeomDestroy(cube);
    }
}
