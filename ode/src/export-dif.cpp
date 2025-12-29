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

/*
 * Export a DIF (Dynamics Interchange Format) file.
 */


// @@@ TODO:
//	* export all spaces, and geoms in spaces, not just ones attached to bodies
//	  (separate export function?)
//	* say the space each geom is in, so reader can construct space hierarchy
//	* limot --> separate out into limits and motors?
//	* make sure ODE-specific parameters divided out


#include <ode/ode.h>
#include "objects.h"
#include "joints/joints.h"
#include "collision_kernel.h"

//***************************************************************************
// utility

struct PrintingContext {
    FILE *file;		// file to write to
    int precision;		// digits of precision to print
    int indent;		// number of levels of indent

    void printIndent() const;
    void printReal (dReal x) const;
    void print (const char *name, int x) const;
    void print (const char *name, unsigned x) const;
    void print (const char *name, dReal x) const;
    void print (const char *name, const dReal *x, int n=3) const;
    void print (const char *name, const char *x=NULL) const;
    void printNonzero (const char *name, dReal x) const;
    void printNonzero (const char *name, const dReal x[3]) const;
};


void PrintingContext::printIndent() const
{
    for (int i=0; i<indent; i++) (void)fputc ('\t',file);
}


void PrintingContext::print (const char *name, int x) const
{
    printIndent();
    (void)fprintf (file,"%s = %d,\n",name,x);
}

void PrintingContext::print (const char *name, unsigned x) const
{
    printIndent();
    (void)fprintf (file,"%s = %u,\n",name,x);
}

void PrintingContext::printReal (dReal x) const
{
    if (x==dInfinity) {
        (void)fprintf (file,"inf");
    }
    else if (x==-dInfinity) {
        (void)fprintf (file,"-inf");
    }
    else {
        (void)fprintf (file,"%.*g",precision,(double)x);
    }
}


void PrintingContext::print (const char *name, dReal x) const
{
    printIndent();
    (void)fprintf (file,"%s = ",name);
    printReal (x);
    (void)fprintf (file,",\n");
}


void PrintingContext::print (const char *name, const dReal *x, int n) const
{
    printIndent();
    (void)fprintf (file,"%s = {",name);
    for (int i=0; i<n; i++) {
        printReal (x[i]);
        if (i < n-1) (void)fputc (',',file);
    }
    (void)fprintf (file,"},\n");
}


void PrintingContext::print (const char *name, const char *x) const
{
    printIndent();
    if (x) {
        (void)fprintf (file,"%s = \"%s\",\n",name,x);
    }
    else {
        (void)fprintf (file,"%s\n",name);
    }
}


void PrintingContext::printNonzero (const char *name, dReal x) const
{
    if (x != 0) print (name,x);
}


void PrintingContext::printNonzero (const char *name, const dReal x[3]) const
{
    if (x[0] != 0 && x[1] != 0 && x[2] != 0) print (name,x);
}

//***************************************************************************
// joints


static void printLimot (PrintingContext &c, const dxJointLimitMotor &limot, int num)
{
    if (num >= 0) {
        c.printIndent();
        (void)fprintf (c.file,"limit%d = {\n",num);
    }
    else {
        c.print ("limit = {");
    }
    c.indent++;
    c.print ("low_stop",limot.lostop);
    c.print ("high_stop",limot.histop);
    c.printNonzero ("bounce",limot.bounce);
    c.print ("ODE = {");
    c.indent++;
    c.printNonzero ("stop_erp",limot.stop_erp);
    c.printNonzero ("stop_cfm",limot.stop_cfm);
    c.indent--;
    c.print ("},");
    c.indent--;
    c.print ("},");

    if (num >= 0) {
        c.printIndent();
        (void)fprintf (c.file,"motor%d = {\n",num);
    }
    else {
        c.print ("motor = {");
    }
    c.indent++;
    c.printNonzero ("vel",limot.vel);
    c.printNonzero ("fmax",limot.fmax);
    c.print ("ODE = {");
    c.indent++;
    c.printNonzero ("fudge_factor",limot.fudge_factor);
    c.printNonzero ("normal_cfm",limot.normal_cfm);
    c.indent--;
    c.print ("},");
    c.indent--;
    c.print ("},");
}


static const char *getJointName (const dxJoint *j)
{
    switch (j->type()) {
        case dJointTypeBall: return "ball";
        case dJointTypeHinge: return "hinge";
        case dJointTypeSlider: return "slider";
        case dJointTypeContact: return "contact";
        case dJointTypeUniversal: return "universal";
        case dJointTypeHinge2: return "ODE_hinge2";
        case dJointTypeFixed: return "fixed";
        case dJointTypeNull: return "null";
        case dJointTypeAMotor: return "ODE_angular_motor";
        case dJointTypeLMotor: return "ODE_linear_motor";
        case dJointTypePR: return "PR";
        case dJointTypePU: return "PU";
        case dJointTypePiston: return "piston";
        case dJointTypePlane2D: return "plane2d";
        case dJointTypeDBall: return "dball";
        case dJointTypeDHinge: return "dhinge";
        case dJointTypeTransmission: return "transmission";
        case dJointTypeNone: return "none";
    }
    return "unknown";
}


static void printBall (PrintingContext &c, const dxJoint *j)
{
    const dxJointBall *b = (const dxJointBall*) j;
    c.print ("anchor1",b->anchor1);
    c.print ("anchor2",b->anchor2);
    c.print ("erp",b->erp);
    c.print ("cfm",b->cfm);
}


static void printHinge (PrintingContext &c, const dxJoint *j)
{
    const dxJointHinge *h = (const dxJointHinge*) j;
    c.print ("anchor1",h->anchor1);
    c.print ("anchor2",h->anchor2);
    c.print ("axis1",h->axis1);
    c.print ("axis2",h->axis2);
    c.print ("qrel",h->qrel,4);
    printLimot (c,h->limot,-1);
}


static void printSlider (PrintingContext &c, const dxJoint *j)
{
    const dxJointSlider *s = (const dxJointSlider*) j;
    c.print ("axis1",s->axis1);
    c.print ("qrel",s->qrel,4);
    c.print ("offset",s->offset);
    printLimot (c,s->limot,-1);
}


static void printContact (PrintingContext &c, const dxJoint *j)
{
    const dxJointContact *ct = (const dxJointContact*) j;
    const int mode = ct->contact.surface.mode;
    c.print ("pos",ct->contact.geom.pos);
    c.print ("normal",ct->contact.geom.normal);
    c.print ("depth",ct->contact.geom.depth);
    //@@@ may want to write the geoms g1 and g2 that are involved, for debugging.
    //    to do this we must have written out all geoms in all spaces, not just
    //    geoms that are attached to bodies.
    c.print ("mu",ct->contact.surface.mu);
    if (mode & dContactMu2) c.print ("mu2",ct->contact.surface.mu2);
    if (mode & dContactBounce) {
        c.print ("bounce",ct->contact.surface.bounce);
        c.print ("bounce_vel",ct->contact.surface.bounce_vel);
    }
    if (mode & dContactSoftERP) c.print ("soft_ERP",ct->contact.surface.soft_erp);
    if (mode & dContactSoftCFM) c.print ("soft_CFM",ct->contact.surface.soft_cfm);
    if (mode & dContactMotion1) c.print ("motion1",ct->contact.surface.motion1);
    if (mode & dContactMotion2) c.print ("motion2",ct->contact.surface.motion2);
    if (mode & dContactSlip1) c.print ("slip1",ct->contact.surface.slip1);
    if (mode & dContactSlip2) c.print ("slip2",ct->contact.surface.slip2);
    int fa = 0;		// friction approximation code
    if (mode & dContactApprox1_1) fa |= 1;
    if (mode & dContactApprox1_2) fa |= 2;
    if (fa) c.print ("friction_approximation",fa);
    if (mode & dContactFDir1) c.print ("fdir1",ct->contact.fdir1);
}


static void printUniversal (PrintingContext &c, const dxJoint *j)
{
    const dxJointUniversal *u = (const dxJointUniversal*) j;
    c.print ("anchor1",u->anchor1);
    c.print ("anchor2",u->anchor2);
    c.print ("axis1",u->axis1);
    c.print ("axis2",u->axis2);
    c.print ("qrel1",u->qrel1,4);
    c.print ("qrel2",u->qrel2,4);
    printLimot (c,u->limot1,1);
    printLimot (c,u->limot2,2);
}


static void printHinge2 (PrintingContext &c, const dxJoint *j)
{
    const dxJointHinge2 *h = (const dxJointHinge2*) j;
    c.print ("anchor1",h->anchor1);
    c.print ("anchor2",h->anchor2);
    c.print ("axis1",h->axis1);
    c.print ("axis2",h->axis2);
    c.print ("v1",h->v1);	//@@@ much better to write out 'qrel' here, if it's available
    c.print ("v2",h->v2);
    c.print ("susp_erp",h->susp_erp);
    c.print ("susp_cfm",h->susp_cfm);
    printLimot (c,h->limot1,1);
    printLimot (c,h->limot2,2);
}

static void printPR (PrintingContext &c, const dxJoint *j)
{
    const dxJointPR *pr = (const dxJointPR*) j;
    c.print ("anchor2",pr->anchor2);
    c.print ("axisR1",pr->axisR1);
    c.print ("axisR2",pr->axisR2);
    c.print ("axisP1",pr->axisP1);
    c.print ("qrel",pr->qrel,4);
    c.print ("offset",pr->offset);
    printLimot (c,pr->limotP,1);
    printLimot (c,pr->limotR,2);
}

static void printPU (PrintingContext &c, const dxJoint *j)
{
    const dxJointPU *pu = (const dxJointPU*) j;
    c.print ("anchor1",pu->anchor1);
    c.print ("anchor2",pu->anchor2);
    c.print ("axis1",pu->axis1);
    c.print ("axis2",pu->axis2);
    c.print ("axisP",pu->axisP1);
    c.print ("qrel1",pu->qrel1,4);
    c.print ("qrel2",pu->qrel2,4);
    printLimot (c,pu->limot1,1);
    printLimot (c,pu->limot2,2);
    printLimot (c,pu->limotP,3);
}

static void printPiston (PrintingContext &c, const dxJoint *j)
{
    const dxJointPiston *p = (const dxJointPiston*) j;
    c.print ("anchor1",p->anchor1);
    c.print ("anchor2",p->anchor2);
    c.print ("axis1",p->axis1);
    c.print ("axis2",p->axis2);
    c.print ("qrel",p->qrel,4);
    printLimot (c,p->limotP,1);
    printLimot (c, p->limotR, 2);
}

static void printFixed (PrintingContext &c, const dxJoint *j)
{
    const dxJointFixed *f = (const dxJointFixed*) j;
    c.print ("qrel",f->qrel);
    c.print ("offset",f->offset);
    c.print ("erp",f->erp);
    c.print ("cfm",f->cfm);
}

static void printLMotor (PrintingContext &c, const dxJoint *j)
{
    const dxJointLMotor *lm = (const dxJointLMotor*) j;
    c.print("num", lm->num);
    c.printIndent();
    (void)fprintf (c.file,"rel = {%d,%d,%d},\n",lm->rel[0],lm->rel[1],lm->rel[2]);
    c.print ("axis1",lm->axis[0]);
    c.print ("axis2",lm->axis[1]);
    c.print ("axis3",lm->axis[2]);
    for (int i=0; i<3; i++) printLimot (c,lm->limot[i],i+1);
}

static void printPlane2d (PrintingContext &c, const dxJoint *j)
{
    const dxJointPlane2D *pl = (const dxJointPlane2D*) j;
    c.print ("row_motor_x"    ,pl->row_motor_x);
    c.print ("row_motor_y"    ,pl->row_motor_y);
    c.print ("row_motor_angle",pl->row_motor_angle);
    printLimot (c,pl->motor_x    ,-1);
    printLimot (c,pl->motor_y    ,-1);
    printLimot (c,pl->motor_angle,-1);
}

static void printDBall (PrintingContext &c, const dxJoint *j)
{
    const dxJointDBall *b = (const dxJointDBall*) j;
    c.print ("anchor1",b->anchor1);
    c.print ("anchor2",b->anchor2);
    c.print ("targetdist",b->targetDistance);
    c.print ("erp",b->erp);
    c.print ("cfm",b->cfm);
}

static void printDHinge (PrintingContext &c, const dxJoint *j)
{
    const dxJointDHinge *dh = (const dxJointDHinge*) j;
    printDBall(c,j);
    c.print ("axis1",dh->axis1);
    c.print ("axis2",dh->axis2);
}

static void printTransmission (PrintingContext &c, const dxJoint *j)
{
    const dxJointTransmission *tm = (const dxJointTransmission*) j;
    c.print ("mode",tm->mode);
    c.print ("update",tm->update);
    c.print ("contacts1",tm->contacts[0]);
    c.print ("contacts2",tm->contacts[1]);
    c.print ("axes1",tm->axes[0]);
    c.print ("axes2",tm->axes[1]);
    c.print ("anchor1",tm->anchors[0]);
    c.print ("anchor2",tm->anchors[1]);

    c.print ("reference1",tm->reference[0],16);
    c.print ("reference2",tm->reference[1],16);

    c.print ("phase1",tm->phase[0]);
    c.print ("phase2",tm->phase[1]);
    c.print ("radii1",tm->radii[0]);
    c.print ("radii2",tm->radii[1]);
    c.print ("backlash",tm->backlash);
    c.print ("ratio",tm->ratio);
    c.print ("erp",tm->erp);
    c.print ("cfm",tm->cfm);
}

struct dxAMotorJointPrinter
{
    static void print(PrintingContext &c, const dxJointAMotor *a)
    {
        c.print ("num",a->m_num);
        c.print ("mode",a->m_mode);
        c.printIndent();
        (void)fprintf (c.file,"rel = {%d,%d,%d},\n",a->m_rel[0],a->m_rel[1],a->m_rel[2]);
        c.print ("axis1",a->m_axis[0]);
        c.print ("axis2",a->m_axis[1]);
        c.print ("axis3",a->m_axis[2]);
        for (int i=0; i<3; i++) printLimot (c,a->m_limot[i],i+1);
        c.print ("angle1",a->m_angle[0]);
        c.print ("angle2",a->m_angle[1]);
        c.print ("angle3",a->m_angle[2]);
    }
};

static void printAMotor (PrintingContext &c, const dxJoint *j)
{
    const dxJointAMotor *a = (const dxJointAMotor*) j;
    dxAMotorJointPrinter::print(c, a);
}

//***************************************************************************
// geometry

static void printSphere (PrintingContext &c, dxGeom *g)
{
    c.print ("type","sphere");
    c.print ("radius",dGeomSphereGetRadius (g));
}


static void printBox (PrintingContext &c, dxGeom *g)
{
    dVector3 sides;
    dGeomBoxGetLengths (g,sides);
    c.print ("type","box");
    c.print ("sides",sides);
}


static void printCapsule (PrintingContext &c, dxGeom *g)
{
    dReal radius,length;
    dGeomCapsuleGetParams (g,&radius,&length);
    c.print ("type","capsule");
    c.print ("radius",radius);
    c.print ("length",length);
}


static void printCylinder (PrintingContext &c, dxGeom *g)
{
    dReal radius,length;
    dGeomCylinderGetParams (g,&radius,&length);
    c.print ("type","cylinder");
    c.print ("radius",radius);
    c.print ("length",length);
}


static void printPlane (PrintingContext &c, dxGeom *g)
{
    dVector4 e;
    dGeomPlaneGetParams (g,e);
    c.print ("type","plane");
    c.print ("normal",e);
    c.print ("d",e[3]);
}


static void printRay (PrintingContext &c, dxGeom *g)
{
    const dReal length = dGeomRayGetLength (g);
    c.print ("type","ray");
    c.print ("length",length);
}


static void printConvex (PrintingContext &c, dxGeom * /*g*/)
{
    c.print ("type","convex");
    ///@todo Print information about convex hull
}



static void printTriMesh (PrintingContext &c, dxGeom * /*g*/)
{
    c.print ("type","trimesh");
    //@@@ i don't think that the trimesh accessor functions are really
    //    sufficient to read out all the triangle data, and anyway we
    //    should have a method of not duplicating trimesh data that is
    //    shared.
}


static void printHeightfieldClass (PrintingContext &c, dxGeom * /*g*/)
{
    c.print ("type","heightfield");
    ///@todo Print information about heightfield
}


static void printGeom (PrintingContext &c, dxGeom *g)
{
    const unsigned long category = dGeomGetCategoryBits (g);
    if (category != ~(unsigned long)0) {
        c.printIndent();
        (void)fprintf (c.file,"category_bits = %lu\n",category);
    }
    const unsigned long collide = dGeomGetCollideBits (g);
    if (collide != ~(unsigned long)0) {
        c.printIndent();
        (void)fprintf (c.file,"collide_bits = %lu\n",collide);
    }
    if (!dGeomIsEnabled (g)) {
        c.print ("disabled",1);
    }
    switch (g->type) {
        case dSphereClass:      printSphere           (c,g); break;
        case dBoxClass:         printBox              (c,g); break;
        case dCapsuleClass:     printCapsule          (c,g); break;
        case dCylinderClass:    printCylinder         (c,g); break;
        case dPlaneClass:       printPlane            (c,g); break;
        case dRayClass:         printRay              (c,g); break;
        case dConvexClass:      printConvex           (c,g); break;
        case dTriMeshClass:     printTriMesh          (c,g); break;
        case dHeightfieldClass: printHeightfieldClass (c,g); break;
        default: break;
    }
}

//***************************************************************************
// world

void dWorldExportDIF (dWorldID w, FILE *file, const char *prefix)
{
    PrintingContext c = {};
    c.file = file;
#if defined(dSINGLE)
    c.precision = 7;
#else
    c.precision = 15;
#endif
    c.indent = 1;

    (void)fprintf (file,"-- Dynamics Interchange Format v0.1\n\n%sworld = dynamics.world {\n",prefix);
    c.print ("gravity",w->gravity);
    c.print ("ODE = {");
    c.indent++;
    c.print ("ERP",w->global_erp);
    c.print ("CFM",w->global_cfm);
    c.print ("auto_disable = {");
    c.indent++;
    c.print ("linear_threshold",w->adis.linear_average_threshold);
    c.print ("angular_threshold",w->adis.angular_average_threshold);
    c.print ("average_samples",(int)w->adis.average_samples);
    c.print ("idle_time",w->adis.idle_time);
    c.print ("idle_steps",w->adis.idle_steps);
    (void)fprintf (file,"\t\t},\n\t},\n}\n");
    c.indent -= 3;

    // bodies
    int num = 0;
    (void)fprintf (file,"%sbody = {}\n",prefix);
    for (dxBody *b=w->firstbody; b; b=(dxBody*)b->next) {
        b->tag = num;
        (void)fprintf (file,"%sbody[%d] = dynamics.body {\n\tworld = %sworld,\n",prefix,num,prefix);
        c.indent++;
        c.print ("pos",b->posr.pos);
        c.print ("q",b->q,4);
        c.print ("lvel",b->lvel);
        c.print ("avel",b->avel);
        c.print ("mass",b->mass.mass);
        (void)fprintf (file,"\tI = {{");
        for (int i=0; i<3; i++) {
            for (int j=0; j<3; j++) {
                c.printReal (b->mass.I[i*4+j]);
                if (j < 2) (void)fputc (',',file);
            }
            if (i < 2) (void)fprintf (file,"},{");
        }
        (void)fprintf (file,"}},\n");
        c.printNonzero ("com",b->mass.c);
        c.print ("ODE = {");
        c.indent++;
        if (b->flags & dxBodyFlagFiniteRotation) c.print ("finite_rotation",1);
        if (b->flags & dxBodyDisabled) c.print ("disabled",1);
        if (b->flags & dxBodyNoGravity) c.print ("no_gravity",1);
        if (b->flags & dxBodyAutoDisable) {
            c.print ("auto_disable = {");
            c.indent++;
            c.print ("linear_threshold",b->adis.linear_average_threshold);
            c.print ("angular_threshold",b->adis.angular_average_threshold);
            c.print ("average_samples",(int)b->adis.average_samples);
            c.print ("idle_time",b->adis.idle_time);
            c.print ("idle_steps",b->adis.idle_steps);
            c.print ("time_left",b->adis_timeleft);
            c.print ("steps_left",b->adis_stepsleft);
            c.indent--;
            c.print ("},");
        }
        c.printNonzero ("facc",b->facc);
        c.printNonzero ("tacc",b->tacc);
        if (b->flags & dxBodyFlagFiniteRotationAxis) {
            c.print ("finite_rotation_axis",b->finite_rot_axis);
        }
        c.indent--;
        c.print ("},");
        if (b->geom) {
            c.print ("geometry = {");
            c.indent++;
            for (dxGeom *g=b->geom; g; g=g->body_next) {
                c.print ("{");
                c.indent++;
                printGeom (c,g);
                c.indent--;
                c.print ("},");
            }
            c.indent--;
            c.print ("},");
        }
        c.indent--;
        c.print ("}");
        num++;
    }

    // joints
    num = 0;
    (void)fprintf (file,"%sjoint = {}\n",prefix);
    for (dxJoint *j=w->firstjoint; j; j=(dxJoint*)j->next) {
        c.indent++;
        const char *name = getJointName (j);
        (void)fprintf (file,
            "%sjoint[%d] = dynamics.%s_joint {\n"
            "\tworld = %sworld,\n"
            "\tbody = {"
            ,prefix,num,name,prefix);

        if ( j->node[0].body )
            (void)fprintf (file,"%sbody[%d]",prefix,j->node[0].body->tag);
        if ( j->node[1].body )
            (void)fprintf (file,",%sbody[%d]",prefix,j->node[1].body->tag);
        (void)fprintf (file,"}\n");

        switch (j->type()) {
            case dJointTypeBall:         printBall         (c,j); break;
            case dJointTypeHinge:        printHinge        (c,j); break;
            case dJointTypeSlider:       printSlider       (c,j); break;
            case dJointTypeContact:      printContact      (c,j); break;
            case dJointTypeUniversal:    printUniversal    (c,j); break;
            case dJointTypeHinge2:       printHinge2       (c,j); break;
            case dJointTypeFixed:        printFixed        (c,j); break;
            case dJointTypeAMotor:       printAMotor       (c,j); break;
            case dJointTypeLMotor:       printLMotor       (c,j); break;
            case dJointTypePR:           printPR           (c,j); break;
            case dJointTypePU:           printPU           (c,j); break;
            case dJointTypePiston:       printPiston       (c,j); break;
            case dJointTypePlane2D:      printPlane2d      (c,j); break;
            case dJointTypeDBall:        printDBall        (c,j); break;
            case dJointTypeDHinge:       printDHinge       (c,j); break;
            case dJointTypeTransmission: printTransmission (c,j); break;
            case dJointTypeNull:                                  break;
            case dJointTypeNone: c.print("unknown joint");
        }
        c.indent--;
        c.print ("}");
        num++;
    }
}
