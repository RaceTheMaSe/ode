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


#ifndef ODE_ODECPP_H
#define ODE_ODECPP_H
#ifdef __cplusplus
#include <ode/odeconfig.h>
#include <ode/common.h>
#include <ode/objects.h>

//namespace ode {


class ODE_API dWorldSimpleIDContainer {
public:
    dWorldSimpleIDContainer(const dWorldSimpleIDContainer& rhs) ODE_DELETE;
    dWorldSimpleIDContainer& operator=(const dWorldSimpleIDContainer& rhs) ODE_DELETE;

protected:
	dWorldID _id;

	dWorldSimpleIDContainer(): _id(NULL) {}
	~dWorldSimpleIDContainer() { destroy(); }

	void destroy() { 
		if (_id) {
			dWorldDestroy(_id); 
			_id = NULL;
		}
	}
};

class ODE_API dWorldDynamicIDContainer: public dWorldSimpleIDContainer {
public:
    dWorldDynamicIDContainer(const dWorldDynamicIDContainer& rhs) ODE_DELETE;
    dWorldDynamicIDContainer& operator=(const dWorldDynamicIDContainer& rhs) ODE_DELETE;
	  virtual ~dWorldDynamicIDContainer();

protected:
  dWorldDynamicIDContainer() : dWorldSimpleIDContainer() {}
};

template <class dWorldTemplateBase>
class dWorldTemplate: public dWorldTemplateBase {
public:
  // intentionally undefined, don't use these
  dWorldTemplate (const dWorldTemplate<dWorldTemplateBase> &) ODE_DELETE;
  dWorldTemplate& operator= (const dWorldTemplate<dWorldTemplateBase> &) ODE_DELETE;

protected:
  dWorldID get_id() const { return dWorldTemplateBase::_id; }
  void set_id(dWorldID value) { dWorldTemplateBase::_id = value; }

public:
  dWorldTemplate()
    { set_id(dWorldCreate()); }

  dWorldID id() const
    { return get_id(); }
  operator dWorldID() const
    { return get_id(); }

  void setGravity (dReal x, dReal y, dReal z)
    { dWorldSetGravity (get_id(), x, y, z); }
  void setGravity (const dVector3 g)
    { setGravity (g[0], g[1], g[2]); }
  void getGravity (dVector3 g) const
    { dWorldGetGravity (get_id(), g); }

  void setERP (dReal erp)
    { dWorldSetERP(get_id(), erp); }
  dReal getERP() const
    { return dWorldGetERP(get_id()); }

  void setCFM (dReal cfm)
    { dWorldSetCFM(get_id(), cfm); }
  dReal getCFM() const
    { return dWorldGetCFM(get_id()); }

  void step (dReal stepsize)
    { dWorldStep (get_id(), stepsize); }

  void quickStep(dReal stepsize)
    { dWorldQuickStep (get_id(), stepsize); }
  void setQuickStepNumIterations(int num)
    { dWorldSetQuickStepNumIterations (get_id(), num); }
  int getQuickStepNumIterations() const
    { return dWorldGetQuickStepNumIterations (get_id()); }
  void setQuickStepW(dReal over_relaxation)
    { dWorldSetQuickStepW (get_id(), over_relaxation); }
  dReal getQuickStepW() const
    { return dWorldGetQuickStepW (get_id()); }

  void  setAutoDisableLinearThreshold (dReal threshold) 
    { dWorldSetAutoDisableLinearThreshold (get_id(), threshold); }
  dReal getAutoDisableLinearThreshold() const
    { return dWorldGetAutoDisableLinearThreshold (get_id()); }
  void setAutoDisableAngularThreshold (dReal threshold)
    { dWorldSetAutoDisableAngularThreshold (get_id(), threshold); }
  dReal getAutoDisableAngularThreshold() const
    { return dWorldGetAutoDisableAngularThreshold (get_id()); }
  void setAutoDisableSteps (int steps)
    { dWorldSetAutoDisableSteps (get_id(), steps); }
  int getAutoDisableSteps() const
    { return dWorldGetAutoDisableSteps (get_id()); }
  void setAutoDisableTime (dReal time)
    { dWorldSetAutoDisableTime (get_id(), time); }
  dReal getAutoDisableTime() const
    { return dWorldGetAutoDisableTime (get_id()); }
  void setAutoDisableFlag (int do_auto_disable)
    { dWorldSetAutoDisableFlag (get_id(), do_auto_disable); }
  int getAutoDisableFlag() const
    { return dWorldGetAutoDisableFlag (get_id()); }

  dReal getLinearDampingThreshold() const
    { return dWorldGetLinearDampingThreshold(get_id()); }
  void setLinearDampingThreshold(dReal threshold)
    { dWorldSetLinearDampingThreshold(get_id(), threshold); }
  dReal getAngularDampingThreshold() const
    { return dWorldGetAngularDampingThreshold(get_id()); }
  void setAngularDampingThreshold(dReal threshold)
    { dWorldSetAngularDampingThreshold(get_id(), threshold); }
  dReal getLinearDamping() const
    { return dWorldGetLinearDamping(get_id()); }
  void setLinearDamping(dReal scale)
    { dWorldSetLinearDamping(get_id(), scale); }
  dReal getAngularDamping() const
    { return dWorldGetAngularDamping(get_id()); }
  void setAngularDamping(dReal scale)
    { dWorldSetAngularDamping(get_id(), scale); }
  void setDamping(dReal linear_scale, dReal angular_scale)
    { dWorldSetDamping(get_id(), linear_scale, angular_scale); }

  dReal getMaxAngularSpeed() const
    { return dWorldGetMaxAngularSpeed(get_id()); }
  void setMaxAngularSpeed(dReal max_speed)
    { dWorldSetMaxAngularSpeed(get_id(), max_speed); }

  void setContactSurfaceLayer(dReal depth)
    { dWorldSetContactSurfaceLayer (get_id(), depth); }
  dReal getContactSurfaceLayer() const
    { return dWorldGetContactSurfaceLayer (get_id()); }

  void impulseToForce (dReal stepsize, dReal ix, dReal iy, dReal iz, 
		       dVector3 force)
    { dWorldImpulseToForce (get_id(), stepsize, ix, iy, iz, force); }
};


class ODE_API dBodySimpleIDContainer {
public:
	virtual ~dBodySimpleIDContainer() { destroy(); }

protected:
	dBodyID _id;

	dBodySimpleIDContainer(): _id(NULL) {}

	virtual void destroy();
};

class ODE_API dBodyDynamicIDContainer: public dBodySimpleIDContainer {
public:
  dBodyDynamicIDContainer(const dBodyDynamicIDContainer& rhs) ODE_DELETE;
  dBodyDynamicIDContainer& operator=(const dBodyDynamicIDContainer& rhs) ODE_DELETE;
	~dBodyDynamicIDContainer() ODE_OVERRIDE;

protected:
  dBodyDynamicIDContainer() : dBodySimpleIDContainer() {}
};

template <class dBodyTemplateBase, class dWorldTemplateBase>
class dBodyTemplate: public dBodyTemplateBase {
public:
  // intentionally undefined, don't use these
  dBodyTemplate (const dBodyTemplate<dBodyTemplateBase, dWorldTemplateBase> &) ODE_DELETE;
  dBodyTemplate& operator= (const dBodyTemplate<dBodyTemplateBase, dWorldTemplateBase> &) ODE_DELETE;

protected:
  virtual dBodyID get_id() const { return dBodyTemplateBase::_id; }
  virtual void set_id(dBodyID value) { dBodyTemplateBase::_id = value; }

public:
  dBodyTemplate()
    { }
  explicit dBodyTemplate (dWorldID wrld)
    { set_id(dBodyCreate(wrld)); }
  explicit dBodyTemplate (dWorldTemplate<dWorldTemplateBase>& wrld)
    { set_id(dBodyCreate(wrld.id())); }

  void create (dWorldID wrld) {
    dBodyTemplateBase::destroy();
    set_id(dBodyCreate(wrld));
  }
  void create (dWorldTemplate<dWorldTemplateBase>& wrld) {
    create(wrld.id());
  }

  dBodyID id() const
    { return get_id(); }
  operator dBodyID() const
    { return get_id(); }

  void setData (void *data)
    { dBodySetData (get_id(), data); }
  void *getData() const
    { return dBodyGetData (get_id()); }

  void setPosition (dReal x, dReal y, dReal z)
    { dBodySetPosition (get_id(), x, y, z); }
  void setPosition (const dVector3 p)
    { setPosition(p[0], p[1], p[2]); }

  void setRotation (const dMatrix3 R)
    { dBodySetRotation (get_id(), R); }
  void setQuaternion (const dQuaternion q)
    { dBodySetQuaternion (get_id(), q); }
  void setLinearVel (dReal x, dReal y, dReal z)
    { dBodySetLinearVel (get_id(), x, y, z); }
  void setLinearVel (const dVector3 v)
    { setLinearVel(v[0], v[1], v[2]); }
  void setAngularVel (dReal x, dReal y, dReal z)
    { dBodySetAngularVel (get_id(), x, y, z); }
  void setAngularVel (const dVector3 v)
    { setAngularVel (v[0], v[1], v[2]); }

  const dReal * getPosition() const
    { return dBodyGetPosition (get_id()); }
  const dReal * getRotation() const
    { return dBodyGetRotation (get_id()); }
  const dReal * getQuaternion() const
    { return dBodyGetQuaternion (get_id()); }
  const dReal * getLinearVel() const
    { return dBodyGetLinearVel (get_id()); }
  const dReal * getAngularVel() const
    { return dBodyGetAngularVel (get_id()); }

  void setMass (const dMass *mass)
    { dBodySetMass (get_id(), mass); }
  void setMass (const dMass &mass)
    { setMass (&mass); }
  dMass getMass () const
    { dMass mass; dBodyGetMass (get_id(), &mass); return mass; }

  void addForce (dReal fx, dReal fy, dReal fz)
    { dBodyAddForce (get_id(), fx, fy, fz); }
  void addForce (const dVector3 f)
    { addForce (f[0], f[1], f[2]); }
  void addTorque (dReal fx, dReal fy, dReal fz)
    { dBodyAddTorque (get_id(), fx, fy, fz); }
  void addTorque (const dVector3 t)
    { addTorque(t[0], t[1], t[2]); }

  void addRelForce (dReal fx, dReal fy, dReal fz)
    { dBodyAddRelForce (get_id(), fx, fy, fz); }
  void addRelForce (const dVector3 f)
    { addRelForce (f[0], f[1], f[2]); }
  void addRelTorque (dReal fx, dReal fy, dReal fz)
    { dBodyAddRelTorque (get_id(), fx, fy, fz); }
  void addRelTorque (const dVector3 t)
    { addRelTorque (t[0], t[1], t[2]); }

  void addForceAtPos (dReal fx, dReal fy, dReal fz, 
		      dReal px, dReal py, dReal pz)
    { dBodyAddForceAtPos (get_id(), fx, fy, fz, px, py, pz); }
  void addForceAtPos (const dVector3 f, const dVector3 p)
    { addForceAtPos (f[0], f[1], f[2], p[0], p[1], p[2]); }

  void addForceAtRelPos (dReal fx, dReal fy, dReal fz, 
                         dReal px, dReal py, dReal pz)
    { dBodyAddForceAtRelPos (get_id(), fx, fy, fz, px, py, pz); }
  void addForceAtRelPos (const dVector3 f, const dVector3 p)
    { addForceAtRelPos (f[0], f[1], f[2], p[0], p[1], p[2]); }

  void addRelForceAtPos (dReal fx, dReal fy, dReal fz, 
			 dReal px, dReal py, dReal pz)
    { dBodyAddRelForceAtPos (get_id(), fx, fy, fz, px, py, pz); }
  void addRelForceAtPos (const dVector3 f, const dVector3 p)
    { addRelForceAtPos (f[0], f[1], f[2], p[0], p[1], p[2]); }

  void addRelForceAtRelPos (dReal fx, dReal fy, dReal fz, 
			    dReal px, dReal py, dReal pz)
    { dBodyAddRelForceAtRelPos (get_id(), fx, fy, fz, px, py, pz); }
  void addRelForceAtRelPos (const dVector3 f, const dVector3 p)
    { addRelForceAtRelPos (f[0], f[1], f[2], p[0], p[1], p[2]); }

  const dReal * getForce() const
    { return dBodyGetForce(get_id()); }
  const dReal * getTorque() const
    { return dBodyGetTorque(get_id()); }
  void setForce (dReal x, dReal y, dReal z)
    { dBodySetForce (get_id(), x, y, z); }
  void setForce (const dVector3 f)
    { setForce (f[0], f[1], f[2]); }
  void setTorque (dReal x, dReal y, dReal z)
    { dBodySetTorque (get_id(), x, y, z); }
  void setTorque (const dVector3 t)
  { setTorque (t[0], t[1], t[2]); }

  void setDynamic()
    { dBodySetDynamic (get_id()); }
  void setKinematic()
    { dBodySetKinematic (get_id()); }
  bool isKinematic() const
    { return dBodyIsKinematic (get_id()) != 0; }

  void enable()
    { dBodyEnable (get_id()); }
  void disable()
    { dBodyDisable (get_id()); }
  bool isEnabled() const
    { return dBodyIsEnabled (get_id()) != 0; }

  void getRelPointPos (dReal px, dReal py, dReal pz, dVector3 result) const
    { dBodyGetRelPointPos (get_id(), px, py, pz, result); }
  void getRelPointPos (const dVector3 p, dVector3 result) const
    { getRelPointPos (p[0], p[1], p[2], result); }

  void getRelPointVel (dReal px, dReal py, dReal pz, dVector3 result) const
    { dBodyGetRelPointVel (get_id(), px, py, pz, result); }
  void getRelPointVel (const dVector3 p, dVector3 result) const
    { getRelPointVel (p[0], p[1], p[2], result); }

  void getPointVel (dReal px, dReal py, dReal pz, dVector3 result) const
    { dBodyGetPointVel (get_id(), px, py, pz, result); }
  void getPointVel (const dVector3 p, dVector3 result) const
    { getPointVel (p[0], p[1], p[2], result); }

  void getPosRelPoint (dReal px, dReal py, dReal pz, dVector3 result) const
    { dBodyGetPosRelPoint (get_id(), px, py, pz, result); }
  void getPosRelPoint (const dVector3 p, dVector3 result) const
    { getPosRelPoint (p[0], p[1], p[2], result); }

  void vectorToWorld (dReal px, dReal py, dReal pz, dVector3 result) const
    { dBodyVectorToWorld (get_id(), px, py, pz, result); }
  void vectorToWorld (const dVector3 p, dVector3 result) const
    { vectorToWorld (p[0], p[1], p[2], result); }

  void vectorFromWorld (dReal px, dReal py, dReal pz, dVector3 result) const
    { dBodyVectorFromWorld (get_id(), px, py, pz, result); }
  void vectorFromWorld (const dVector3 p, dVector3 result) const
    { vectorFromWorld (p[0], p[1], p[2], result); }

  void setFiniteRotationMode (bool mode)
    { dBodySetFiniteRotationMode (get_id(), mode); }

  void setFiniteRotationAxis (dReal x, dReal y, dReal z)
    { dBodySetFiniteRotationAxis (get_id(), x, y, z); }
  void setFiniteRotationAxis (const dVector3 a)
    { setFiniteRotationAxis (a[0], a[1], a[2]); }

  bool getFiniteRotationMode() const
    { return dBodyGetFiniteRotationMode (get_id()) != 0; }
  void getFiniteRotationAxis (dVector3 result) const
    { dBodyGetFiniteRotationAxis (get_id(), result); }

  int getNumJoints() const
    { return dBodyGetNumJoints (get_id()); }
  dJointID getJoint (int index) const
    { return dBodyGetJoint (get_id(), index); }

  void setGravityMode (bool mode)
    { dBodySetGravityMode (get_id(), mode); }
  bool getGravityMode() const
    { return dBodyGetGravityMode (get_id()) != 0; }

  bool isConnectedTo (dBodyID body) const
    { return dAreConnected (get_id(), body) != 0; }

  void  setAutoDisableLinearThreshold (dReal threshold) 
    { dBodySetAutoDisableLinearThreshold (get_id(), threshold); }
  dReal getAutoDisableLinearThreshold() const
    { return dBodyGetAutoDisableLinearThreshold (get_id()); }
  void setAutoDisableAngularThreshold (dReal threshold)
    { dBodySetAutoDisableAngularThreshold (get_id(), threshold); }
  dReal getAutoDisableAngularThreshold() const
    { return dBodyGetAutoDisableAngularThreshold (get_id()); }
  void setAutoDisableSteps (int steps)
    { dBodySetAutoDisableSteps (get_id(), steps); }
  int getAutoDisableSteps() const
    { return dBodyGetAutoDisableSteps (get_id()); }
  void setAutoDisableTime (dReal time)
    { dBodySetAutoDisableTime (get_id(), time); }
  dReal getAutoDisableTime() const
    { return dBodyGetAutoDisableTime (get_id()); }
  void setAutoDisableFlag (bool do_auto_disable)
    { dBodySetAutoDisableFlag (get_id(), do_auto_disable); }
  bool getAutoDisableFlag() const
    { return dBodyGetAutoDisableFlag (get_id()) != 0; }

  dReal getLinearDamping() const
    { return dBodyGetLinearDamping(get_id()); }
  void setLinearDamping(dReal scale)
    { dBodySetLinearDamping(get_id(), scale); }
  dReal getAngularDamping() const
    { return dBodyGetAngularDamping(get_id()); }
  void setAngularDamping(dReal scale)
    { dBodySetAngularDamping(get_id(), scale); }
  void setDamping(dReal linear_scale, dReal angular_scale)
    { dBodySetDamping(get_id(), linear_scale, angular_scale); }
  dReal getLinearDampingThreshold() const
    { return dBodyGetLinearDampingThreshold(get_id()); }
  void setLinearDampingThreshold(dReal threshold) const
    { dBodySetLinearDampingThreshold(get_id(), threshold); }
  dReal getAngularDampingThreshold() const
    { return dBodyGetAngularDampingThreshold(get_id()); }
  void setAngularDampingThreshold(dReal threshold)
    { dBodySetAngularDampingThreshold(get_id(), threshold); }
  void setDampingDefaults()
    { dBodySetDampingDefaults(get_id()); }

  dReal getMaxAngularSpeed() const
    { return dBodyGetMaxAngularSpeed(get_id()); }
  void setMaxAngularSpeed(dReal max_speed)
    { dBodySetMaxAngularSpeed(get_id(), max_speed); }

  bool getGyroscopicMode() const
    { return dBodyGetGyroscopicMode(get_id()) != 0; }
  void setGyroscopicMode(bool mode)
    { dBodySetGyroscopicMode(get_id(), mode); }

};


class ODE_API dJointGroupSimpleIDContainer {
public:
  dJointGroupSimpleIDContainer(const dJointGroupSimpleIDContainer& rhs) ODE_DELETE;
  dJointGroupSimpleIDContainer& operator=(const dJointGroupSimpleIDContainer& rhs) ODE_DELETE;
	virtual ~dJointGroupSimpleIDContainer() { destroy(); }

protected:
	dJointGroupID _id;

	dJointGroupSimpleIDContainer(): _id(NULL) {}

	virtual void destroy();
};

class ODE_API dJointGroupDynamicIDContainer: public dJointGroupSimpleIDContainer {
public:
  dJointGroupDynamicIDContainer(const dJointGroupDynamicIDContainer& rhs) ODE_DELETE;
  dJointGroupDynamicIDContainer& operator=(const dJointGroupDynamicIDContainer& rhs) ODE_DELETE;
	~dJointGroupDynamicIDContainer() ODE_OVERRIDE;

protected:
  dJointGroupDynamicIDContainer() : dJointGroupSimpleIDContainer() {}
};

template <class dJointGroupTemplateBase>
class dJointGroupTemplate: public dJointGroupTemplateBase {
public:
  // intentionally undefined, don't use these
  dJointGroupTemplate (const dJointGroupTemplate<dJointGroupTemplateBase> &) ODE_DELETE;
  dJointGroupTemplate& operator= (const dJointGroupTemplate<dJointGroupTemplateBase> &) ODE_DELETE;

protected:
  virtual dJointGroupID get_id() const { return dJointGroupTemplateBase::_id; }
  virtual void set_id(dJointGroupID value) { dJointGroupTemplateBase::_id = value; }

public:
  dJointGroupTemplate ()
    { set_id(dJointGroupCreate(0)); }
  
  void create () {
    dJointGroupTemplateBase::destroy();
    set_id(dJointGroupCreate(0));
  }

  dJointGroupID id() const
    { return get_id(); }
  operator dJointGroupID() const
    { return get_id(); }

  void empty()
    { dJointGroupEmpty (get_id()); }
  void clear()
    { empty(); }
};


class ODE_API dJointSimpleIDContainer {
public:
  dJointSimpleIDContainer(const dJointSimpleIDContainer& rhs) ODE_DELETE;
  dJointSimpleIDContainer& operator=(const dJointSimpleIDContainer& rhs) ODE_DELETE;
	virtual ~dJointSimpleIDContainer() { destroy(); }

protected:
	dJointID _id;

	dJointSimpleIDContainer(): _id(NULL) {}

	virtual void destroy();
};

class ODE_API dJointDynamicIDContainer: public dJointSimpleIDContainer {
public:
  dJointDynamicIDContainer(const dJointDynamicIDContainer& rhs) ODE_DELETE;
  dJointDynamicIDContainer& operator=(const dJointDynamicIDContainer& rhs) ODE_DELETE;
	~dJointDynamicIDContainer() ODE_OVERRIDE;

protected:
  dJointDynamicIDContainer() : dJointSimpleIDContainer() {}
};

template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dJointTemplate: public dJointTemplateBase {
public:
  // intentionally undefined, don't use these
  dJointTemplate (const dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE ;
  dJointTemplate& operator= (const dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;

protected:
  virtual dJointID get_id() const { return dJointTemplateBase::_id; }
  virtual void set_id(dJointID value) { dJointTemplateBase::_id = value; }

  dJointTemplate() // don't let user construct pure dJointTemplate objects
    { }

public:
  dJointID id() const
    { return get_id(); }
  operator dJointID() const
    { return get_id(); }

  int getNumBodies() const
    { return dJointGetNumBodies(get_id()); }

  void attach (dBodyID body1, dBodyID body2)
    { dJointAttach (get_id(), body1, body2); }
  void attach (dBodyTemplate<dBodyTemplateBase, dWorldTemplateBase>& body1, dBodyTemplate<dBodyTemplateBase, dWorldTemplateBase>& body2)
    { attach(body1.id(), body2.id()); }

  void enable()
    { dJointEnable (get_id()); }
  void disable()
    { dJointDisable (get_id()); }
  bool isEnabled() const
    { return dJointIsEnabled (get_id()) != 0; }

  void setData (void *data)
    { dJointSetData (get_id(), data); }
  void *getData() const
    { return dJointGetData (get_id()); }

  dJointType getType() const
    { return dJointGetType (get_id()); }

  dBodyID getBody (int index) const
    { return dJointGetBody (get_id(), index); }

  void setFeedback(dJointFeedback *fb)
    { dJointSetFeedback(get_id(), fb); }
  dJointFeedback *getFeedback() const
    { return dJointGetFeedback(get_id()); }

  // If not implemented it will do nothing as describe in the doc
  virtual void setParam (int /**/, dReal /**/) {}
  virtual dReal getParam (int /**/) const { return 0; }
};


template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dBallJointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> {
public:
  // intentionally undefined, don't use these
  dBallJointTemplate (const dBallJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;
  dBallJointTemplate& operator= (const dBallJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;

protected:
  typedef dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> dBaseTemplate;

public:
  dBallJointTemplate() { }
  explicit dBallJointTemplate (dWorldID wrld, dJointGroupID group=NULL)
    { dBaseTemplate::set_id(dJointCreateBall(wrld, group)); }
  explicit dBallJointTemplate (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group=NULL)
    { dBaseTemplate::set_id(dJointCreateBall(wrld.id(), group)); }

  void create (dWorldID wrld, dJointGroupID group=NULL) {
    dBaseTemplate::destroy();
    dBaseTemplate::set_id(dJointCreateBall(wrld, group));
  }
  void create (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group=NULL)
    { create(wrld.id(), group); }

  void setAnchor (dReal x, dReal y, dReal z)
    { dJointSetBallAnchor (dBaseTemplate::get_id(), x, y, z); }
  void setAnchor (const dVector3 a)
    { setAnchor (a[0], a[1], a[2]); }
  void getAnchor (dVector3 result) const
    { dJointGetBallAnchor (dBaseTemplate::get_id(), result); }
  void getAnchor2 (dVector3 result) const
    { dJointGetBallAnchor2 (dBaseTemplate::get_id(), result); }
  void setParam (int parameter, dReal value) ODE_OVERRIDE
    { dJointSetBallParam (dBaseTemplate::get_id(), parameter, value); }
  dReal getParam (int parameter) const ODE_OVERRIDE
    { return dJointGetBallParam (dBaseTemplate::get_id(), parameter); }
  // TODO: expose params through methods
} ;


template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dHingeJointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> {
public:
  // intentionally undefined, don't use these
  dHingeJointTemplate (const dHingeJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;
  dHingeJointTemplate& operator = (const dHingeJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;

protected:
  typedef dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> dBaseTemplate;

public:
  dHingeJointTemplate() { }
  explicit dHingeJointTemplate (dWorldID wrld, dJointGroupID group=NULL)
    { dBaseTemplate::set_id(dJointCreateHinge(wrld, group)); }
    explicit dHingeJointTemplate (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group=NULL)
    { dBaseTemplate::set_id(dJointCreateHinge(wrld.id(), group)); }

  void create (dWorldID wrld, dJointGroupID group=NULL) {
    dBaseTemplate::destroy();
    dBaseTemplate::set_id(dJointCreateHinge (wrld, group));
  }
  void create (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group=NULL)
    { create(wrld.id(), group); }
  
  void setAnchor (dReal x, dReal y, dReal z)
    { dJointSetHingeAnchor (dBaseTemplate::get_id(), x, y, z); }
  void setAnchor (const dVector3 a)
    { setAnchor (a[0], a[1], a[2]); }
  void getAnchor (dVector3 result) const
    { dJointGetHingeAnchor (dBaseTemplate::get_id(), result); }
  void getAnchor2 (dVector3 result) const
    { dJointGetHingeAnchor2 (dBaseTemplate::get_id(), result); }

  void setAxis (dReal x, dReal y, dReal z)
    { dJointSetHingeAxis (dBaseTemplate::get_id(), x, y, z); }
  void setAxis (const dVector3 a)
    { setAxis(a[0], a[1], a[2]); }
  void getAxis (dVector3 result) const
    { dJointGetHingeAxis (dBaseTemplate::get_id(), result); }

  dReal getAngle() const
    { return dJointGetHingeAngle (dBaseTemplate::get_id()); }
  dReal getAngleRate() const
    { return dJointGetHingeAngleRate (dBaseTemplate::get_id()); }

  void setParam (int parameter, dReal value) ODE_OVERRIDE
    { dJointSetHingeParam (dBaseTemplate::get_id(), parameter, value); }
  dReal getParam (int parameter) const ODE_OVERRIDE
    { return dJointGetHingeParam (dBaseTemplate::get_id(), parameter); }
  // TODO: expose params through methods

  void addTorque (dReal torque)
	{ dJointAddHingeTorque(dBaseTemplate::get_id(), torque); }
};


template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dSliderJointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> {
public:
  // intentionally undefined, don't use these
  dSliderJointTemplate (const dSliderJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;
  dSliderJointTemplate& operator = (const dSliderJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;

protected:
  typedef dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> dBaseTemplate;

public:
  dSliderJointTemplate() { }
  explicit dSliderJointTemplate (dWorldID wrld, dJointGroupID group=NULL)
    { dBaseTemplate::set_id(dJointCreateSlider(wrld, group)); }
  explicit dSliderJointTemplate (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group=NULL)
    { dBaseTemplate::set_id(dJointCreateSlider(wrld.id(), group)); }

  void create (dWorldID wrld, dJointGroupID group=NULL) {
    dBaseTemplate::destroy();
    dBaseTemplate::set_id(dJointCreateSlider(wrld, group));
  }
  void create (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group=NULL)
    { create(wrld.id(), group); }

  void setAxis (dReal x, dReal y, dReal z)
    { dJointSetSliderAxis (dBaseTemplate::get_id(), x, y, z); }
  void setAxis (const dVector3 a)
    { setAxis (a[0], a[1], a[2]); }
  void getAxis (dVector3 result) const
    { dJointGetSliderAxis (dBaseTemplate::get_id(), result); }

  dReal getPosition() const
    { return dJointGetSliderPosition (dBaseTemplate::get_id()); }
  dReal getPositionRate() const
    { return dJointGetSliderPositionRate (dBaseTemplate::get_id()); }

  void setParam (int parameter, dReal value) ODE_OVERRIDE
    { dJointSetSliderParam (dBaseTemplate::get_id(), parameter, value); }
  dReal getParam (int parameter) const ODE_OVERRIDE
    { return dJointGetSliderParam (dBaseTemplate::get_id(), parameter); }
  // TODO: expose params through methods

  void addForce (dReal force)
	{ dJointAddSliderForce(dBaseTemplate::get_id(), force); }
};


template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dUniversalJointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> {
public:
  // intentionally undefined, don't use these
  dUniversalJointTemplate (const dUniversalJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;
  dUniversalJointTemplate& operator = (const dUniversalJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;

protected:
  typedef dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> dBaseTemplate;

public:
  dUniversalJointTemplate() { }
  explicit dUniversalJointTemplate (dWorldID wrld, dJointGroupID group=NULL)
    { dBaseTemplate::set_id(dJointCreateUniversal(wrld, group)); }
  explicit dUniversalJointTemplate (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group=NULL)
    { dBaseTemplate::set_id(dJointCreateUniversal(wrld.id(), group)); }

  void create (dWorldID wrld, dJointGroupID group=NULL) {
    dBaseTemplate::destroy();
    dBaseTemplate::set_id(dJointCreateUniversal(wrld, group));
  }
  void create (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group=NULL)
    { create(wrld.id(), group); }

  void setAnchor (dReal x, dReal y, dReal z)
    { dJointSetUniversalAnchor (dBaseTemplate::get_id(), x, y, z); }
  void setAnchor (const dVector3 a)
    { setAnchor(a[0], a[1], a[2]); }
  void setAxis1 (dReal x, dReal y, dReal z)
    { dJointSetUniversalAxis1 (dBaseTemplate::get_id(), x, y, z); }
  void setAxis1 (const dVector3 a)
    { setAxis1 (a[0], a[1], a[2]); }
  void setAxis2 (dReal x, dReal y, dReal z)
    { dJointSetUniversalAxis2 (dBaseTemplate::get_id(), x, y, z); }
  void setAxis2 (const dVector3 a)
    { setAxis2 (a[0], a[1], a[2]); }

  void getAnchor (dVector3 result) const
    { dJointGetUniversalAnchor (dBaseTemplate::get_id(), result); }
  void getAnchor2 (dVector3 result) const
    { dJointGetUniversalAnchor2 (dBaseTemplate::get_id(), result); }
  void getAxis1 (dVector3 result) const
    { dJointGetUniversalAxis1 (dBaseTemplate::get_id(), result); }
  void getAxis2 (dVector3 result) const
    { dJointGetUniversalAxis2 (dBaseTemplate::get_id(), result); }

  void setParam (int parameter, dReal value) ODE_OVERRIDE
    { dJointSetUniversalParam (dBaseTemplate::get_id(), parameter, value); }
  dReal getParam (int parameter) const ODE_OVERRIDE
    { return dJointGetUniversalParam (dBaseTemplate::get_id(), parameter); }
  // TODO: expose params through methods
  
  void getAngles(dReal *angle1, dReal *angle2) const
    { dJointGetUniversalAngles (dBaseTemplate::get_id(), angle1, angle2); }

  dReal getAngle1() const
    { return dJointGetUniversalAngle1 (dBaseTemplate::get_id()); }
  dReal getAngle1Rate() const
    { return dJointGetUniversalAngle1Rate (dBaseTemplate::get_id()); }
  dReal getAngle2() const
    { return dJointGetUniversalAngle2 (dBaseTemplate::get_id()); }
  dReal getAngle2Rate() const
    { return dJointGetUniversalAngle2Rate (dBaseTemplate::get_id()); }

  void addTorques (dReal torque1, dReal torque2)
	{ dJointAddUniversalTorques(dBaseTemplate::get_id(), torque1, torque2); }
};


template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dHinge2JointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> {
public:
  // intentionally undefined, don't use these
  dHinge2JointTemplate (const dHinge2JointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;
  dHinge2JointTemplate& operator = (const dHinge2JointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;

protected:
  typedef dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> dBaseTemplate;

public:
  dHinge2JointTemplate() { }
  explicit dHinge2JointTemplate (dWorldID wrld, dJointGroupID group=NULL)
    { dBaseTemplate::set_id(dJointCreateHinge2(wrld, group)); }
  explicit dHinge2JointTemplate (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group=NULL)
    { dBaseTemplate::set_id(dJointCreateHinge2(wrld.id(), group)); }

  void create (dWorldID wrld, dJointGroupID group=NULL) {
    dBaseTemplate::destroy();
    dBaseTemplate::set_id(dJointCreateHinge2(wrld, group));
  }
  void create (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group=NULL)
    { create(wrld.id(), group); }

  void setAnchor (dReal x, dReal y, dReal z)
    { dJointSetHinge2Anchor (dBaseTemplate::get_id(), x, y, z); }
  void setAnchor (const dVector3 a)
    { setAnchor(a[0], a[1], a[2]); }
  void setAxes (const dReal *axis1/*=NULL*/, const dReal *axis2/*=NULL*/)
    {  dJointSetHinge2Axes (dBaseTemplate::get_id(), axis1, axis2); }
  ODE_API_DEPRECATED void setAxis1 (dReal x, dReal y, dReal z)
    { dVector3 a = { x, y, z }; dJointSetHinge2Axes (dBaseTemplate::get_id(), a, NULL); }
  ODE_API_DEPRECATED void setAxis1 (const dVector3 a)
    { dJointSetHinge2Axes (dBaseTemplate::get_id(), a, NULL); }
  ODE_API_DEPRECATED void setAxis2 (dReal x, dReal y, dReal z)
    { dVector3 a = { x, y, z }; dJointSetHinge2Axes (dBaseTemplate::get_id(), NULL, a); }
  ODE_API_DEPRECATED void setAxis2 (const dVector3 a)
    { dJointSetHinge2Axes (dBaseTemplate::get_id(), NULL, a); }
    
  void getAnchor (dVector3 result) const
    { dJointGetHinge2Anchor (dBaseTemplate::get_id(), result); }
  void getAnchor2 (dVector3 result) const
    { dJointGetHinge2Anchor2 (dBaseTemplate::get_id(), result); }
  void getAxis1 (dVector3 result) const
    { dJointGetHinge2Axis1 (dBaseTemplate::get_id(), result); }
  void getAxis2 (dVector3 result) const
    { dJointGetHinge2Axis2 (dBaseTemplate::get_id(), result); }

  dReal getAngle1() const
    { return dJointGetHinge2Angle1 (dBaseTemplate::get_id()); }
  dReal getAngle1Rate() const
    { return dJointGetHinge2Angle1Rate (dBaseTemplate::get_id()); }
  dReal getAngle2Rate() const
    { return dJointGetHinge2Angle2Rate (dBaseTemplate::get_id()); }

  void setParam (int parameter, dReal value) ODE_OVERRIDE
    { dJointSetHinge2Param (dBaseTemplate::get_id(), parameter, value); }
  dReal getParam (int parameter) const ODE_OVERRIDE
    { return dJointGetHinge2Param (dBaseTemplate::get_id(), parameter); }
  // TODO: expose params through methods

  void addTorques(dReal torque1, dReal torque2)
	{ dJointAddHinge2Torques(dBaseTemplate::get_id(), torque1, torque2); }
};


template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dPRJointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> {
public:
  // intentionally undefined, don't use these
  dPRJointTemplate (const dPRJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;
  dPRJointTemplate& operator = (const dPRJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;

protected:
  typedef dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> dBaseTemplate;

public:
  dPRJointTemplate() { }
  explicit dPRJointTemplate (dWorldID wrld, dJointGroupID group=NULL)
    { dBaseTemplate::set_id(dJointCreatePR(wrld, group)); }
  explicit dPRJointTemplate (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group=NULL)
    { dBaseTemplate::set_id(dJointCreatePR(wrld.id(), group)); }

  void create (dWorldID wrld, dJointGroupID group=NULL) {
    dBaseTemplate::destroy();
    dBaseTemplate::set_id(dJointCreatePR(wrld, group));
  }
  void create (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group=NULL)
    { create(wrld.id(), group); }

  void setAnchor (dReal x, dReal y, dReal z)
    { dJointSetPRAnchor (dBaseTemplate::get_id(), x, y, z); }
  void setAnchor (const dVector3 a)
    { setAnchor (a[0], a[1], a[2]); }
  void setAxis1 (dReal x, dReal y, dReal z)
    { dJointSetPRAxis1 (dBaseTemplate::get_id(), x, y, z); }
  void setAxis1 (const dVector3 a)
    { setAxis1(a[0], a[1], a[2]); }
  void setAxis2 (dReal x, dReal y, dReal z)
    { dJointSetPRAxis2 (dBaseTemplate::get_id(), x, y, z); }
  void setAxis2 (const dVector3 a)
    { setAxis2(a[0], a[1], a[2]); }

  void getAnchor (dVector3 result) const
    { dJointGetPRAnchor (dBaseTemplate::get_id(), result); }
  void getAxis1 (dVector3 result) const
    { dJointGetPRAxis1 (dBaseTemplate::get_id(), result); }
  void getAxis2 (dVector3 result) const
    { dJointGetPRAxis2 (dBaseTemplate::get_id(), result); }

  dReal getPosition() const
    { return dJointGetPRPosition (dBaseTemplate::get_id()); }
  dReal getPositionRate() const
    { return dJointGetPRPositionRate (dBaseTemplate::get_id()); }

  dReal getAngle() const
    { return dJointGetPRAngle (dBaseTemplate::get_id()); }
  dReal getAngleRate() const
    { return dJointGetPRAngleRate (dBaseTemplate::get_id()); }

  void setParam (int parameter, dReal value) ODE_OVERRIDE
    { dJointSetPRParam (dBaseTemplate::get_id(), parameter, value); }
  dReal getParam (int parameter) const ODE_OVERRIDE
    { return dJointGetPRParam (dBaseTemplate::get_id(), parameter); }
};


template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dPUJointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase>
{
public:
  // intentionally undefined, don't use these
  dPUJointTemplate (const dPUJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;
  dPUJointTemplate& operator = (const dPUJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;

protected:
  typedef dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> dBaseTemplate;

public:
  dPUJointTemplate() { }
  explicit dPUJointTemplate (dWorldID wrld, dJointGroupID group=NULL)
    { dBaseTemplate::set_id(dJointCreatePU(wrld, group)); }
  explicit dPUJointTemplate (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group=NULL)
    { dBaseTemplate::set_id(dJointCreatePU(wrld.id(), group)); }

  void create (dWorldID wrld, dJointGroupID group=NULL)
  {
    dBaseTemplate::destroy();
    dBaseTemplate::set_id(dJointCreatePU(wrld, group));
  }
  void create (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group=NULL)
  { create(wrld.id(), group); }

  void setAnchor (dReal x, dReal y, dReal z)
    { dJointSetPUAnchor (dBaseTemplate::get_id(), x, y, z); }
  void setAnchor (const dVector3 a)
    { setAnchor (a[0], a[1], a[2]); }
  void setAxis1 (dReal x, dReal y, dReal z)
    { dJointSetPUAxis1 (dBaseTemplate::get_id(), x, y, z); }
  void setAxis1 (const dVector3 a)
    { setAxis1(a[0], a[1], a[2]); }
  void setAxis2 (dReal x, dReal y, dReal z)
  { dJointSetPUAxis2 (dBaseTemplate::get_id(), x, y, z); }
  void setAxis3 (dReal x, dReal y, dReal z)
  { dJointSetPUAxis3 (dBaseTemplate::get_id(), x, y, z); }
  void setAxis3 (const dVector3 a)
    { setAxis3(a[0], a[1], a[2]); }
  void setAxisP (dReal x, dReal y, dReal z)
  { dJointSetPUAxis3 (dBaseTemplate::get_id(), x, y, z); }
  void setAxisP (const dVector3 a)
    { setAxisP(a[0], a[1], a[2]); }

  virtual void getAnchor (dVector3 result) const
    { dJointGetPUAnchor (dBaseTemplate::get_id(), result); }
  void getAxis1 (dVector3 result) const
    { dJointGetPUAxis1 (dBaseTemplate::get_id(), result); }
  void getAxis2 (dVector3 result) const
    { dJointGetPUAxis2 (dBaseTemplate::get_id(), result); }
  void getAxis3 (dVector3 result) const
    { dJointGetPUAxis3 (dBaseTemplate::get_id(), result); }
  void getAxisP (dVector3 result) const
    { dJointGetPUAxis3 (dBaseTemplate::get_id(), result); }

  dReal getAngle1() const
    { return dJointGetPUAngle1 (dBaseTemplate::get_id()); }
  dReal getAngle1Rate() const
    { return dJointGetPUAngle1Rate (dBaseTemplate::get_id()); }
  dReal getAngle2() const
    { return dJointGetPUAngle2 (dBaseTemplate::get_id()); }
  dReal getAngle2Rate() const
    { return dJointGetPUAngle2Rate (dBaseTemplate::get_id()); }

  dReal getPosition() const
    { return dJointGetPUPosition (dBaseTemplate::get_id()); }
  dReal getPositionRate() const
    { return dJointGetPUPositionRate (dBaseTemplate::get_id()); }

  void setParam (int parameter, dReal value) ODE_OVERRIDE
  { dJointSetPUParam (dBaseTemplate::get_id(), parameter, value); }
  dReal getParam (int parameter) const ODE_OVERRIDE
    { return dJointGetPUParam (dBaseTemplate::get_id(), parameter); }
  // TODO: expose params through methods
};


template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dPistonJointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase>
{
public:
  // intentionally undefined, don't use these
  dPistonJointTemplate (const dPistonJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;
  dPistonJointTemplate& operator = (const dPistonJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;

protected:
  typedef dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> dBaseTemplate;

public:
  dPistonJointTemplate() { }
  explicit dPistonJointTemplate (dWorldID wrld, dJointGroupID group=NULL)
    { dBaseTemplate::set_id(dJointCreatePiston(wrld, group)); }
  explicit dPistonJointTemplate (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group=NULL)
    { dBaseTemplate::set_id(dJointCreatePiston(wrld, group)); }

  void create (dWorldID wrld, dJointGroupID group=NULL)
  {
    dBaseTemplate::destroy();
    dBaseTemplate::set_id(dJointCreatePiston(wrld, group));
  }
  void create (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group=NULL)
    { create(wrld.id(), group); }

  void setAnchor (dReal x, dReal y, dReal z)
    { dJointSetPistonAnchor (dBaseTemplate::get_id(), x, y, z); }
  void setAnchor (const dVector3 a)
    { setAnchor (a[0], a[1], a[2]); }
  void getAnchor (dVector3 result) const
    { dJointGetPistonAnchor (dBaseTemplate::get_id(), result); }
  void getAnchor2 (dVector3 result) const
    { dJointGetPistonAnchor2 (dBaseTemplate::get_id(), result); }

  void setAxis (dReal x, dReal y, dReal z)
    { dJointSetPistonAxis (dBaseTemplate::get_id(), x, y, z); }
  void setAxis (const dVector3 a)
    { setAxis(a[0], a[1], a[2]); }
  void getAxis (dVector3 result) const
    { dJointGetPistonAxis (dBaseTemplate::get_id(), result); }

  dReal getPosition() const
    { return dJointGetPistonPosition (dBaseTemplate::get_id()); }
  dReal getPositionRate() const
    { return dJointGetPistonPositionRate (dBaseTemplate::get_id()); }

  void setParam (int parameter, dReal value) ODE_OVERRIDE
  { dJointSetPistonParam (dBaseTemplate::get_id(), parameter, value); }
  dReal getParam (int parameter) const ODE_OVERRIDE
    { return dJointGetPistonParam (dBaseTemplate::get_id(), parameter); }
  // TODO: expose params through methods

  void addForce (dReal force)
  { dJointAddPistonForce (dBaseTemplate::get_id(), force); }
};


template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dFixedJointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase>
{
public:
  // intentionally undefined, don't use these
  dFixedJointTemplate (const dFixedJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;
  dFixedJointTemplate& operator = (const dFixedJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;

protected:
  typedef dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> dBaseTemplate;

public:
  dFixedJointTemplate() { }
  explicit dFixedJointTemplate (dWorldID wrld, dJointGroupID group=NULL)
    { dBaseTemplate::set_id(dJointCreateFixed(wrld, group)); }
  explicit dFixedJointTemplate (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group=NULL)
    { dBaseTemplate::set_id(dJointCreateFixed(wrld, group)); }

  void create (dWorldID wrld, dJointGroupID group=NULL) {
    dBaseTemplate::destroy();
    dBaseTemplate::set_id(dJointCreateFixed(wrld, group));
  }
  void create (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group=NULL)
    { create(wrld.id(), group); }

  void set()
    { dJointSetFixed (dBaseTemplate::get_id()); }

  void setParam (int parameter, dReal value) ODE_OVERRIDE
    { dJointSetFixedParam (dBaseTemplate::get_id(), parameter, value); }

  dReal getParam (int parameter) const ODE_OVERRIDE
    { return dJointGetFixedParam (dBaseTemplate::get_id(), parameter); }
  // TODO: expose params through methods
};


template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dContactJointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> {
public:
  // intentionally undefined, don't use these
  dContactJointTemplate (const dContactJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;
  dContactJointTemplate& operator = (const dContactJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;

protected:
  typedef dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> dBaseTemplate;

public:
  dContactJointTemplate() { }
  dContactJointTemplate (dWorldID wrld, dJointGroupID group, dContact *contact)
    { dBaseTemplate::set_id(dJointCreateContact(wrld, group, contact)); }
  dContactJointTemplate (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group, dContact *contact)
    { dBaseTemplate::set_id(dJointCreateContact(wrld.id(), group, contact)); }

  void create (dWorldID wrld, dJointGroupID group, dContact *contact) {
    dBaseTemplate::destroy();
    dBaseTemplate::set_id(dJointCreateContact(wrld, group, contact));
  }
  
  void create (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group, dContact *contact)
    { create(wrld.id(), group, contact); }
};


template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dNullJointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> {
public:
  // intentionally undefined, don't use these
  dNullJointTemplate (const dNullJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;
  dNullJointTemplate& operator = (const dNullJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;

protected:
  typedef dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> dBaseTemplate;

public:
  dNullJointTemplate() { }
  explicit dNullJointTemplate (dWorldID wrld, dJointGroupID group=NULL)
    { dBaseTemplate::set_id(dJointCreateNull(wrld, group)); }
  explicit dNullJointTemplate (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group=NULL)
    { dBaseTemplate::set_id(dJointCreateNull (wrld.id(), group)); }

  void create (dWorldID wrld, dJointGroupID group=NULL) {
    dBaseTemplate::destroy();
    dBaseTemplate::set_id(dJointCreateNull(wrld, group));
  }
  void create (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group=NULL)
    { create(wrld.id(), group); }
};


template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dAMotorJointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> {
public:
  // intentionally undefined, don't use these
  dAMotorJointTemplate (const dAMotorJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;
  dAMotorJointTemplate& operator = (const dAMotorJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;

protected:
  typedef dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> dBaseTemplate;

public:
  dAMotorJointTemplate() { }
  explicit dAMotorJointTemplate (dWorldID wrld, dJointGroupID group=NULL)
    { dBaseTemplate::set_id(dJointCreateAMotor(wrld, group)); }
  explicit dAMotorJointTemplate (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group=NULL)
    { dBaseTemplate::set_id(dJointCreateAMotor(wrld.id(), group)); }

  void create (dWorldID wrld, dJointGroupID group=NULL) {
    dBaseTemplate::destroy();
    dBaseTemplate::set_id(dJointCreateAMotor(wrld, group));
  }
  void create (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group=NULL)
    { create(wrld.id(), group); }

  void setMode (int mode)
    { dJointSetAMotorMode (dBaseTemplate::get_id(), mode); }
  int getMode() const
    { return dJointGetAMotorMode (dBaseTemplate::get_id()); }

  void setNumAxes (int num)
    { dJointSetAMotorNumAxes (dBaseTemplate::get_id(), num); }
  int getNumAxes() const
    { return dJointGetAMotorNumAxes (dBaseTemplate::get_id()); }

  void setAxis (int anum, int rel, dReal x, dReal y, dReal z)
    { dJointSetAMotorAxis (dBaseTemplate::get_id(), anum, rel, x, y, z); }
  void setAxis (int anum, int rel, const dVector3 a)
    { setAxis(anum, rel, a[0], a[1], a[2]); }
  void getAxis (int anum, dVector3 result) const
    { dJointGetAMotorAxis (dBaseTemplate::get_id(), anum, result); }
  int getAxisRel (int anum) const
    { return dJointGetAMotorAxisRel (dBaseTemplate::get_id(), anum); }

  void setAngle (int anum, dReal angle)
    { dJointSetAMotorAngle (dBaseTemplate::get_id(), anum, angle); }
  dReal getAngle (int anum) const
    { return dJointGetAMotorAngle (dBaseTemplate::get_id(), anum); }
  dReal getAngleRate (int anum)
    { return dJointGetAMotorAngleRate (dBaseTemplate::get_id(), anum); }

  void setParam (int parameter, dReal value) ODE_OVERRIDE
    { dJointSetAMotorParam (dBaseTemplate::get_id(), parameter, value); }
  dReal getParam (int parameter) const ODE_OVERRIDE
    { return dJointGetAMotorParam (dBaseTemplate::get_id(), parameter); }
  // TODO: expose params through methods

  void addTorques(dReal torque1, dReal torque2, dReal torque3)
	{ dJointAddAMotorTorques(dBaseTemplate::get_id(), torque1, torque2, torque3); }
};


template <class dJointTemplateBase, class dWorldTemplateBase, class dBodyTemplateBase>
class dLMotorJointTemplate : public dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> {
public:
  // intentionally undefined, don't use these
  dLMotorJointTemplate (const dLMotorJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;
  dLMotorJointTemplate& operator = (const dLMotorJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> &) ODE_DELETE;

protected:
  typedef dJointTemplate<dJointTemplateBase, dWorldTemplateBase, dBodyTemplateBase> dBaseTemplate;

public:
  dLMotorJointTemplate() { }
  explicit dLMotorJointTemplate (dWorldID wrld, dJointGroupID group=NULL)
    { dBaseTemplate::set_id(dJointCreateLMotor(wrld, group)); }
  explicit dLMotorJointTemplate (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group=NULL)
    { dBaseTemplate::set_id(dJointCreateLMotor(wrld.id(), group)); }

  void create (dWorldID wrld, dJointGroupID group=NULL) {
    dBaseTemplate::destroy();
    dBaseTemplate::set_id(dJointCreateLMotor(wrld, group));
  }
  void create (dWorldTemplate<dWorldTemplateBase>& wrld, dJointGroupID group=NULL)
    { create(wrld.id(), group); }

  void setNumAxes (int num)
    { dJointSetLMotorNumAxes (dBaseTemplate::get_id(), num); }
  int getNumAxes() const
    { return dJointGetLMotorNumAxes (dBaseTemplate::get_id()); }

  void setAxis (int anum, int rel, dReal x, dReal y, dReal z)
    { dJointSetLMotorAxis (dBaseTemplate::get_id(), anum, rel, x, y, z); }
  void setAxis (int anum, int rel, const dVector3 a)
    { setAxis(anum, rel, a[0], a[1], a[2]); }
  void getAxis (int anum, dVector3 result) const
    { dJointGetLMotorAxis (dBaseTemplate::get_id(), anum, result); }

  void setParam (int parameter, dReal value) ODE_OVERRIDE
    { dJointSetLMotorParam (dBaseTemplate::get_id(), parameter, value); }
  dReal getParam (int parameter) const ODE_OVERRIDE
    { return dJointGetLMotorParam (dBaseTemplate::get_id(), parameter); }
  // TODO: expose params through methods
};

//}

#if !defined(dODECPP_WORLD_TEMPLATE_BASE)

#if defined(dODECPP_BODY_TEMPLATE_BASE) || defined(dODECPP_JOINTGROUP_TEMPLATE_BASE) || defined(dODECPP_JOINT_TEMPLATE_BASE)
#error All the odecpp template bases must be defined or not defined together
#endif

#define dODECPP_WORLD_TEMPLATE_BASE dWorldDynamicIDContainer
#define dODECPP_BODY_TEMPLATE_BASE dBodyDynamicIDContainer
#define dODECPP_JOINTGROUP_TEMPLATE_BASE dJointGroupDynamicIDContainer
#define dODECPP_JOINT_TEMPLATE_BASE dJointDynamicIDContainer

#else // #if defined(dODECPP_WORLD_TEMPLATE_BASE)

#if !defined(dODECPP_BODY_TEMPLATE_BASE) || !defined(dODECPP_JOINTGROUP_TEMPLATE_BASE) || !defined(dODECPP_JOINT_TEMPLATE_BASE)
#error All the odecpp template bases must be defined or not defined together
#endif

#endif // #if defined(dODECPP_WORLD_TEMPLATE_BASE)


typedef dWorldTemplate<dODECPP_WORLD_TEMPLATE_BASE> dWorld;
typedef dBodyTemplate<dODECPP_BODY_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE> dBody;
typedef dJointGroupTemplate<dODECPP_JOINTGROUP_TEMPLATE_BASE> dJointGroup;
typedef dJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dJoint;
typedef dBallJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dBallJoint;
typedef dHingeJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dHingeJoint;
typedef dSliderJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dSliderJoint;
typedef dUniversalJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dUniversalJoint;
typedef dHinge2JointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dHinge2Joint;
typedef dPRJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dPRJoint;
typedef dPUJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dPUJoint;
typedef dPistonJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dPistonJoint;
typedef dFixedJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dFixedJoint;
typedef dContactJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dContactJoint;
typedef dNullJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dNullJoint;
typedef dAMotorJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dAMotorJoint;
typedef dLMotorJointTemplate<dODECPP_JOINT_TEMPLATE_BASE, dODECPP_WORLD_TEMPLATE_BASE, dODECPP_BODY_TEMPLATE_BASE> dLMotorJoint;


#endif // __cplusplus
#endif // ODE_ODECPP_H

// Local variables:
// mode:c++
// c-basic-offset:2
// End:
