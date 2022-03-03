// ----------------------------------------------------------------------------
// RigidSolver.hpp
//
//  Created on: 18 Dec 2020
//      Author: Kiwon Um
//        Mail: kiwon.um@telecom-paris.fr
//
// Description: Simple Rigid Body Solver (DO NOT DISTRIBUTE!)
//
// Copyright 2020-2022 Kiwon Um
//
// The copyright to the computer program(s) herein is the property of Kiwon Um,
// Telecom Paris, France. The program(s) may be used and/or copied only with
// the written permission of Kiwon Um or in accordance with the terms and
// conditions stipulated in the agreement/contract under which the program(s)
// have been supplied.
// ----------------------------------------------------------------------------

#ifndef _RIGIDSOLVER_HPP_
#define _RIGIDSOLVER_HPP_

#include <glm/ext/matrix_transform.hpp>
#include <iostream>

#include "Vector3.hpp"
#include "Matrix3x3.hpp"
#include "Quaternion.hpp"


struct BodyAttributes {
  BodyAttributes() :
    X(0, 0, 0), R(Mat3f::I()), P(0, 0, 0), L(0, 0, 0),
    V(0, 0, 0), omega(0, 0, 0), F(0, 0, 0), tau(0, 0, 0), Q(0,1,0,0){}

  glm::mat4 worldMat() const
  {
    return glm::mat4(           // column-major
      R(0,0), R(1,0), R(2,0), 0,
      R(0,1), R(1,1), R(2,1), 0,
      R(0,2), R(1,2), R(2,2), 0,
      X[0],   X[1],   X[2],   1);
  }

  tReal M;                      // mass
  Mat3f I0, I0inv;              // inertia tensor and its inverse in body space
  Mat3f Iinv;                   // inverse of inertia tensor

  // rigid body state
  Vec3f X;                      // position
  Mat3f R;                      // rotation
  Quaternion Q;                 // rotation computation alias
  Vec3f P;                      // linear momentum
  Vec3f L;                      // angular momentum

  // auxiliary quantities
  Vec3f V;                      // linear velocity
  Vec3f omega;                  // angular velocity

  // force and torque
  Vec3f F;                      // force
  Vec3f tau;                    // torque

  // mesh's vertices in body space
  std::vector<Vec3f> vdata0;
};

class Box : public BodyAttributes {
public:
  explicit Box(
    const tReal w=1.0, const tReal h=1.0, const tReal d=1.0, const tReal dens=10.0,
    const Vec3f v0=Vec3f(0, 0, 0), const Vec3f omega0=Vec3f(0, 0, 0)) :
    width(w), height(h), depth(d)
  {
    V = v0;                     // initial velocity
    omega = omega0;             // initial angular velocity

    // TODO: calculate physical attributes
    M = dens * w * h * d;
    I0 = (1/12) * M * Mat3f(
                          h * h + d * d, 0, 0,
                          0, w * w + d * d, 0,
                          0, 0, w * w + h * h
                          );
    I0inv = (12 / M) * Mat3f(
                          1/(h * h + d * d), 0, 0,
                          0, 1/(w * w + d * d), 0,
                          0, 0, 1/(w * w + h * h)
                          );
    Iinv = R * I0inv.mulTranspose(R);

    // vertices data (8 vertices)
    vdata0.push_back(Vec3f(-0.5*w, -0.5*h, -0.5*d));
    vdata0.push_back(Vec3f( 0.5*w, -0.5*h, -0.5*d));
    vdata0.push_back(Vec3f( 0.5*w,  0.5*h, -0.5*d));
    vdata0.push_back(Vec3f(-0.5*w,  0.5*h, -0.5*d));

    vdata0.push_back(Vec3f(-0.5*w, -0.5*h,  0.5*d));
    vdata0.push_back(Vec3f( 0.5*w, -0.5*h,  0.5*d));
    vdata0.push_back(Vec3f( 0.5*w,  0.5*h,  0.5*d));
    vdata0.push_back(Vec3f(-0.5*w,  0.5*h,  0.5*d));
  }

  // rigid body property
  tReal width, height, depth;
};




class RigidSolver {
public:
  explicit RigidSolver(
    BodyAttributes *body0=nullptr, const Vec3f g=Vec3f(0, 0, 0)) :
    body(body0), _g(g), _step(0), _sim_t(0) {}

  void init(BodyAttributes *body0)
  {
    body = body0;
    _step = 0;
    _sim_t = 0;
  }

  void step(const tReal dt)
  {
    std::cout << "t=" << _sim_t << " (dt=" << dt << ")" << std::endl;

    //second time derivatives
    computeForceAndTorque();

    
    // first time derivatives
    body->P += dt * body->F;            // Linear momentum
    body->L += dt * body->tau;          // Rotation momentum

    body->V = body->P/body->M;          // Linear velocity
    body->omega = body->Iinv * body->L; // Angular velocity

    //zero time derivatives
    body->X += dt * body->V;
    
    body->Q = body->Q + Quaternion( 0, body->omega.x, body->omega.y, body->omega.z) * body->Q * dt;
    body->Q.normalize();
    body->R = body->Q.toRotationMatrix();

    printf("body->Q = %s, body->tau length = %f, body->R length = %f\n", body->Q.toString().c_str(), body->tau.length(), body->R.sumSqr());
    ++_step;
    _sim_t += dt;
  }

  BodyAttributes *body;

private:
  void computeForceAndTorque()
  {
    body->F = _g;
    body->tau = body->tau * .5;
    // TODO: instance force at the very first step
    if(_step==1) {
        body->F = Vec3f(1.0, 7, 2.4)/5.0;
        // body->F += Vec3f(.15, .25, .03);

        body->tau = Vec3f(.001, .001, 0.0);
    }
  }

  // simulation parameters
  Vec3f _g;                     // gravity
  tIndex _step;                 // step count
  tReal _sim_t;                 // simulation time
};

#endif  /* _RIGIDSOLVER_HPP_ */
