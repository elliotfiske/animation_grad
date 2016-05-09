//
//  Link.cpp
//  Lab07
//
//  Created by Elliot Fiske on 2/21/16.
//
//

#include "Link.hpp"

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <Eigen/Geometry>

#include "GLSL.h"
#include "Program.h"

using namespace Eigen;
using namespace std;

#define SEGMENT_WIDTH 1.0f

Link::Link() 
: Rigid()
{
   angle = 0;
   position << 1.0f, 0.0f, 0.0f;
   
   // Set the E transform to the starting position and rotation
   curr_E = Matrix4d::Identity();
   Matrix3d rot_matrix = AngleAxisd(angle, Vector3d::UnitZ()).matrix();
   curr_E.block<3,3>(0,0) = rot_matrix;
   curr_E.block<3,1>(0,3) = position;
   
   
   // Start angular velocity as 0, 0, 2.0
   // and positional velocity as -1, 0, 0
   Vector3d w( 0, 0, 2.0);
   Vector3d v(-1, 0, 0);
   
   curr_phi.resize(6);
   curr_phi << w(0), w(1), w(2), v(0), v(1), v(2);
}

Link::~Link()
{
}

// Move a link via its angular and positional velocity
void Link::step(double h) {
   // Mass of the rigidbody
   double m = 1.0;
   // length of the sides of the cube
   double s = 0.5;
   
   Matrix6d bracket_phi = Matrix6d::Zero(6, 6);
   bracket_phi.block<3,3>(0,0) = bracket3(curr_phi.segment(0, 3));
   bracket_phi.block<3,3>(3,3) = bracket3(curr_phi.segment(0, 3));
   
   Matrix6d Mi = MatrixXd::Zero(6, 6);
   Mi(0, 0) = m * s*s / 6;
   Mi(1, 1) = m * s*s / 6;
   Mi(2, 2) = m * s*s / 6;
   Mi(3, 3) = m;
   Mi(4, 4) = m;
   Mi(5, 5) = m;
   
   Vector6d fg;
   fg << 0, 0, 0, 0.0, -2.0, 0.0;
   
   // Solve Ax = b where A = Mi
   // and b is this huge thing from the worksheet
   Vector6d b = Mi * curr_phi +
                 h * (bracket_phi.transpose() * Mi*curr_phi + fg);
   
   curr_phi = Mi.ldlt().solve(b);
   
   printf("%f, %f, %f, %f, %f, %f\n", curr_phi(0), curr_phi(1), curr_phi(2), curr_phi(3), curr_phi(4), curr_phi(5));

   Matrix4d next_E = integrate(curr_E, curr_phi, h);
   curr_E = next_E;
}

// Push this object's matrices onto the stack and draw it
void Link::draw(MatrixStack *M, const std::shared_ptr<Program> prog, const std::shared_ptr<Shape> shape) {
//   Affine3f transformation_to_world(Translation3f((float) position(0), (float) position(1), (float) position(2)));
//   Matrix4f i_to_world_E = transformation_to_world.matrix();
   
   M->pushMatrix();
   
   position += curr_phi.segment(3, 3) * 0.1;
   
   angle += curr_phi(2) * 0.1;
   
//   Matrix3d rot_matrix = AngleAxisd(angle, Vector3d::UnitZ()).matrix();
//   curr_E.block<3,3>(0,0) = rot_matrix;
//   
//   M->translate(position.cast<float>());
   
   M->multMatrix(curr_E.cast<float>());
   glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, M->topMatrix().data());
   shape->draw(prog);
   
   M->popMatrix();
}