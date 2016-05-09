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
   position << 0.0f, 0.0f, 0.0f;
}

Link::~Link()
{
}

// Push this object's matrices onto the stack and draw it
void Link::draw(MatrixStack *M, const std::shared_ptr<Program> prog, const std::shared_ptr<Shape> shape) {
   Affine3f transformation_to_world(Translation3f(position(0), position(1), position(2)));
   Matrix4f i_to_world_E = transformation_to_world.matrix();
   Matrix3f rot_matrix = AngleAxisf(angle, Vector3f::UnitZ()).matrix();
   i_to_world_E.block<3,3>(0,0) = rot_matrix;
   
   M->pushMatrix();
   
   M->multMatrix(i_to_world_E);
   glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, M->topMatrix().data());
   shape->draw(prog);
   
   M->popMatrix();
}