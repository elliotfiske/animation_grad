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
#include "mosek_man.h"

using namespace Eigen;
using namespace std;

#define SEGMENT_WIDTH 1.0f

/* This function prints log output from MOSEK to the terminal. */
static void MSKAPI printstr(void *handle,
                            MSKCONST char str[])
{
//   printf("%s",str);
} /* printstr */

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
   Vector3d w( 1.0, 0, 2.0);
   Vector3d v(-1, 2.0, 0);
   
   curr_phi.resize(6);
   curr_phi << w(0), w(1), w(2), v(0), v(1), v(2);
}

Link::~Link()
{
}

void Link::check_corner(double x_offset, double y_offset, double z_offset) {
   // The vector (x, y, z) is in model-space, I want to convert it to
   //  world space
   Vector3d xi;
   xi << x_offset, y_offset, z_offset;
   
   Vector4d xw;
   xw << x_offset, y_offset, z_offset, 1.0;
   xw = curr_E * xw;
   
   //    printf("x_world... %f %f %f\n", xi(0), xi(1), xi(2));
   
   
   // Check if the point is below the water level
   if (xw(1) < -2.0) {
      printf("Hey it's colliding");
      Contact c;
      c.xw = xw;
      c.nw = Vector3d(0.0, 1.0, 0.0);
      
      MatrixXd J = curr_E.block<3,3>(0,0) * gamma(xi); // Jacobian
      Vector3d world_vel = J * curr_phi;
      
      double thresh = 1e-6;
      Vector3d tangent0 = c.nw.cross(world_vel);
      if (tangent0.norm() < thresh) { // IDK what this means. Should ask.
         Vector3d tmp;
         if (abs(c.nw(2)) < thresh) {
            tmp << 0.0, 1.0, 0.0;
         }
         else {
            tmp << 1.0, 0.0, 0.0;
         }
         tangent0 = c.nw.cross(tmp);
      }
      tangent0.normalize();
      Vector3d tangent1 = c.nw.cross(tangent0);
      c.tangent0w = tangent0;
      c.tangent1w = tangent1;
   }
}

void Link::do_collision() {
   contacts.clear();
   
   // Check each corner of me for a collision
   check_corner(-1, -1, -1);
   check_corner(-1, -1,  1);
   check_corner(-1,  1, -1);
   check_corner(-1,  1,  1);
   check_corner( 1, -1, -1);
   check_corner( 1, -1,  1);
   check_corner( 1,  1, -1);
   check_corner( 1,  1,  1);
}

void got_error(MSKrescodee r) {
   /* In case of an error print error code and description. */
   char symname[MSK_MAX_STR_LEN];
   char desc[MSK_MAX_STR_LEN];
   
   printf("An error occurred while optimizing.\n");
   MSK_getcodedesc (r,
                    symname,
                    desc);
   printf("Error %s - '%s'\n",symname,desc);
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
   
   Vector4d world_gravity(0, -2.0, 0, 0);
   Vector4d local_gravity = curr_E.transpose() * world_gravity;
   
   Vector6d fg = VectorXd::Zero(6);
   fg.segment(3, 3) = local_gravity.segment(0, 3);
   
   do_collision();
   
   bool collisions = true;
   int num_collisions = 0;
   int num_vars = 6 * 1; // TODO: this 1 becomes more if we have more rigidbodies
   
   // Solve Ax = b where A = Mi
   // and b is this huge thing from the worksheet
   Vector6d b = Mi * curr_phi +
   h * (bracket_phi.transpose() * Mi*curr_phi + fg);
   
   if (collisions) {
      // Collisions present! Let's ask our friend Mosek for help solving this one.
      MSKenv_t env = NULL;
      MSKrescodee r = MSK_makeenv(&env, NULL);
      MSKtask_t task = NULL;
      
      if (r != MSK_RES_OK) { printf("Wow, a problem already :3\n"); got_error(r); }
      
      r = MSK_maketask(env, num_collisions, num_vars, &task);
      if (r != MSK_RES_OK) { printf("ERror making task\n"); got_error(r); }
      
      r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);
      
      if (r != MSK_RES_OK) { printf("How did you get an error doing that??\n"); got_error(r); }
      
      // Stick the constraints in, from the list of collisions we got from back up there.
      r = MSK_appendcons(task, num_collisions);
      if (r != MSK_RES_OK) { printf("Error appending constraints\n"); got_error(r); }
      
      // Append 6 * number of rigidbodies to the equation. This is one variable for each
      //  component in phi, per rigidbody.
      r = MSK_appendvars(task, 6);
      if (r != MSK_RES_OK) { printf("Error appending variables\n"); got_error(r); }
      
      for (int j = 0; j < num_vars; j++) {
         
         r = MSK_putcj(task, j, -b[j]); // TODO: idk whats supposed to go here
         if (r != MSK_RES_OK) { printf("Error adding linear term\n"); got_error(r); }
         
         r = MSK_putvarbound(task, j, MSK_BK_FR, -MSK_INFINITY, +MSK_INFINITY);
         if (r != MSK_RES_OK) { printf("Error adding variable bound\n"); got_error(r); }
      }
      
      
      // TODO: calculate N, put it here in the A-variable's spot.
      //r = MSK_putacol(task, j, <#MSKint32t nzj#>, <#const MSKint32t *subj#>, <#const MSKrealt *valj#>)
      
      
      
      // input  mass matrix
      MSKint32t qsubi[6];
      MSKint32t qsubj[6];
      double    qval[6];
      
      for (int ndx = 0; ndx < 3; ndx++) {
         qsubi[ndx] = ndx;
         qsubj[ndx] = ndx;
         qval[ndx] = m * s*s / 6;
      }
      
      for (int ndx = 3; ndx < 6; ndx++) {
         qsubi[ndx] = ndx;
         qsubj[ndx] = ndx;
         qval[ndx] = m;
      }
      
      r = MSK_putqobj(task, 6, qsubi, qsubj, qval);
      if (r != MSK_RES_OK) { printf("Error inputting Mass Matrix\n"); got_error(r); }
      
      MSKrescodee trmcode;
      r  = MSK_optimizetrm(task, &trmcode);
      if (r != MSK_RES_OK) { printf("Error finding solution\n"); got_error(r); }
      
      MSKsolstae solsta;
      MSK_getsolsta(task, MSK_SOL_ITR, &solsta);
      
      vector<double> result;
      result.resize(num_vars);
      
      switch(solsta) {
         case MSK_SOL_STA_OPTIMAL:
         case MSK_SOL_STA_NEAR_OPTIMAL:
            MSK_getxx(task, MSK_SOL_ITR, &result[0]);
            break;
            
         default:
            printf("Got some weird solution status: %d\n", solsta);
            break;
      }
      
      for (int ndx = 0; ndx < curr_phi.rows(); ndx++) {
         curr_phi[ndx] = result[ndx];
      }
      
      MSK_deletetask(&task);
      MSK_deleteenv(&env);
   }
   else {
      curr_phi = Mi.ldlt().solve(b);
   }
   
   
   
//   printf("%f, %f, %f, %f, %f, %f\n", curr_phi(0), curr_phi(1), curr_phi(2), curr_phi(3), curr_phi(4), curr_phi(5));

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
   
   M->multMatrix(curr_E.cast<float>());
   glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, M->topMatrix().data());
   shape->draw(prog);
   
   M->popMatrix();
}