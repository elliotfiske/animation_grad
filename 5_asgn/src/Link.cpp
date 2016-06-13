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
#include <iostream>

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
   printf("%s",str);
} /* printstr */

Link::Link(double w, double h, double d, double pos_x, double pos_y, double pos_z, double m, int i)
: Rigid()
{
   body_ndx = i;
   
   angle = 0.0;
   position << pos_x, pos_y, pos_z;
   
   width = w;
   height = h;
   depth = d;
   mass = m;
   
   explosion_force = Vector3d::Zero();
   
   missile = false;
   manual_velocity = Vector3d::Zero();
   
   // Set the E transform to the starting position and rotation
   curr_E = Matrix4d::Identity();
   Matrix3d rot_matrix = AngleAxisd(angle, Vector3d::UnitZ()).matrix();
   curr_E.block<3,3>(0,0) = rot_matrix;
   curr_E.block<3,1>(0,3) = position;
   
   // Start angular velocity as 0, 0, 2.0
   // and positional velocity as -1, 0, 0
   Vector3d ang(0.0, 0.0, 0.0);
   Vector3d v(0.0, 0.0, 0.0);
   
   curr_phi.resize(6);
   curr_phi << ang(0), ang(1), ang(2), v(0), v(1), v(2);

   M_mass = MatrixXd::Zero(6, 6);
   
   double m_12 = mass / 12.0;
   
   M_mass(0, 0) = m_12 * (width*width + depth*depth);
   M_mass(1, 1) = m_12 * (depth*depth + height*height);
   M_mass(2, 2) = m_12 * (width*width + height*height);
   M_mass(3, 3) = mass;
   M_mass(4, 4) = mass;
   M_mass(5, 5) = mass;
}

Link::~Link()
{
}

Vector6d Link::get_curr_f() {
   Vector6d result = Vector6d::Zero();
   
   Vector3d world_gravity(0.0, -2.0, 0.0);
   Vector3d local_gravity = curr_E.block<3,3>(0,0).transpose() * world_gravity;
   
   result.segment(3, 3) = local_gravity;
   
   Vector6d coriolis = M_mass * curr_phi;
   coriolis.segment<3>(0) = curr_phi.segment<3>(0).cross(coriolis.segment<3>(0));
   coriolis.segment<3>(3) = curr_phi.segment<3>(0).cross(coriolis.segment<3>(3));
   
   result -= coriolis;
   
   result.tail(3) += explosion_force;
   explosion_force = Vector3d::Zero();
   
   if (abs(result(4)) > 1000) {
      printf( "hmm");
   }
   
//   printf("FORCE: \n");
//   
//   const IOFormat fmt(2, DontAlignCols, "\t", " ", "", "", "", "");
//   cout << result.format(fmt) << endl;
   
   return result;
}

void Link::check_corner(double x_offset, double y_offset, double z_offset, vector<Contact>* all_contacts) {
   // The vector (x, y, z) is in model-space, I want to convert it to
   //  world space
   Vector3d xi;
   xi << x_offset*width/2, y_offset*height/2, z_offset*depth/2;
   
   Vector4d xw;
   xw << x_offset*width/2, y_offset*height/2, z_offset*depth/2, 1.0;
   xw = curr_E * xw;
   
   //    printf("x_world... %f %f %f\n", xi(0), xi(1), xi(2));
   
   
   // Check if the point is below the water level
   if (xw(1) < -0.0) {
      Contact c;
      c.rigid_body_other_ndx = -1; // There is no other body, just the ground
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
      
      c.N_component = c.nw.transpose() * J;
      c.rigid_body_ndx = body_ndx;
      
      all_contacts->push_back(c);
   }
}

void Link::do_collision(std::vector<Contact>* scene_contacts) {
   contacts.clear();
   
   // Check each corner of me for a collision
   check_corner(-1, -1, -1, scene_contacts);
   check_corner(-1, -1,  1, scene_contacts);
   check_corner(-1,  1, -1, scene_contacts);
   check_corner(-1,  1,  1, scene_contacts);
   check_corner( 1, -1, -1, scene_contacts);
   check_corner( 1, -1,  1, scene_contacts);
   check_corner( 1,  1, -1, scene_contacts);
   check_corner( 1,  1,  1, scene_contacts);
}

void got_error(MSKrescodee r) {
   /* In case of an error print error code and description. */
   char symname[MSK_MAX_STR_LEN];
   char desc[MSK_MAX_STR_LEN];
   
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
   
   VectorXd coriolis = Mi * curr_phi;
   coriolis.segment<3>(0) = curr_phi.segment<3>(0).cross(coriolis.segment<3>(0));
   coriolis.segment<3>(3) = curr_phi.segment<3>(0).cross(coriolis.segment<3>(3));
   
   fg -= coriolis;
   
//   do_collision();
   
   int num_vars = 6 * 1; // TODO: this 1 becomes more if we have more rigidbodies
   
   // Solve Ax = b where A = Mi
   // and b is this huge thing from the worksheet
   Vector6d b = Mi * curr_phi +
      h * (bracket_phi.transpose() * Mi*curr_phi + fg);
   
   printf("B here is: %f %f %f %f %f %f\n", b(0), b(1), b(2), b(3), b(4), b(5));
   
   if (contacts.size() > 0) {
      
      // Collisions present! Let's ask our friend Mosek for help solving this one.
      MSKenv_t env = NULL;
      MSKrescodee r = MSK_makeenv(&env, NULL);
      MSKtask_t task = NULL;
      int num_constraints = contacts.size();
      
      if (r != MSK_RES_OK) { printf("Wow, a problem already :3\n"); got_error(r); }
      
      r = MSK_maketask(env, num_constraints, num_vars, &task);
      if (r != MSK_RES_OK) { printf("ERror making task\n"); got_error(r); }
      
      r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);
      
      if (r != MSK_RES_OK) { printf("How did you get an error doing that??\n"); got_error(r); }
      
      // Stick the constraints in, from the list of collisions we got from back up there.
      r = MSK_appendcons(task, num_constraints);
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
      
      
      // we need the "indexes of the non-zero elements in the row", but since everything has non-zero,
      //  we'll just make an array with 1->6, for goofs and gaffs.
      vector<MSKint32t> row_ndxs;
      for (int ndx = 0; ndx < 6 * 1; ndx++) {
         row_ndxs.push_back(ndx);
      }
      
//      printf("Contact: %f %f %f %f %f %f\n", contacts[0].N_component[0], contacts[0].N_component[1], contacts[0].N_component[2], contacts[0].N_component[3], contacts[0].N_component[4], contacts[0].N_component[5]);
      
      // Insert the rows of the constraint
      for (int contact_ndx = 0; contact_ndx < contacts.size(); contact_ndx++) {
         for (int n_ndx = 0; n_ndx < 6; n_ndx++) {
            r = MSK_putaij(task, contact_ndx, n_ndx, contacts[contact_ndx].N_component[n_ndx]);
            if (r != MSK_RES_OK) {
               printf("Error setting constraint\n"); got_error(r);
            }
         }
         
         r = MSK_putconbound(task, contact_ndx, MSK_BK_LO, 0, +MSK_INFINITY);
         if (r != MSK_RES_OK) { printf("Error setting contraint bounds\n"); got_error(r); }
      }
      
       
      
      // input  mass matrix
      vector<MSKint32t> qsubi;
      qsubi.resize(6);
      vector<MSKint32t> qsubj;
      qsubj.resize(6);
      vector<double>    qval;
      qval.resize(6);
      
      // Building mass matrix by hand here :/
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
      
      r = MSK_putqobj(task, 6, &qsubi[0], &qsubj[0], &qval[0]);
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

void Link::update_pos(double h) {
   for (int i = 0; i < 6; i++) {
      curr_phi(i) = clip(curr_phi(i), -10.0, 10.0);
   }
   Matrix4d next_E = integrate(curr_E, curr_phi, h);
   curr_E = next_E;
}

// Push this object's matrices onto the stack and draw it
void Link::draw(MatrixStack *M, const std::shared_ptr<Program> prog, const std::shared_ptr<Shape> shape) {
   M->pushMatrix();
   
   position += curr_phi.segment(3, 3) * 0.1;
   
   angle += curr_phi(2) * 0.1;
   
   M->multMatrix(curr_E.cast<float>());
   M->scale(Vector3f(width/2, height/2, depth/2));
   
   glUniformMatrix4fv(prog->getUniform("MV"), 1, GL_FALSE, M->topMatrix().data());
   shape->draw(prog);
   
   M->popMatrix();
}