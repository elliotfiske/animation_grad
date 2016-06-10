//
//  Scene.cpp
//  Lab07
//
//  Created by Elliot Fiske on 6/9/16.
//
//

#include "Scene.hpp"
#include "mosek_man.h"

using namespace std;
using namespace Eigen;

/* This function prints log output from MOSEK to the terminal. */
static void MSKAPI printstr_debug(void *handle,
                            MSKCONST char str[])
{
//   printf("%s",str);
} /* printstr */

void got_mad_error(MSKrescodee r) {
   /* In case of an error print error code and description. */
   char symname[MSK_MAX_STR_LEN];
   char desc[MSK_MAX_STR_LEN];
   
   MSK_getcodedesc (r,
                    symname,
                    desc);
   printf("Error %s - '%s'\n",symname,desc);
}

Scene::Scene() {
}

void Scene::make_links() {

   
   Link baby_link(1.0, 1.0, 1.0, 0.0, 2.0, 0.0, 1.0);
   bodies.push_back(baby_link);
   
   M_accum = MatrixXd::Zero(bodies.size() * 6, bodies.size() * 6);
//   phi_accum.resize(bodies.size() * 6);
//   f_accum.resize(bodies.size() * 6);
   
   for (int i_ndx = 0; i_ndx < bodies.size() * 6; i_ndx++) {
      phi_accum(i_ndx) = 0;
   }
   
   for (int ndx = 0; ndx < bodies.size(); ndx++) {
      M_accum.block<6, 6>(ndx * 6, ndx*6) = bodies[ndx].M_mass;
   }
   
}

// Move all the rigid bodies
void Scene::step_all(double h) {
   
   for (int ndx = 0; ndx < bodies.size(); ndx++) {
      phi_accum.segment<6>(ndx * 6) = bodies[ndx].curr_phi;
//      bodies[ndx].step(h);
      
      f_accum.segment<6>(ndx * 6) = bodies[ndx].get_curr_f();
   }
   
   // TODO: the collisions, plz
 
   
   int num_vars = 6 * bodies.size();
   
   // Solve Ax = b where A = Mi
   // and b is this huge thing from the worksheet // TODO: nopes
   Eigen::Matrix<double, 6, 1> b = M_accum * phi_accum +
                                   h * f_accum;
   
   printf("B is: %f %f %f %f %f %f\n", b(0), b(1), b(2), b(3), b(4), b(5));
   
   if (isnan(b(5)) || isnan(b(0)) || isnan(b(2)) || isnan(b(1)) || isnan(b(3)) || isnan(b(4))) {
      printf("ah snap\n");
   }
   
//   bodies[0].step(h);
   
   vector<double> result;
   
   if (true) {
      
      // Collisions present! Let's ask our friend Mosek for help solving this one.
      MSKenv_t env = NULL;
      MSKrescodee r = MSK_makeenv(&env, NULL);
      MSKtask_t task = NULL;
      int num_constraints = contacts.size();
      
      if (r != MSK_RES_OK) { printf("Wow, a problem already :3\n"); got_mad_error(r); }
      
      r = MSK_maketask(env, num_constraints, num_vars, &task);
      if (r != MSK_RES_OK) { printf("ERror making task\n"); got_mad_error(r); }
      
      r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr_debug);
      
      if (r != MSK_RES_OK) { printf("How did you get an error doing that??\n"); got_mad_error(r); }
      
      // Stick the constraints in, from the list of collisions we got from back up there.
      r = MSK_appendcons(task, num_constraints);
      if (r != MSK_RES_OK) { printf("Error appending constraints\n"); got_mad_error(r); }
      
      // Append 6 * number of rigidbodies to the equation. This is one variable for each
      //  component in phi, per rigidbody.
      r = MSK_appendvars(task, 6 * bodies.size());
      if (r != MSK_RES_OK) { printf("Error appending variables\n"); got_mad_error(r); }
      
      for (int j = 0; j < num_vars; j++) {
         
         r = MSK_putcj(task, j, -b[j]); // TODO: idk whats supposed to go here
         if (r != MSK_RES_OK) { printf("Error adding linear term\n"); got_mad_error(r); }
         
         r = MSK_putvarbound(task, j, MSK_BK_FR, -MSK_INFINITY, +MSK_INFINITY);
         if (r != MSK_RES_OK) { printf("Error adding variable bound\n"); got_mad_error(r); }
      }
      
      //      printf("Contact: %f %f %f %f %f %f\n", contacts[0].N_component[0], contacts[0].N_component[1], contacts[0].N_component[2], contacts[0].N_component[3], contacts[0].N_component[4], contacts[0].N_component[5]);
      
      // Insert the rows of the constraint
      for (int contact_ndx = 0; contact_ndx < contacts.size(); contact_ndx++) {
         for (int n_ndx = 0; n_ndx < 6; n_ndx++) {
            r = MSK_putaij(task, contact_ndx, n_ndx, contacts[contact_ndx].N_component[n_ndx]);
            if (r != MSK_RES_OK) {
               printf("Error setting constraint\n"); got_mad_error(r);
            }
         }
         
         r = MSK_putconbound(task, contact_ndx, MSK_BK_LO, 0, +MSK_INFINITY);
         if (r != MSK_RES_OK) { printf("Error setting contraint bounds\n"); got_mad_error(r); }
      }
      
      
      
      // input  mass matrix
      vector<MSKint32t> qsubi;
      qsubi.resize(6 * bodies.size());
      vector<MSKint32t> qsubj;
      qsubj.resize(6 * bodies.size());
      vector<double>    qval;
      qval.resize(6 * bodies.size());
      
      // Building mass matrix by hand here :/
      for (int ndx = 0; ndx < M_accum.rows(); ndx++) {
         qsubi[ndx] = ndx;
         qsubj[ndx] = ndx;
         qval[ndx] = M_accum(ndx, ndx);
      }
      
      r = MSK_putqobj(task, 6 * bodies.size(), &qsubi[0], &qsubj[0], &qval[0]);
      if (r != MSK_RES_OK) { printf("Error inputting Mass Matrix\n"); got_mad_error(r); }
      
      MSKrescodee trmcode;
      r  = MSK_optimizetrm(task, &trmcode);
      if (r != MSK_RES_OK) { printf("Error finding solution\n"); got_mad_error(r); }
      
      MSKsolstae solsta;
      MSK_getsolsta(task, MSK_SOL_ITR, &solsta);
      
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
      
      for (int ndx = 0; ndx < bodies.size(); ndx++) {
         Vector6d new_phi;
         new_phi << result[ndx + 0], result[ndx + 1], result[ndx + 2], result[ndx + 3], result[ndx + 4], result[ndx + 5];
         
         bodies[ndx].curr_phi = new_phi;
      }
      
      MSK_deletetask(&task);
      MSK_deleteenv(&env);
   }
   else {
      phi_accum = M_accum.ldlt().solve(b);
      
      for (int ndx = 0; ndx < bodies.size(); ndx++) {
         Vector6d new_phi;
         new_phi = phi_accum.segment<6>(ndx * 6);
         
         bodies[ndx].curr_phi = new_phi;
      }
   }
   
   printf("%f, %f, %f, %f, %f, %f\n", phi_accum(0), phi_accum(1), phi_accum(2), phi_accum(3), phi_accum(4), phi_accum(5));
   
   for (int ndx = 0; ndx < bodies.size(); ndx++) {
      bodies[ndx].update_pos(h);
   }

}

// Draw everybody in the scene
void Scene::draw(MatrixStack *M, const std::shared_ptr<Program> prog, const std::shared_ptr<Shape> shape) {
   for (int ndx = 0; ndx < bodies.size(); ndx++) {
      bodies[ndx].draw(M, prog, shape);
   }
}