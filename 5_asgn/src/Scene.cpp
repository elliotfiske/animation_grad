//
//  Scene.cpp
//  Lab07
//
//  Created by Elliot Fiske on 6/9/16.
//
//

#include "Scene.hpp"

using namespace std;
using namespace Eigen;

Scene::Scene() {
}

void Scene::make_links() {

   
   Link baby_link(0.5, 1.0, 1.0, 0.0, 2.0, 0.0, 1.0);
   bodies.push_back(baby_link);
   
   M_accum.resize(bodies.size() * 6, bodies.size() * 6);
   
   for (int ndx = 0; ndx < bodies.size(); ndx++) {
      M_accum.block<6, 6>(ndx * 6, ndx*6) = bodies[ndx].M_mass;
   }
   
   phi_accum.resize(bodies.size() * 6);
}

// Move all the rigid bodies
void Scene::step_all(double h) {
   
   for (int ndx = 0; ndx < bodies.size(); ndx++) {
      phi_accum.segment<6>(ndx * 6) = bodies[ndx].curr_phi;
      bodies[ndx].step(h);
      
      Vector6d farce = bodies[ndx].get_curr_f();
   }
 
//   do_collision();
//   
//   int num_vars = 6 * bodies.size();
//   
//   // Solve Ax = b where A = Mi
//   // and b is this huge thing from the worksheet
//   VectorXd b = Mi * curr_phi +
//   h * (bracket_phi.transpose() * Mi*curr_phi + fg);
//   
//   if (contacts.size() > 0) {
//      
//      // Collisions present! Let's ask our friend Mosek for help solving this one.
//      MSKenv_t env = NULL;
//      MSKrescodee r = MSK_makeenv(&env, NULL);
//      MSKtask_t task = NULL;
//      int num_constraints = contacts.size();
//      
//      if (r != MSK_RES_OK) { printf("Wow, a problem already :3\n"); got_error(r); }
//      
//      r = MSK_maketask(env, num_constraints, num_vars, &task);
//      if (r != MSK_RES_OK) { printf("ERror making task\n"); got_error(r); }
//      
//      r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);
//      
//      if (r != MSK_RES_OK) { printf("How did you get an error doing that??\n"); got_error(r); }
//      
//      // Stick the constraints in, from the list of collisions we got from back up there.
//      r = MSK_appendcons(task, num_constraints);
//      if (r != MSK_RES_OK) { printf("Error appending constraints\n"); got_error(r); }
//      
//      // Append 6 * number of rigidbodies to the equation. This is one variable for each
//      //  component in phi, per rigidbody.
//      r = MSK_appendvars(task, 6 * bodies.size());
//      if (r != MSK_RES_OK) { printf("Error appending variables\n"); got_error(r); }
//      
//      for (int j = 0; j < num_vars; j++) {
//         
//         r = MSK_putcj(task, j, -b[j]); // TODO: idk whats supposed to go here
//         if (r != MSK_RES_OK) { printf("Error adding linear term\n"); got_error(r); }
//         
//         r = MSK_putvarbound(task, j, MSK_BK_FR, -MSK_INFINITY, +MSK_INFINITY);
//         if (r != MSK_RES_OK) { printf("Error adding variable bound\n"); got_error(r); }
//      }
//      
//      // TODO: calculate N, put it here in the A-variable's spot.
//      //r = MSK_putacol(task, j, <#MSKint32t nzj#>, <#const MSKint32t *subj#>, <#const MSKrealt *valj#>)
//      
//      
//      // we need the "indexes of the non-zero elements in the row", but since everything has non-zero,
//      //  we'll just make an array with 1->6, for goofs and gaffs.
//      vector<MSKint32t> row_ndxs;
//      for (int ndx = 0; ndx < 6 * 1; ndx++) {
//         row_ndxs.push_back(ndx);
//      }
//      
//      //      printf("Contact: %f %f %f %f %f %f\n", contacts[0].N_component[0], contacts[0].N_component[1], contacts[0].N_component[2], contacts[0].N_component[3], contacts[0].N_component[4], contacts[0].N_component[5]);
//      
//      // Insert the rows of the constraint
//      for (int contact_ndx = 0; contact_ndx < contacts.size(); contact_ndx++) {
//         for (int n_ndx = 0; n_ndx < 6; n_ndx++) {
//            r = MSK_putaij(task, contact_ndx, n_ndx, contacts[contact_ndx].N_component[n_ndx]);
//            if (r != MSK_RES_OK) {
//               printf("Error setting constraint\n"); got_error(r);
//            }
//         }
//         
//         r = MSK_putconbound(task, contact_ndx, MSK_BK_LO, 0, +MSK_INFINITY);
//         if (r != MSK_RES_OK) { printf("Error setting contraint bounds\n"); got_error(r); }
//      }
//      
//      
//      
//      // input  mass matrix
//      vector<MSKint32t> qsubi;
//      qsubi.resize(6);
//      vector<MSKint32t> qsubj;
//      qsubj.resize(6);
//      vector<double>    qval;
//      qval.resize(6);
//      
//      // Building mass matrix by hand here :/
//      for (int ndx = 0; ndx < 3; ndx++) {
//         qsubi[ndx] = ndx;
//         qsubj[ndx] = ndx;
//         qval[ndx] = m * s*s / 6;
//      }
//      
//      for (int ndx = 3; ndx < 6; ndx++) {
//         qsubi[ndx] = ndx;
//         qsubj[ndx] = ndx;
//         qval[ndx] = m;
//      }
//      
//      r = MSK_putqobj(task, 6, &qsubi[0], &qsubj[0], &qval[0]);
//      if (r != MSK_RES_OK) { printf("Error inputting Mass Matrix\n"); got_error(r); }
//      
//      MSKrescodee trmcode;
//      r  = MSK_optimizetrm(task, &trmcode);
//      if (r != MSK_RES_OK) { printf("Error finding solution\n"); got_error(r); }
//      
//      MSKsolstae solsta;
//      MSK_getsolsta(task, MSK_SOL_ITR, &solsta);
//      
//      vector<double> result;
//      result.resize(num_vars);
//      
//      switch(solsta) {
//         case MSK_SOL_STA_OPTIMAL:
//         case MSK_SOL_STA_NEAR_OPTIMAL:
//            MSK_getxx(task, MSK_SOL_ITR, &result[0]);
//            break;
//            
//         default:
//            printf("Got some weird solution status: %d\n", solsta);
//            break;
//      }
//      
//      for (int ndx = 0; ndx < curr_phi.rows(); ndx++) {
//         curr_phi[ndx] = result[ndx];
//      }
//      
//      MSK_deletetask(&task);
//      MSK_deleteenv(&env);
//   }
//   else {
//      curr_phi = Mi.ldlt().solve(b);
//   }
//   
//   //   printf("%f, %f, %f, %f, %f, %f\n", curr_phi(0), curr_phi(1), curr_phi(2), curr_phi(3), curr_phi(4), curr_phi(5));
//   
//   Matrix4d next_E = integrate(curr_E, curr_phi, h);
//   curr_E = next_E;

}

// Draw everybody in the scene
void Scene::draw(MatrixStack *M, const std::shared_ptr<Program> prog, const std::shared_ptr<Shape> shape) {
   for (int ndx = 0; ndx < bodies.size(); ndx++) {
      bodies[ndx].draw(M, prog, shape);
   }
}