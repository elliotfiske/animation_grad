//
//  Scene.cpp
//  Lab07
//
//  Created by Elliot Fiske on 6/9/16.
//
//

#include "Scene.hpp"
#include "odeBoxBox.h"

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

Scene::Scene() :
burd(1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1) {
   MSKrescodee r = MSK_makeenv(&env, NULL);
   if (r != MSK_RES_OK) { printf("Wow, a problem already :3\n"); got_mad_error(r); }
}

void Scene::make_links() {
   bodies.clear();
   
   int scene_num = 3;
   switch (scene_num) {
      case 2: {
         for (int i = 0; i < 20; i++) {
            double tower_block_height = 0.5;
            Link lol(0.5, tower_block_height, 0.5, 0.0, (tower_block_height / 2) + i * (tower_block_height + 0.1), 0.0, 0.1, i);
            bodies.push_back(lol);
         }
         break;
      }
      case 3:
         Link baby_link(0.2, 3.0, 1.0, -1.0, 1.5, 0.0, 1.0, bodies.size()); // leg 1
         bodies.push_back(baby_link);
         
         Link baby_link2(0.2, 3.0, 1.0, 1.0, 1.5, 0.0, 1.0, bodies.size()); // leg 2
         bodies.push_back(baby_link2);
         
         Link baby_link3(2.1, 0.5, 1.0, 0.0, 3.25, 0.0, 1.0, bodies.size()); // hips
         bodies.push_back(baby_link3);
         
         Link baby_link4(0.6, 2.0, 1.0, 0.0, 4.5, 0.0, 1.0, bodies.size()); // torso
         bodies.push_back(baby_link4);
         
         Link baby_link5(3.0, 0.5, 1.0, 0.0, 5.75, 0.0, 1.0, bodies.size()); // shoulders
         bodies.push_back(baby_link5);
         
         Link baby_link6(0.2, 1.0, 1.0, -1.5, 6.5, 0.0, 1.0, bodies.size()); // arm 1
         bodies.push_back(baby_link6);
         
         Link baby_link7(0.2, 1.0, 1.0, 1.5, 6.5, 0.0, 1.0, bodies.size()); // arm 2
         bodies.push_back(baby_link7);
         
         Link baby_link8(0.4, 2.0, 0.5, 0.0, 7.0, 0.0, 1.0, bodies.size()); // neck
         bodies.push_back(baby_link8);
         
         Link baby_link9(2.0, 2.0, 1.0, 0.0, 9.0, 0.0, 1.0, bodies.size()); // head
         bodies.push_back(baby_link9);
         
         break;
   }
   
   Link wall1(1.0, 20.0, 4.0, 6.0, 11.0, 0.0, 1.0, bodies.size());
   wall1.curr_phi.tail(3) = Vector3d(-2.0, 0.0, 0.0);
   bodies.push_back(wall1);
   
   burd = Link(0.5, 0.5, 0.5, -8.0, 0.25, 0.0, 1.0, bodies.size());
   burd.mass = 10;
//   burd.missile = true;
   
   M_accum = MatrixXd::Zero(bodies.size() * 6, bodies.size() * 6);
   phi_accum.resize(bodies.size() * 6);
   f_accum.resize(bodies.size() * 6);
   
   for (int i_ndx = 0; i_ndx < bodies.size() * 6; i_ndx++) {
      phi_accum(i_ndx) = 0;
   }
   
   for (int ndx = 0; ndx < bodies.size(); ndx++) {
      M_accum.block<6, 6>(ndx * 6, ndx*6) = bodies[ndx].M_mass;
   }
   
   finish_off();
}

void Scene::finish_off() {
   M_accum = MatrixXd::Zero(bodies.size() * 6, bodies.size() * 6);
   phi_accum.resize(bodies.size() * 6);
   f_accum.resize(bodies.size() * 6);
   
   for (int i_ndx = 0; i_ndx < bodies.size() * 6; i_ndx++) {
      phi_accum(i_ndx) = 0;
   }
   
   for (int ndx = 0; ndx < bodies.size(); ndx++) {
      M_accum.block<6, 6>(ndx * 6, ndx*6) = bodies[ndx].M_mass;
      bodies[ndx].body_ndx = ndx;
   }
}

// Move all the rigid bodies
Eigen::Vector3d Scene::step_all(double h) {
   contacts.clear();
   num_contacts = 0;
   
   for (int ndx = 0; ndx < bodies.size(); ndx++) {
      phi_accum.segment<6>(ndx * 6) = bodies[ndx].curr_phi;
//      bodies[ndx].step(h);
      
      f_accum.segment<6>(ndx * 6) = bodies[ndx].get_curr_f();
   }
   
   for (int ndx = 0; ndx < bodies.size(); ndx++) {
      bodies[ndx].do_collision(&contacts);
   }
   
   num_contacts = contacts.size(); // weird stuff
   
   for (int a = 0; a < bodies.size(); a++) {
      for (int b = a + 1; b < bodies.size(); b++) {
         if (a == b) {
            continue;
         }
         
         Link bod_a = bodies[a];
         Vector3d whd_a(bod_a.width, bod_a.height, bod_a.depth);
         
         Link bod_b = bodies[b];
         Vector3d whd_b(bod_b.width, bod_b.height, bod_b.depth);
         
         Contacts muh_contacts = odeBoxBox(bod_a.curr_E, whd_a, bod_b.curr_E, whd_b);
         
         for (int ndx = 0; ndx < muh_contacts.count; ndx++) {
            Contact c;
            c.xw << muh_contacts.positions[ndx], 1.0;
            
            c.nw = -muh_contacts.normal;
            
            c.rigid_body_ndx = bod_a.body_ndx;
            c.rigid_body_other_ndx = bod_b.body_ndx;
            
//            Matrix3x6d J_a = bod_a.curr_E.block<3,3>(0,0) * gamma(xi_a.head(3));
//            c.N_component = -c.nw.transpose() * J_a;
//            Vector3d vw_a = bod_a.curr_phi * J_a;
            
            
//            Matrix3x6d J_b = bod_b.curr_E.block<3,3>(0,0) * gamma(xi_b.head(3));
//            c_b.N_component = c_b.nw.transpose() * J_b;
//            Vector3d vw_b = bod_b.curr_phi * J_b;
            
            contacts.push_back(c);
            num_contacts += 2; //ugh glitch central plz
            
            if (bodies[a].missile) {
               bodies[b].manual_velocity = bodies[a].curr_phi.tail(3);
            }
            
            if (bodies[b].missile) {
               bodies[a].manual_velocity = bodies[b].curr_phi.tail(3);
            }
         }
      }
   }
   
   int num_vars = 6 * bodies.size();
   
   // Solve Ax = b where A = Mi
   // and b is this huge thing from the worksheet // TODO: nopes
   MatrixXd b = M_accum * phi_accum +
                 h * f_accum;
   
//   printf("B is: %f %f %f %f %f %f\n", b(0), b(1), b(2), b(3), b(4), b(5));
   
   if (isnan(b(5)) || isnan(b(0)) || isnan(b(2)) || isnan(b(1)) || isnan(b(3)) || isnan(b(4))) {
      printf("ah snap\n");
   }
   
   if (b(2) > 999999999) {
      printf("wut\n");
   }
   
//   bodies[0].step(h);
   
   vector<double> result;
   
   if (contacts.size() > 0) {
      
      // Collisions present! Let's ask our friend Mosek for help solving this one.
      MSKtask_t task = NULL;
      int num_constraints = num_contacts;
      MSKrescodee r;
      
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
         
         r = MSK_putcj(task, j, -b(j, 0)); // TODO: idk whats supposed to go here
         if (r != MSK_RES_OK) { printf("Error adding linear term\n"); got_mad_error(r); }
         
         r = MSK_putvarbound(task, j, MSK_BK_FR, -MSK_INFINITY, +MSK_INFINITY);
         if (r != MSK_RES_OK) { printf("Error adding variable bound\n"); got_mad_error(r); }
      }
      
      //      printf("Contact: %f %f %f %f %f %f\n", contacts[0].N_component[0], contacts[0].N_component[1], contacts[0].N_component[2], contacts[0].N_component[3], contacts[0].N_component[4], contacts[0].N_component[5]);
      MatrixXd N_accum = MatrixXd::Zero(num_contacts, 6 * bodies.size());
      for (int contact_ndx = 0; contact_ndx < contacts.size(); contact_ndx++) {
         Contact c = contacts[contact_ndx];
         
         Link bod_a = bodies[c.rigid_body_ndx];
         Matrix3d Ra = bod_a.curr_E.block<3,3>(0,0);
         Vector4d xa = bod_a.curr_E.inverse() * c.xw;
         Vector6d N_guy = c.nw.transpose() * Ra * gamma(xa.head(3));
         N_accum.block(contact_ndx, 6 * c.rigid_body_ndx, 1, 6) = N_guy.transpose();
         
         if (c.rigid_body_other_ndx != -1) {
            Link bod_b = bodies[c.rigid_body_other_ndx];
            Matrix3d Rb = bod_b.curr_E.block<3,3>(0,0);
            Vector4d xb = bod_b.curr_E.inverse() * c.xw;
            Vector6d N_guy_b = -c.nw.transpose() * Rb * gamma(xb.head(3));
            N_accum.block(contact_ndx, 6 * c.rigid_body_other_ndx, 1, 6) = N_guy_b.transpose();
         }
         
      }
      double restitution = 0.9;
      VectorXd Nv = N_accum * phi_accum * restitution;
      
//      printf("Nv size: %ld\n", Nv.rows());
      
      // Insert the values of the constraint
      for (int contact_ndx = 0; contact_ndx < N_accum.rows(); contact_ndx++) {
         for (int j = 0; j < N_accum.cols(); j++) {
            r = MSK_putaij(task, contact_ndx, j, N_accum(contact_ndx, j));
            if (r != MSK_RES_OK) {
               printf("Error setting constraint\n"); got_mad_error(r);
            }
         }
         
         r = MSK_putconbound(task, contact_ndx, MSK_BK_LO, -Nv(contact_ndx), +MSK_INFINITY);
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
         new_phi << result[ndx*6 + 0], result[ndx*6 + 1], result[ndx*6 + 2], result[ndx*6 + 3], result[ndx*6 + 4], result[ndx*6 + 5];
         
         bodies[ndx].curr_phi = new_phi;
         if (bodies[ndx].manual_velocity != Vector3d::Zero()) {
            bodies[ndx].curr_phi.tail(3) = bodies[ndx].manual_velocity;
            bodies[ndx].manual_velocity = Vector3d::Zero();
         }
      }
      
      MSK_deletetask(&task);
   }
   else {
      phi_accum = M_accum.ldlt().solve(b);
      
      for (int ndx = 0; ndx < bodies.size(); ndx++) {
         Vector6d new_phi;
         new_phi = phi_accum.segment<6>(ndx * 6);
         
         bodies[ndx].curr_phi = new_phi;
      }
   }
   
//   printf("%f, %f, %f, %f, %f, %f\n", phi_accum(0), phi_accum(1), phi_accum(2), phi_accum(3), phi_accum(4), phi_accum(5));
   
   for (int ndx = 0; ndx < bodies.size(); ndx++) {
      bodies[ndx].update_pos(h);
   }

   return bodies[0].curr_E.block<3, 1>(0, 3);
}

bool poop = true;

// Draw everybody in the scene
void Scene::draw(MatrixStack *M, const std::shared_ptr<Program> prog, const std::shared_ptr<Shape> shape) {
   for (int ndx = 0; ndx < bodies.size(); ndx++) {
      bodies[ndx].draw(M, prog, shape);
   }
   
   if (poop) {
      burd.draw(M, prog, shape);
   }
}

void Scene::explode() {
   for (int ndx = 0; ndx < bodies.size(); ndx++) {
      bodies[ndx].explosion_force = Vector3d(0.0, 5000.0, 0.0);
   }
}

void Scene::new_bomb(double x, double y, double z, double pitch, double yaw) {
   burd.manual_velocity = Vector3d(9.0, 8.0, 0.0);
   Link new_bomb(0.5, 0.5, 0.5, x, y, z, 5.0, bodies.size());
   new_bomb.manual_velocity = Vector3d(sin(pitch), 0.0, cos(pitch));
   bodies.push_back(new_bomb);
   
   finish_off();
   poop = false;
}