//
//  Scene.hpp
//  Lab07
//
//  Created by Elliot Fiske on 6/9/16.
//
//

#ifndef Scene_hpp
#define Scene_hpp

#include <stdio.h>
#include "Link.hpp"
#include "mosek_man.h"
#include "Rigid.h"

class Scene : public Rigid {
public:
   Scene();
   void make_links();
   Eigen::Vector3d step_all(double time);
   void draw(MatrixStack *M, const std::shared_ptr<Program> prog, const std::shared_ptr<Shape> shape);
   Link burd;
   void activate_burd();
   void finish_off();
   void explode();
   
   void new_bomb(double x, double y, double z, double pitch, double yaw);
   
private:
   std::vector<Link> bodies;
   std::vector<Contact> contacts;
   int num_contacts = 0;
   
   MSKenv_t env;
   
   Eigen::VectorXd phi_accum;
   Eigen::MatrixXd M_accum; // Mass matrix for all the bods
   Eigen::VectorXd f_accum;
};

#endif /* Scene_hpp */
