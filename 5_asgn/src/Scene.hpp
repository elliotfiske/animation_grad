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

class Scene {
public:
   Scene();
   void make_links();
   void step_all(double time);
   void draw(MatrixStack *M, const std::shared_ptr<Program> prog, const std::shared_ptr<Shape> shape);
   
private:
   std::vector<Link> bodies;
   std::vector<Contact> contacts;
   
   Eigen::VectorXd phi_accum;
   Eigen::MatrixXd M_accum; // Mass matrix for all the bods
};

#endif /* Scene_hpp */
