//
//  Scene.cpp
//  Lab07
//
//  Created by Elliot Fiske on 6/9/16.
//
//

#include "Scene.hpp"

Scene::Scene() {
   Link baby_link;
   bodies.push_back(baby_link);
}

// Move all the rigid bodies
void Scene::step_all(double h) {
   for (int ndx = 0; ndx < bodies.size(); ndx++) {
      bodies[ndx].step(h);
   }
}

// Draw everybody in the scene
void Scene::draw(MatrixStack *M, const std::shared_ptr<Program> prog, const std::shared_ptr<Shape> shape) {
   for (int ndx = 0; ndx < bodies.size(); ndx++) {
      bodies[ndx].draw(M, prog, shape);
   }
}