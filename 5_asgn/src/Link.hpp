//
//  Link.hpp
//  Lab07
//
//  Created by Elliot Fiske on 2/21/16.
//
//

#ifndef Link_hpp
#define Link_hpp

#include <stdio.h>
#include <memory>
#include <vector>

#define EIGEN_DONT_ALIGN_STATICALLY
#include <Eigen/Dense>
#include "MatrixStack.h"
#include "Rigid.h"
#include "Shape.h"

struct Contact {
   int rigid_body_ndx;
   Eigen::Vector4d xw; // World space coords of contact
   Eigen::Vector3d nw; // World space coords of normal
   Eigen::Vector3d tangent0w;
   Eigen::Vector3d tangent1w;
   Eigen::VectorXd N_component;
};


class Link : public Rigid
{
public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   Link();
   virtual ~Link();
   
   std::shared_ptr<Link> parent;
   std::vector<std::shared_ptr<Link> > children;
   
   std::vector<Contact> contacts;
   
   // Rigid transform
   Eigen::Matrix4d curr_E;
   Eigen::VectorXd curr_phi;
   
   // For now, just the rotation around the Z axis
   float angle;
    
   Eigen::Vector3d position;
   
   void draw(MatrixStack *M, const std::shared_ptr<Program> prog, const std::shared_ptr<Shape> shape);
   void step(double h);
   void do_collision();
   void check_corner(double x_offset, double y_offset, double z_offset);
    
   
   
   // What shape to draw when we draw this Link
   static std::shared_ptr<Shape> shape;
};

#endif /* Link_hpp */
