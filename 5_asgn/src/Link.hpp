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

template <typename T>
T clip(const T& n, const T& lower, const T& upper) {
   return std::max(lower, std::min(n, upper));
}

struct Contact {
   int rigid_body_ndx;
   int rigid_body_other_ndx;
   Eigen::Vector4d xw; // World space coords of contact
   Eigen::Vector3d nw; // World space coords of normal
   Eigen::Vector3d tangent0w;
   Eigen::Vector3d tangent1w;
   Vector6d N_component;
};


class Link : public Rigid
{
public:
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
   Link(double w, double h, double d, double pos_x, double pos_y, double pos_z, double m, int i);
   virtual ~Link();
   
   double width;
   double height;
   double depth;
   
   Eigen::Vector3d explosion_force;
   
   bool missile;
   Eigen::Vector3d manual_velocity;
   
   double mass;
   
   int body_ndx;
   
   std::vector<Contact> contacts;
   
   // Rigid transform
   Eigen::Matrix4d curr_E;
   Vector6d curr_phi;
   
   // Mass MAtrix
   Matrix6d M_mass;
   
   // Calculate the current f-component
   Vector6d get_curr_f();
   
   void update_pos(double h);
   
   // For now, just the rotation around the Z axis
   float angle;
    
   Eigen::Vector3d position;
   
   void draw(MatrixStack *M, const std::shared_ptr<Program> prog, const std::shared_ptr<Shape> shape);
   void step(double h);
   void do_collision(std::vector<Contact>* scene_contacts);
   void check_corner(double x_offset, double y_offset, double z_offset, std::vector<Contact>* all_contacts);
    
   
   
   // What shape to draw when we draw this Link
   static std::shared_ptr<Shape> shape;
};

#endif /* Link_hpp */
