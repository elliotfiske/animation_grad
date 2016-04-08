#include <iostream>
#include <vector>
#include <random>

#define GLEW_STATIC
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "GLSL.h"
#include "Program.h"
#include "MatrixStack.h"

#include <iostream>
#include <chrono>

using namespace std;
using namespace Eigen;
using namespace std::chrono;

float random_float(float max) {
   return static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/max));
}

void do_mult(int dim, bool optimized) {
   
   MatrixXf A = MatrixXf::Random(dim, dim);
   MatrixXf B = MatrixXf::Random(dim, dim);
   MatrixXf C = MatrixXf::Random(dim, dim);
   MatrixXf D = MatrixXf::Random(dim, dim);
   MatrixXf E = MatrixXf::Random(dim, dim);
   
   MatrixXf R(dim, dim);
   
   VectorXf x = VectorXf::Random(dim, 1);
   MatrixXf xt = x.transpose();

   // Grab clock time BEFORE doing magic multiplication
   high_resolution_clock::time_point t1 = high_resolution_clock::now();
   
   if (optimized) {
      R = A * B * C * D * E * x;
   }
   else {
      R = xt * A * B * C * D * E;
   }
   
   // Grab clock time AFTER doing tons of pointless math
   high_resolution_clock::time_point t2 = high_resolution_clock::now();
   
   // Print duration!
   auto duration = duration_cast<microseconds>( t2 - t1 ).count();
   cout << "Results for " << dim << ": " << duration << " µs" << endl;
}

void do_inverse(int dim, bool optimized) {

   MatrixXf A = MatrixXf::Random(dim, dim);
   MatrixXf B = MatrixXf::Random(dim, dim);
   VectorXf b(dim, 1);
   
   MatrixXf At = A.transpose();
   
   VectorXf x = VectorXf::Random(dim);
   MatrixXf xt = x.transpose();
   
   MatrixXf R(dim, dim);
   
   // Grab clock time BEFORE doing magic multiplication
   high_resolution_clock::time_point t1 = high_resolution_clock::now();
   
   if (optimized) {
      VectorXf Ax = A * x;
      b = B.colPivHouseholderQr().solve(Ax);
      R = xt * At * b;
   }
   else {
      R = xt * At * B.inverse() * A * x;
   }
   
   // Grab clock time AFTER doing tons of pointless math
   high_resolution_clock::time_point t2 = high_resolution_clock::now();
   
   // Print duration!
   auto duration = duration_cast<microseconds>( t2 - t1 ).count();
   cout << "Results for " << dim << ": " << duration << " µs" << endl;
}

int main(int argc, char **argv)
{
   do_inverse(10, true);
   do_inverse(10, false);
   
   cout << endl;
   
   do_inverse(50, true);
   do_inverse(50, false);
   
   cout << endl;
   
   do_inverse(100, true);
   do_inverse(100, false);
   
   cout << endl;
   
   do_inverse(300, true);
   do_inverse(300, false);
   
   cout << endl;
   
   do_inverse(1000, true);
   do_inverse(1000, false);
   
   cout << endl;
   cout << "---------------------------------------------";
   cout << endl;
   
   return 0;
}
