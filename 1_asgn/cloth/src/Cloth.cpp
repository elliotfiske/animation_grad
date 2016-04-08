#include <iostream>

#include "Cloth.h"
#include "Particle.h"
#include "Spring.h"
#include "MatrixStack.h"
#include "Program.h"
#include "GLSL.h"

using namespace std;
using namespace Eigen;

shared_ptr<Spring> createSpring(const shared_ptr<Particle> p0, const shared_ptr<Particle> p1, double E)
{
	auto s = make_shared<Spring>(p0, p1);
	s->E = E;
	Vector3d x0 = p0->x;
	Vector3d x1 = p1->x;
	Vector3d dx = x1 - x0;
	s->L = dx.norm();
	return s;
}

// At each index N, contains a list of the springs for the Nth particle.
vector< vector< shared_ptr<Spring> > > all_springs_for_particle;
vector< vector< shared_ptr<Spring> > > negative_springs_for_particle;
vector< vector< shared_ptr<Spring> > > positive_springs_for_particle;

Cloth::Cloth(int rows, int cols,
			 const Vector3d &x00,
			 const Vector3d &x01,
			 const Vector3d &x10,
			 const Vector3d &x11,
			 double mass,
			 double stiffness,
			 const Vector2d &damping)
{
	assert(rows > 1);
	assert(cols > 1);
	assert(mass > 0.0);
	assert(stiffness > 0.0);
	
	this->rows = rows;
	this->cols = cols;
	this->damping = damping;
	
	// Create particles
	n = 0;
	double r = 0.02; // Used for collisions
	int nVerts = rows*cols;
	for(int i = 0; i < rows; ++i) {
		double u = i / (rows - 1.0);
		Vector3d x0 = (1 - u)*x00 + u*x10;
		Vector3d x1 = (1 - u)*x01 + u*x11;
		for(int j = 0; j < cols; ++j) {
			double v = j / (cols - 1.0);
			Vector3d x = (1 - v)*x0 + v*x1;
			auto p = make_shared<Particle>();
			particles.push_back(p);
			p->r = r;
			p->x = x;
			p->v << 0.0, 0.0, 0.0;
			p->m = mass/(nVerts);
			// Pin two particles
			if(i == 0 && (j == 0 || j == cols-1)) {
				p->fixed = true;
				p->i = -1;
			} else {
				p->fixed = false;
				p->i = n;
				n += 3;
			}
		}
	}
   
   all_springs_for_particle.resize(particles.size());
   positive_springs_for_particle.resize(particles.size());
   negative_springs_for_particle.resize(particles.size());
	
	// Create x springs
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols-1; ++j) {
			int k0 = i*cols + j;
			int k1 = k0 + 1;
			springs.push_back(createSpring(particles[k0], particles[k1], stiffness));
         
         all_springs_for_particle[k0].push_back(springs.back());
         all_springs_for_particle[k1].push_back(springs.back());
         positive_springs_for_particle[k0].push_back(springs.back());
         negative_springs_for_particle[k1].push_back(springs.back());
		}
	}
	
	// Create y springs
	for(int j = 0; j < cols; ++j) {
		for(int i = 0; i < rows-1; ++i) {
			int k0 = i*cols + j;
			int k1 = k0 + cols;
			springs.push_back(createSpring(particles[k0], particles[k1], stiffness));
         
         all_springs_for_particle[k0].push_back(springs.back());
         all_springs_for_particle[k1].push_back(springs.back());
         positive_springs_for_particle[k0].push_back(springs.back());
         negative_springs_for_particle[k1].push_back(springs.back());
		}
	}
	
	// Create shear springs
	for(int i = 0; i < rows-1; ++i) {
		for(int j = 0; j < cols-1; ++j) {
			int k00 = i*cols + j;
			int k10 = k00 + 1;
			int k01 = k00 + cols;
			int k11 = k01 + 1;
         springs.push_back(createSpring(particles[k00], particles[k11], stiffness));
         all_springs_for_particle[k00].push_back(springs.back());
         all_springs_for_particle[k11].push_back(springs.back());
         positive_springs_for_particle[k00].push_back(springs.back());
         negative_springs_for_particle[k11].push_back(springs.back());
         
			springs.push_back(createSpring(particles[k10], particles[k01], stiffness));
         all_springs_for_particle[k10].push_back(springs.back());
         all_springs_for_particle[k01].push_back(springs.back());
         positive_springs_for_particle[k10].push_back(springs.back());
         negative_springs_for_particle[k01].push_back(springs.back());
		}
	}

	// Build system matrices and vectors
	M.resize(n,n);
	K.resize(n,n);
	v.resize(n);
	f.resize(n);
	
	// Build vertex buffers
	posBuf.clear();
	norBuf.clear();
	texBuf.clear();
	eleBuf.clear();
	posBuf.resize(nVerts*3);
	norBuf.resize(nVerts*3);
	updatePosNor();
	// Texture coordinates (don't change)
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			texBuf.push_back(i/(rows-1.0));
			texBuf.push_back(j/(cols-1.0));
		}
	}
	// Elements (don't change)
	for(int i = 0; i < rows-1; ++i) {
		for(int j = 0; j < cols; ++j) {
			int k0 = i*cols + j;
			int k1 = k0 + cols;
			// Triangle strip
			eleBuf.push_back(k0);
			eleBuf.push_back(k1);
		}
	}
}

Cloth::~Cloth()
{
}

void Cloth::tare()
{
	for(int k = 0; k < (int)particles.size(); ++k) {
		particles[k]->tare();
	}
}

void Cloth::reset()
{
	for(int k = 0; k < (int)particles.size(); ++k) {
		particles[k]->reset();
	}
	updatePosNor();
}

void Cloth::updatePosNor()
{
	// Position
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			int k = i*cols + j;
			Vector3d x = particles[k]->x;
			posBuf[3*k+0] = x(0);
			posBuf[3*k+1] = x(1);
			posBuf[3*k+2] = x(2);
		}
	}
	// Normal
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols; ++j) {
			// Each particle has four neighbors
			//
			//      v1
			//     /|\
			// u0 /_|_\ u1
			//    \ | /
			//     \|/
			//      v0
			//
			// Use these four triangles to compute the normal
			int k = i*cols + j;
			int ku0 = k - 1;
			int ku1 = k + 1;
			int kv0 = k - cols;
			int kv1 = k + cols;
			Vector3d x = particles[k]->x;
			Vector3d xu0, xu1, xv0, xv1, dx0, dx1, c;
			Vector3d nor(0.0, 0.0, 0.0);
			int count = 0;
			// Top-right triangle
			if(j != cols-1 && i != rows-1) {
				xu1 = particles[ku1]->x;
				xv1 = particles[kv1]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Top-left triangle
			if(j != 0 && i != rows-1) {
				xu1 = particles[kv1]->x;
				xv1 = particles[ku0]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Bottom-left triangle
			if(j != 0 && i != 0) {
				xu1 = particles[ku0]->x;
				xv1 = particles[kv0]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			// Bottom-right triangle
			if(j != cols-1 && i != 0) {
				xu1 = particles[kv0]->x;
				xv1 = particles[ku1]->x;
				dx0 = xu1 - x;
				dx1 = xv1 - x;
				c = dx0.cross(dx1);
				nor += c.normalized();
				++count;
			}
			nor /= count;
			nor.normalize();
			norBuf[3*k+0] = nor(0);
			norBuf[3*k+1] = nor(1);
			norBuf[3*k+2] = nor(2);
		}
	}
}

typedef Eigen::Triplet<double> Trip;
vector<Trip> A_trips;

// Iterates through a 3x3 matrix and puts all the non-zero
//  entries into the list o' triplets.
//
// offset_x and offset_y determine where the matrix is in the super big matrix. ITS LATE OK
void convert_3x3_mat_to_trips(const Matrix3d &mat, int offset_row, int offset_col) {
   for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
         double curr_num = mat(i, j);
         
         if (curr_num != 0) {
            A_trips.push_back(Trip(offset_row + i, offset_col + j, curr_num));
         }
      }
   }
}

void Cloth::step(double h, const Vector3d &grav, const vector< shared_ptr<Particle> > spheres)
{
	M.setZero();
	K.setZero();
	v.setZero();
	f.setZero();
   
   A_trips.clear();
   SparseMatrix<double> A_sparse(n, n);
   
   // Set up our delicious right-hand vector.
   VectorXd b;
   b.resize(n);
   
   // Loop through every single particle
   for (int ndx = 0; ndx < particles.size(); ndx++) {
      shared_ptr<Particle> p = particles[ndx];
      
      Matrix3d mini_M = p->m * Matrix3d::Identity();
      
      int particle_ndx = p->i;
      if (particle_ndx != -1) {
         vector<shared_ptr<Spring> > curr_springs = all_springs_for_particle[ndx];
         vector<shared_ptr<Spring> > negative_curr_springs = negative_springs_for_particle[ndx];
         vector<shared_ptr<Spring> > positive_curr_springs = positive_springs_for_particle[ndx];
         
         Matrix3d diagonal_k;
         diagonal_k.setZero();
         
         // Check all of the springs attached to the CURRENT PARTICLE
         for (int s_ndx = 0; s_ndx < curr_springs.size(); s_ndx++) {
            shared_ptr<Spring> s = curr_springs[s_ndx];
            Vector3d dx = s->p1->x - s->p0->x;
            double l = dx.norm();
            
            double l_L_l = (l - s->L)/l;
            double scalar_thing = (l_L_l * dx.transpose()*dx);
            // "ks" is mini ks block between the current particle and the other particle on the end of the current spring
            Matrix3d ks = s->E/(l*l) * (
                                        (1 - l_L_l) * dx*dx.transpose() +
                                        scalar_thing * Matrix3d::Identity());
            
            ks *= damping[1] * h*h;
            
            // We do the diagonal blocks of K separately, cuz they're the only ones that depend
            // on more than 1 spring.
            diagonal_k += ks;
            
            // Tricky part! Here we fill in the entries that look like this:
            //
            //
            //   x- - -     and nothing else!
            //   |
            //   |
            //   |
            //
            //   They're all equal to -ks.
            if (s->p0->i != -1 && s->p1->i != -1) {
               if (s->p0->i > p->i) {
                  convert_3x3_mat_to_trips(-ks, s->p0->i, p->i);
                  convert_3x3_mat_to_trips(-ks, p->i, s->p0->i);
               }
               
               if (s->p1->i > p->i) {
                  convert_3x3_mat_to_trips(-ks, s->p1->i, p->i);
                  convert_3x3_mat_to_trips(-ks, p->i, s->p1->i);
               }
            }
         }
         // end looping through all the silly springs
         
         // Take the diagonal K value and add the Mass to it
         diagonal_k += mini_M + mini_M * damping[0] * h;
         convert_3x3_mat_to_trips(diagonal_k, p->i, p->i);
         
         // Set the Mv component of b
         Vector3d mv = mini_M * p->v;
         b.block<3,1>(particle_ndx, 0) = mv + p->m * grav * h;
         
         // calculate the rest of b with the forces (easy)
         for (int p_ndx = 0; p_ndx < positive_curr_springs.size(); p_ndx++) {
            shared_ptr<Spring> s = positive_curr_springs[p_ndx];
            Vector3d dx = s->p1->x - s->p0->x;
            double l = dx.norm();
            Vector3d fs = s->E * (l - s->L) * dx / l;
            fs *= h;
            
            if (s->p0->i != -1) {
               b.block<3, 1>(particle_ndx, 0) += fs;
            }
         }
         
         // dont look at this code its bad ok
         
         for (int n_ndx = 0; n_ndx < negative_curr_springs.size(); n_ndx++) {
            shared_ptr<Spring> s = negative_curr_springs[n_ndx];
            Vector3d dx = s->p1->x - s->p0->x;
            double l = dx.norm();
            Vector3d fs = s->E * (l - s->L) * dx / l;
            fs *= h;
            
            if (s->p1->i != -1) {
               b.block<3, 1>(particle_ndx, 0) += -fs;
            }
         }
      }
   }

   VectorXd result_v;
   result_v.resize(n);
   
   A_sparse.setFromTriplets(A_trips.begin(), A_trips.end());
   
   ConjugateGradient<SparseMatrix<double> > cg;
   cg.setMaxIterations(25);
   cg.setTolerance(1e-3);
   cg.compute(A_sparse);
   result_v = cg.solveWithGuess(b, v);
   
   // Set each particles' new velocity
   for (int ndx = 0; ndx < particles.size(); ndx++) {
      shared_ptr<Particle> p = particles[ndx];
      
      int particle_ndx = p->i;
      if (particle_ndx != -1) {
         p->v = result_v.block<3, 1>(particle_ndx, 0);
      }
   }
   
   // Set each particles' new position
   for (int ndx = 0; ndx < particles.size(); ndx++) {
      shared_ptr<Particle> p = particles[ndx];
      
      int particle_ndx = p->i;
      if (particle_ndx != -1) {
         p->x += h * p->v;
      }
   }
   
   // Check collision
   for (int ndx = 0; ndx < spheres.size(); ndx++) {
      shared_ptr<Particle> collider = spheres[ndx];
      
      for (int p_ndx = 0; p_ndx < particles.size(); p_ndx++) {
         shared_ptr<Particle> p = particles[p_ndx];
         Vector3d dx = p->x - collider->x;
         double dist = dx.norm();
         if (dist <= p->r + collider->r) {
            p->x = ( (collider->r + p->r) * dx.normalized() + collider->x);
            
            Vector3d projected = p->v.dot(dx.normalized()) * dx;
            p->v -= projected;
         }
      }
   }
	
	// Update position and normal buffers
	updatePosNor();
}

void Cloth::init()
{
	glGenBuffers(1, &posBufID);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
	
	glGenBuffers(1, &norBufID);
	glBindBuffer(GL_ARRAY_BUFFER, norBufID);
	glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_DYNAMIC_DRAW);
	
	glGenBuffers(1, &texBufID);
	glBindBuffer(GL_ARRAY_BUFFER, texBufID);
	glBufferData(GL_ARRAY_BUFFER, texBuf.size()*sizeof(float), &texBuf[0], GL_STATIC_DRAW);
	
	glGenBuffers(1, &eleBufID);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, eleBuf.size()*sizeof(unsigned int), &eleBuf[0], GL_STATIC_DRAW);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	
	assert(glGetError() == GL_NO_ERROR);
}

void Cloth::draw(shared_ptr<MatrixStack> MV, const shared_ptr<Program> p) const
{
	// Draw mesh
	glUniform3fv(p->getUniform("kdFront"), 1, Vector3f(1.0, 0.0, 0.0).data());
	glUniform3fv(p->getUniform("kdBack"),  1, Vector3f(1.0, 1.0, 0.0).data());
	MV->pushMatrix();
	glUniformMatrix4fv(p->getUniform("MV"), 1, GL_FALSE, MV->topMatrix().data());
	int h_pos = p->getAttribute("vertPos");
	GLSL::enableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, posBufID);
	glBufferData(GL_ARRAY_BUFFER, posBuf.size()*sizeof(float), &posBuf[0], GL_DYNAMIC_DRAW);
	glVertexAttribPointer(h_pos, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	int h_nor = p->getAttribute("vertNor");
	GLSL::enableVertexAttribArray(h_nor);
	glBindBuffer(GL_ARRAY_BUFFER, norBufID);
	glBufferData(GL_ARRAY_BUFFER, norBuf.size()*sizeof(float), &norBuf[0], GL_DYNAMIC_DRAW);
	glVertexAttribPointer(h_nor, 3, GL_FLOAT, GL_FALSE, 0, (const void *)0);
//	int h_tex = p->getAttribute("vertTex");
//	GLSL::enableVertexAttribArray(h_tex);
//	glBindBuffer(GL_ARRAY_BUFFER, texBufID);
//	glVertexAttribPointer(h_tex, 2, GL_FLOAT, GL_FALSE, 0, (const void *)0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, eleBufID);
	for(int i = 0; i < rows; ++i) {
		glDrawElements(GL_TRIANGLE_STRIP, 2*cols, GL_UNSIGNED_INT, (const void *)(2*cols*i*sizeof(unsigned int)));
	}
	//GLSL::disableVertexAttribArray(h_tex);
	GLSL::disableVertexAttribArray(h_nor);
	GLSL::disableVertexAttribArray(h_pos);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	MV->popMatrix();
}
