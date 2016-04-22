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

void Cloth::circle_constraint() {
   G.resize(1, n);
   G(0, 1) = 1;
   G(0, 2) = 0.2;
}

void Cloth::fake_pinning() {
   G.resize(6, n);
   G.block<3, 3>(0, 0) = Matrix3d::Identity();
   G.block<3, 3>(3, (cols - 1)*3) = Matrix3d::Identity();
}

void Cloth::v0_negative_v1() {
   G.resize(3, n);
   G.block<3, 3>(0, 0) = Matrix3d::Identity();
   G.block<3, 3>(0, n-3) = Matrix3d::Identity();
}

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
//			if(i == 0 && (j == 0 || j == cols-1)) {
//				p->fixed = true;
//				p->i = -1;
//			} else {
				p->fixed = false;
				p->i = n;
				n += 3;
//			}
		}
	}
	
	// Create x springs
	for(int i = 0; i < rows; ++i) {
		for(int j = 0; j < cols-1; ++j) {
			int k0 = i*cols + j;
			int k1 = k0 + 1;
			springs.push_back(createSpring(particles[k0], particles[k1], stiffness));
		}
	}
	
	// Create y springs
	for(int j = 0; j < cols; ++j) {
		for(int i = 0; i < rows-1; ++i) {
			int k0 = i*cols + j;
			int k1 = k0 + cols;
			springs.push_back(createSpring(particles[k0], particles[k1], stiffness));
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
			springs.push_back(createSpring(particles[k10], particles[k01], stiffness));
		}
	}
	
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
   
   // Constraints!
//   v0_negative_v1();
   fake_pinning();
//   circle_constraint();
   
   // Build system matrices and vectors
   int cr = G.rows(); // cr = constraint rows
   M.resize(n + cr, n + cr);
   K.resize(n + cr, n + cr);
   v.resize(n + cr);
   f.resize(n + cr);
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

void Cloth::step(double h, const Vector3d &grav, const vector< shared_ptr<Particle> > spheres)
{
	M.setZero();
	K.setZero();
	v.setZero();
	f.setZero();

   // Fill in M, v and f
   for (int ndx = 0; ndx < particles.size(); ndx++) {
      shared_ptr<Particle> p = particles[ndx];
      
      int particle_ndx = p->i;
      if (particle_ndx != -1) {
         v.block<3,1>(particle_ndx, 0) = p->v;
         
         Matrix3d curr_M = p->m * Matrix3d::Identity();
         M.block<3,3>(particle_ndx, particle_ndx) = curr_M;
         
         f.block<3, 1>(particle_ndx, 0) = p->m * grav;
      }
   }
   
   // Add the spring forces to f, and the stiffness stuff to k
   for (int ndx = 0; ndx < springs.size(); ndx++) {
      shared_ptr<Spring> s = springs[ndx];
      Vector3d dx = s->p1->x - s->p0->x;
      double l = dx.norm();
      Vector3d fs = s->E * (l - s->L) * dx / l;
      
      if (s->p0->i != -1) {
         f.block<3, 1>(s->p0->i, 0) += fs;
      }
      
      if (s->p1->i != -1) {
         f.block<3, 1>(s->p1->i, 0) += -fs;
      }
      
      double l_L_l = (l - s->L)/l;
      double scalar_thing = (l_L_l * dx.transpose()*dx);
      Matrix3d ks = s->E/(l*l) * (
                                  (1 - l_L_l) * dx*dx.transpose() +
                                  scalar_thing * Matrix3d::Identity());
      
      if (s->p0->i != -1) {
         K.block<3, 3>(s->p0->i, s->p0->i) += ks;
      }
      
      if (s->p1->i != -1) {
         K.block<3, 3>(s->p1->i, s->p1->i) += ks;
      }
      
      if (s->p0->i != -1 && s->p1->i != -1) {
         K.block<3, 3>(s->p0->i, s->p1->i) += -ks;
         K.block<3, 3>(s->p1->i, s->p0->i) += -ks;
      }
   }
   
   
   // Solve the system (M + D) v = Mv + hf   -> for v
   MatrixXd A;
   A.resize(n + G.rows(), n + G.rows());
   A = M + h*damping[0]*M + h*h * damping[1]*K;
   A.block(n, 0, G.rows(), G.cols()) = G;
   A.block(0, n, G.cols(), G.rows()) = G.transpose();
   
   VectorXd b;
   b = M*v + h*f;
//   b.block<3, 1>(n, 0) = spheres[0]->v;
   b(n+1, 0) = spheres[0]->v(2);
   b(n+4, 0) = spheres[0]->v(2);
   
   VectorXd result_v;
   result_v = A.ldlt().solve(b);
   
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
