Lab 7 -  Forward Kinematics / Hierarchical Modeling
===================================================

Today we're going to create a simple hierarchical robot.

Please download the code for the lab and go over the code.

- On the web: <http://users.csc.calpoly.edu/~ssueda/teaching/CSC474/2016W/labs/L07/L07.zip>
- On the server: `cp ~ssueda/www/teaching/CSC474/2016W/labs/L07/L07.zip .`

When you run the code, you should see a textured object and a background grid.
You'll be using this object as the mesh for the robot links. When you drag the
mouse with the ALT key pressed, you should see some console output. You'll be
using these values to control the joint angles of the robot.

Create a class called `Link`. It should have the following member variables:

- A pointer to the parent link
- An array (or an `std::vector`) of pointers to child links
- A 4x4 matrix, $^p_iE$, for the transform of this link with respect to the
  parent link
- A 4x4 matrix, $^i_{Mi}E$, for the transform of the mesh with respect to this
  link
- A float for the current angle.

Since you're going to create a 2D planar mechanism, you only need a single
float for the rotation. For convenience, the origin of each link is going to
be where the joint is. For example, for the "upper arm" link, the origin will
be at the shoulder, and for the "lower arm" link the origin is going to be at
the elbow.

<img src="images/image1.jpg" width="500px"/>

The arrows indicate what the two transformation matrices represent. $^p_iE$
describes where the current link is with respect to the parent link. In the
figure above, it represents where the elbow joint is with respect to the
shoulder joint. $^i_{Mi}E$ describes where the mesh's origin is with respect
to the current link.

The first four of these member variables should be set in the scene loading
function. The last variable, `angle`, should be set using the mouse in the
`cursor_position_callback()` function.

The pseudocode for the recursive drawing function is

	void Link::draw(MatrixStack M) {
		M.push();
		M.mult(Ei);
		M.rotate(angle);
		M.push();
		M.mult(Mi);
		Send M's top matrix to the GPU
		drawMesh();
		M.pop();
		for child in children
			child.draw(M);
		M.pop();
	}

Here, the variable `Ei` refers to $^p_iE$, and the variable `Mi` refers to
$^i_{Mi}E$. Start with a simple robot first. Your final task is to build a
two-arm robot composed of 5 links. The root should be draw at the world origin
and should be fixed. Moving the mouse in one direction (x or y) should change
the shoulder angles, and moving the mouse in the other direction should change
the elbow angles.

<img src="images/image2.jpg" width="400px"/>
