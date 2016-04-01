Lab 2 - Cubic Splines
=====================

Today we're going to draw cubic spline curves in OpenGL.

Please download the code for the lab and go over the code.

- On the web: <http://users.csc.calpoly.edu/~ssueda/teaching/CSC474/2016W/labs/L02/L02.zip>
- On the server: `cp ~ssueda/www/teaching/CSC474/2016W/labs/L02/L02.zip .`

Task 0
------

Put your name on the window title.

Task 1
------

Run cmake and then compile the provided source files. On some lab machines,
the compiler may complain about the random number generator. It this happens,
just delete that part of the code.

The code doesn't draw any curves yet — it simply draws dots whenever you
shift-click on the window. These are going to be the control points of the
spline curve. These control points are in 3D. You can rotate the camera with
the mouse. Pressing the 'L' key will toggle on/off the connecting lines.

<img src="images/image1.jpg" width="500px"/>

Using just the first four control points, draw a Bezier curve. The formula is
given by the following matrix equation.

$$
\begin{aligned}
p(u) &= G B \bar{u},\\
G &=
\begin{pmatrix}
x_0 & x_1 & x_2 & x_3\\
y_0 & y_1 & y_2 & y_3\\
z_0 & z_1 & z_2 & z_3
\end{pmatrix},
\quad
B = 
\begin{pmatrix}
1 & -3 & 3 & -1\\
0 & 3 & -6 & 3\\
0 & 0 & 3 & -3\\
0 & 0 & 0 & 1
\end{pmatrix},
\quad
\bar{u} = 
\begin{pmatrix}
1\\
u\\
u^2\\
u^3
\end{pmatrix}.
\end{aligned}
$$

<img src="images/image2.jpg" width="500px"/>

Here, p(u) is a 3x1 vector corresponding to the x, y, and z coordinates of the
curve position at the parameter u. To draw the curve, the parameter u should
go from 0 to 1. G is called the geometry matrix, composed of the control point
positions. B is called the spline basis matrix.

In Eigen, you can create a 4x4 matrix using the built-in class
`Eigen::Matrix4f` and a 4x1 vector using the built-in class `Eigen::Vector4f`.
(If you declare `using namespace Eigen`, you don't need to say `Eigen::` every
time.) For an arbitrarily sized matrices, you can use `Eigen::MatrixXf`. For
example, to do the matrix computation above, you would do something like this:

	MatrixXf G(3,4); // 3 by 4 matrix
	Matrix4f B;      // 4 by 4 matrix
	Vector4f uVec;   // 4 by 1 vector
	...
	// Fill in G, B, and uVec
	...
	Vector3f p = G*B*uVec; // 3 by 1 vector

Eigen is a powerful library with a lot of features. Check online for more details.

Task 2
------

There is a keyboard hook for the 's' key to change the spline type between
Bezier splines, Catmull-Rom splines, and B-splines. Implement the remaining
two. These two support any number ($\ge 4$) of control points, by connecting a
sequence of cubic splines together. For example, if there are 6 control
points, the control points 0, 1, 2, and 3 define the first segment, the
control points 1, 2, 3, and 4 define the second segment, and finally, the
control points 2, 3, 4 and 5 define the third segment.

The basis matrix for the Catmull-Rom spline is

$$
B = \frac{1}{2}
\begin{pmatrix}
0 & -1 & 2 & -1\\
2 & 0 & -5 & 3\\
0 & 1 & 4 & -3\\
0 & 0 & -1 & 1
\end{pmatrix},
$$

<img src="images/image3.jpg" width="500px"/>

and the basis matrix for the B-spline is

$$
B = \frac{1}{6}
\begin{pmatrix}
1 & -3 & 3 & -1\\
4 & 0 & -6 & 3\\
1 & 3 & 3 & -3\\
0 & 0 & 0 & 1
\end{pmatrix}.
$$

<img src="images/image4.jpg" width="500px"/>

Note that Catmull-Rom splines are interpolating, whereas B-splines are
approximating. Both are local in the sense that changing a control point only
affects the curve locally and not globally.

Task 3
------

For the Catmull-Rom spline and the B-spline, draw the Frenet frame that moves
along the curve (shown in red, green, and blue in the figure below). The
tangent should be red, the normal should be green, and the binormal should be
blue.

$$
\begin{aligned}
p'(u) &= G B \bar{u}', \quad
\bar{u}' = \begin{pmatrix} 0 & 1 & 2u & 3u^2\end{pmatrix}^T,\\
p''(u) &= G B \bar{u}'', \quad
\bar{u}'' = \begin{pmatrix} 0 & 0 & 2 & 6u\end{pmatrix}^T,\\
T(u) &= \frac{p'(u)}{\|p'(u)\|},\\
B(u) &= \frac{p'(u) \times p''(u)}{\|p'(u) \times p''(u)\|},\\
N(u) &= B(u) \times T(u).
\end{aligned}
$$

The Frenet frame should travel along the curve using the global variable, `t`,
which is incremented using a timer. You may need to scale the three vectors
for display. You can use the following code to convert from `t` to `u`.

	float kfloat;
	float u = std::modf(std::fmod(t*0.5f, ncps-3.0f), &kfloat);
	int k = (int)std::floor(kfloat);

This code ensures that `u` stays within the range 0 to 1, and that `k` is the
appropriate index into the array of control points. The 0.5 implies that in
when t changes by 1.0, u changes by 0.5.

<img src="images/image5.jpg" width="500px"/>
