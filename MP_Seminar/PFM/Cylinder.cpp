#include "Cylinder.h"
#include <GL/freeglut.h>
#define _USE_MATH_DEFINES
#include <cmath>


Cylinder::Cylinder(float height, float radius)
{
	float halfLength = height / 2;
	int slices;
	for (int i = 0; i<slices; i++) {
		float theta = ((float)i)*2.0*M_PI;
		float nextTheta = ((float)i + 1)*2.0*M_PI;
		glBegin(GL_TRIANGLE_STRIP);
		/*vertex at middle of end */ glVertex3f(0.0, halfLength, 0.0);
		/*vertices at edges of circle*/ glVertex3f(radius*cos(theta), halfLength, radius*sin(theta));
		glVertex3f(radius*cos(nextTheta), halfLength, radius*sin(nextTheta));
		/* the same vertices at the bottom of the cylinder*/
		glVertex3f(radius*cos(nextTheta), -halfLength, radius*sin(nextTheta));
		glVertex3f(radius*cos(theta), -halfLength, radius*sin(theta));
		glVertex3f(0.0, -halfLength, 0.0);
		glEnd();
	}
}


Cylinder::~Cylinder()
{
}
