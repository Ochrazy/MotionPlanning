#include "Cylinder.h"
#include <GL/freeglut.h>
#define _USE_MATH_DEFINES
#include <cmath>

Cylinder::Cylinder()
{
	center = Point(0., 0., 0.);
	r = 1.;
	h = 1.;
}

Cylinder::Cylinder(const Point p, double pr = 1.0, double ph = 1.0) : center(p), r(pr), h(ph)
{
	/*
	float halfLength = height / 2;
	int slices;
	for (int i = 0; i<slices; i++) {
		float theta = ((float)i)*2.0*M_PI;
		float nextTheta = ((float)i + 1)*2.0*M_PI;
		glBegin(GL_TRIANGLE_STRIP);
		//vertex at middle of end
		glVertex3f(0.0, halfLength, 0.0);
		//vertices at edges of circle
		glVertex3f(radius*cos(theta), halfLength, radius*sin(theta));
		glVertex3f(radius*cos(nextTheta), halfLength, radius*sin(nextTheta));
		// the same vertices at the bottom of the cylinder
		glVertex3f(radius*cos(nextTheta), -halfLength, radius*sin(nextTheta));
		glVertex3f(radius*cos(theta), -halfLength, radius*sin(theta));
		glVertex3f(0.0, -halfLength, 0.0);
		glEnd();
	}
	*/
}


Cylinder::~Cylinder()
{
}

Point& Cylinder::GetCenter()
{
	return center;
}

void Cylinder::SetCenter(const Point p)
{
	center = p;
}

void Cylinder::SetCenter(double x, double y, double z)
{
	// sets the center coordinates to x,y,z
	center.x = x;
	center.y = y;
	center.z = z;
}

void Cylinder::SetRadius(const double pr)
{
	// sets the radius
	r = pr;
}

double Cylinder::GetRadius()
{
	// gets the radius
	return r;
}

double Cylinder::distance(Cylinder cyl2, Point *pt)
{
	*pt = cyl2.center - center;
	double d = pt->Magnitude() - r - cyl2.r;
	pt->SetLength(d);

	return d;
}

double Cylinder::distance_sqr(Cylinder cyl2)
{
	Point d = center.Sub(cyl2.center);
	return d.SquareMagnitude() - r*r - cyl2.r*cyl2.r;
}


void Cylinder::Translate(double x, double y, double z)
{
	center.x += x;
	center.y += y;
	center.z += z;
}

void Cylinder::SetRepulsivness(double rep)
{
	repulsivness = rep;
}
double Cylinder::GetRepulsivness(void)
{
	return repulsivness;
}