#pragma once

#include <vector>
#include <GL/freeglut.h>  //laedt auch glut.h und gl.h

#include "Animation.h"
#include "Roboter.h"
#include "SkyBox.h"
#include "Wuerfel.h"

#include "Cylinder.h"
#include "Point.h"
#include "Potential.h"

const int nRob = 4;     // Anzahl der Roboterglieder
const int nHind = 0 + nRob -1;    // Anzahl der Hindernisse
int total_counter[nRob];


Roboter roboter;
Wuerfel box;
SkyBox skybox;
vector<vector<Animation> > animationen;

int animationUpdateInMS = 16;
int width = 1024, height = 768;
GLfloat mass = 1.0; // Mass fuer die Ausdehnung des Modells

GLUquadricObj *quadratic[nRob];
Point robPos[nRob];
bool goal_reached[nRob];
bool local_minimum_reached[nRob];

//const double ds = 0.01f;

vector<Point> path[nRob];     // Create a vector containing integers
Cylinder aHindernis[nHind];  // Unsere Hindernisse
Cylinder roboterPot[nRob];      // Roboter
float radius[nRob];

Potential pot[nRob];

GLfloat farben[] = {
	1.0, 0.0, 0.0, 1.0, // ROT
	0.0, 1.0, 0.0, 1.0, // GRÜN
	0.0, 0.0, 1.0, 1.0, // BLAU
	1.0, 1.0, 0.0, 1.0, // ROT/GRÜN
	1.0, 0.0, 1.0, 1.0, // ROT/BLAU
	0.0, 1.0, 1.0, 1.0, // GRÜN/BLAU
	1.0, 1.0, 1.0, 1.0, // WEISS
	0.5, 0.5, 0.5, 1.0  // GRAU
};

struct Camera {
	GLdouble xpos, ypos, zpos, xrot, yrot, angle, xeye, yeye, zeye, lux, luy, luz;
	void camera()
	{
		xpos = 0, ypos = 10, zpos = 15, xrot = 0, yrot = 0, angle = 0, xeye = 0, yeye = 0, zeye = 0, lux = 0, luy = 1, luz = 0;
	}
} camera;

struct Keys
{
	int right, left, xclick, yclick, xmouse, ymouse, zoom, yaw, pitch;
	void keys()
	{
		right = 0, left = 0, xclick = 0, yclick = 0, xmouse = 0, ymouse = 0, zoom = 0, yaw = 0, pitch = 0;
	}
} keys;