#pragma once
#include <GL/freeglut.h>
#include "Oberarm.h"
#include "Unterarm.h"
#include "Hand.h"
class Arm
{
public:
	Arm(void);
	~Arm(void);
	Oberarm oberarm;
	Unterarm unterarm;
	Hand hand;

	void paint(GLfloat groesse, GLfloat* farben, GLfloat *coo, int lr);
};

