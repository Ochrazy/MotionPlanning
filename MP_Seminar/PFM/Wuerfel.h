#pragma once
#include <GL/freeglut.h>
class Wuerfel
{
public:
	Wuerfel(void);
	Wuerfel(GLfloat fSeitenL, GLfloat * farben);
	Wuerfel(GLfloat fSeitenL, GLfloat r,GLfloat g,GLfloat b,GLfloat a);
	~Wuerfel(void);
};

