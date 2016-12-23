#pragma once
#include <GL/freeglut.h>
class Oberarm
{
public:
	Oberarm(void);
	Oberarm(GLfloat groesse, GLfloat * farben);
        void paint(GLfloat groesse, GLfloat * farben);
	~Oberarm(void);
        GLfloat groesse;
};

