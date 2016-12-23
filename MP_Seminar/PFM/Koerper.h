#pragma once
#include <GL/freeglut.h>
class Koerper
{
public:
	Koerper(void);
	Koerper(GLfloat groesse, GLfloat * farben);
        void paint(GLfloat groesse, GLfloat * farben);
	~Koerper(void);
        GLfloat groesse;
};

