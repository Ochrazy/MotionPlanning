#pragma once
#include <GL/freeglut.h>
class Fuss
{
public:
	Fuss(void);
	Fuss(GLfloat groesse, GLfloat *farben);
        void paint(GLfloat groesse, GLfloat * farben);
	~Fuss(void);
        GLfloat groesse;
};

