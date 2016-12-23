#pragma once
#include <GL/freeglut.h>
class Oberkoerper
{
public:
	Oberkoerper(void);
	Oberkoerper(GLfloat groesse, GLfloat * farben);
        void paint(GLfloat groesse, GLfloat * farben);
	~Oberkoerper(void);
        GLfloat groesse;
};

