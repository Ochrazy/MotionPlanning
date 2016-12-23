#pragma once
#include <GL/freeglut.h>
class Oberschenkel
{
public:
	Oberschenkel(void);
	Oberschenkel(GLfloat groesse, GLfloat * farben);
        void paint(GLfloat groesse, GLfloat * farben);
	~Oberschenkel(void);
        GLfloat groesse;
};

