#pragma once
#include <GL/freeglut.h>
class Unterschenkel
{
public:
	Unterschenkel(void);
	Unterschenkel(GLfloat groesse, GLfloat * farben);
        void paint(GLfloat groesse, GLfloat * farben);
	~Unterschenkel(void);
        GLfloat groesse;
};

