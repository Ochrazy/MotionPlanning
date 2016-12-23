#pragma once
#include <GL/freeglut.h>
class Unterarm
{
public:
	Unterarm(void);
	Unterarm(GLfloat groesse, GLfloat * farben);
        void paint(GLfloat groesse, GLfloat * farben);
	~Unterarm(void);
        GLfloat groesse;
};

