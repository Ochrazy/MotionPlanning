#pragma once
#include <GL/freeglut.h>
class Hand
{
public:
	Hand(void);
	Hand(GLfloat groesse, GLfloat *farben);
        void paint(GLfloat groesse, GLfloat *farben);
	~Hand(void);
        GLfloat groesse;
};

