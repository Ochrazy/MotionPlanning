#pragma once
#include <GL/freeglut.h>
#include "Animation.h"
class Kopf
{
public:
	Kopf(void);
	Kopf(GLfloat groesse, GLfloat *farben);
        void paint(GLfloat groesse, GLfloat *farben);
	~Kopf(void);
        GLfloat groesse;
        

};

