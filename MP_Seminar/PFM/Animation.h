#pragma once
#include <GL/freeglut.h>
class Animation
{
public:
	Animation(void);
	Animation(int kt, int tor, GLfloat x, GLfloat y, GLfloat z, GLfloat grad);
	~Animation();


	int kt_;
	int tor_;
	GLfloat x_;
	GLfloat y_;
	GLfloat z_;
	GLfloat grad_;
};

