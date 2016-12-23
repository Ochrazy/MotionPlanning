#pragma once
#include <GL\freeglut.h>
#include "Unterschenkel.h"
#include "Oberschenkel.h"
#include "Fuss.h"
class Bein
{
public:
	Bein(void);
	~Bein(void);
	void paint(GLfloat groesse, GLfloat* farben, GLfloat *coo, int lr);

	Oberschenkel oberschenkel;
	Unterschenkel unterschenkel;
	Fuss fuss;
};

