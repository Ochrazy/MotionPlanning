#include "Oberschenkel.h"
#include "Wuerfel.h"

Oberschenkel::Oberschenkel(void)
{
}

Oberschenkel::Oberschenkel(GLfloat groesse, GLfloat * farben)
{
    this->groesse = groesse;
}

Oberschenkel::~Oberschenkel(void)
{
}

void Oberschenkel::paint(GLfloat groesse, GLfloat* farben)
{

    glPushMatrix();

        glScalef(1.0*groesse,2.8*groesse,1.0*groesse);
	Wuerfel(groesse, farben[8],farben[9],farben[10],farben[11]);
	glPopMatrix();
}