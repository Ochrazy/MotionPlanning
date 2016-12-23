#include "Fuss.h"
#include "Wuerfel.h"


Fuss::Fuss(void)
{
}

Fuss::Fuss(GLfloat groesse, GLfloat *farben)
{
    this->groesse = groesse;
}
void Fuss::paint(GLfloat groesse, GLfloat* farben)
{
    glPushMatrix();
	glScalef(1.0*groesse,0.4*groesse,2.0*groesse);
	Wuerfel(groesse, farben[0],farben[1],farben[2],farben[3]);
	glPopMatrix();
}

Fuss::~Fuss(void)
{
}
