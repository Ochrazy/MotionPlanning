#include "Unterschenkel.h"
#include "Wuerfel.h"
Unterschenkel::Unterschenkel(void)
{

}

Unterschenkel::Unterschenkel(GLfloat groesse, GLfloat * farben)
{
    this->groesse = groesse;

}

Unterschenkel::~Unterschenkel(void)
{
}

void Unterschenkel::paint(GLfloat groesse, GLfloat* farben)
{
    	glPushMatrix();
	glScalef(1.1*groesse,2.8*groesse,1.1*groesse);
	Wuerfel(groesse, farben[0],farben[1],farben[2],farben[3]);
	glPopMatrix();
}