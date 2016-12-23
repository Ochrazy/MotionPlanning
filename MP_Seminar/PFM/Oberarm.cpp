#include "Oberarm.h"
#include "Wuerfel.h"


Oberarm::Oberarm(void)
{
}
Oberarm::Oberarm(GLfloat groesse, GLfloat *farben)
{
	    this->groesse = groesse;
}
void Oberarm::paint(GLfloat groesse, GLfloat * farben)
{
        glPushMatrix();
	glTranslatef(0.0*groesse,1.4*groesse,0.0*groesse);
	glScalef(1.0*groesse,1.6*groesse,1.0*groesse);
	Wuerfel(groesse, farben[4],farben[5],farben[6],farben[7]);
	glPopMatrix();
	glPushMatrix();
	glScalef(0.8*groesse,1.3*groesse,0.8*groesse);
	Wuerfel(groesse, farben[8],farben[9],farben[10],farben[11]);
	glPopMatrix();
}

Oberarm::~Oberarm(void)
{
}
