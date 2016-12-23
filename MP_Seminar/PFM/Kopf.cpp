#include "Kopf.h"
#include "Wuerfel.h"
Kopf::Kopf(void)
{
}

Kopf::~Kopf(void)
{
}

Kopf::Kopf(GLfloat groesse, GLfloat *farben)
{
    this->groesse = groesse;
}

void Kopf::paint(GLfloat groesse, GLfloat *farben)
{
	glPushMatrix();
	glScalef(2.0*groesse,2.0*groesse,2.0*groesse);
	Wuerfel(groesse, farben[0],farben[1],farben[2],farben[3]);
	glPopMatrix();

}
