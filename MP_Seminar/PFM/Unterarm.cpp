#include "Unterarm.h"
#include "Hand.h"
#include "Wuerfel.h"

Unterarm::Unterarm(void)
{
    
}

Unterarm::Unterarm(GLfloat groesse, GLfloat * farben)
{
     this->groesse = groesse;
	
}

Unterarm::~Unterarm(void)
{
}

void Unterarm::paint(GLfloat groesse, GLfloat* farben)
{
	glPushMatrix();
	glScalef(1.0*groesse,1.9*groesse,1.0*groesse);
	Wuerfel(groesse, farben[4],farben[5],farben[6],farben[7]);
	glPopMatrix();
}
