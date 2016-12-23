#include "Hand.h"
#include "Wuerfel.h"

Hand::Hand(void)
{
}

Hand::Hand(GLfloat groesse, GLfloat *farben)
{
    this->groesse = groesse;
}
void Hand::paint(GLfloat groesse, GLfloat* farben)
{
    	glPushMatrix();
	glScaled(0.5*groesse,0.8*groesse,0.5*groesse);
	Wuerfel(groesse, farben[0],farben[1],farben[2],farben[3]);
	glPopMatrix();
}

Hand::~Hand(void)
{
}
