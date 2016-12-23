#include "Koerper.h"
#include "Wuerfel.h"

Koerper::Koerper(void)
{
 
}

Koerper::~Koerper(void)
{
}

Koerper::Koerper(GLfloat groesse, GLfloat * farben)
{
    this->groesse = groesse;
}
void Koerper::paint(GLfloat groesse, GLfloat * farben)
{
    	glPushMatrix();
	glTranslatef(0.0*groesse,2.0*groesse,0.0*groesse);
	glScalef(3.0*groesse,2.4*groesse,2.0*groesse);
	Wuerfel(groesse, farben[4],farben[5],farben[6],farben[7]);
	glPopMatrix();
	glPushMatrix();
	glScalef(2.4*groesse,1.8*groesse,1.8*groesse);
	Wuerfel(groesse, farben[8],farben[9],farben[10],farben[11]);
	glPopMatrix();
	glPushMatrix();
	glTranslatef(0.0*groesse,-1.8*groesse,0.0*groesse);
	glScalef(2.7*groesse,1.8*groesse,1.6*groesse);
	Wuerfel(groesse, farben[12],farben[13],farben[14],farben[15]);
	glPopMatrix();
}