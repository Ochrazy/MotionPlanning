#include "Oberkoerper.h"
#include "Wuerfel.h"

Oberkoerper::Oberkoerper(void)
{
 
}

Oberkoerper::~Oberkoerper(void)
{
}

Oberkoerper::Oberkoerper(GLfloat groesse, GLfloat * farben)
{
    this->groesse = groesse;
}
void Oberkoerper::paint(GLfloat groesse, GLfloat * farben)
{
    	glPushMatrix();
	glTranslatef(0.0*groesse,0.7*groesse,0.0*groesse);
	glScalef(1.0*groesse,0.8*groesse,1.0*groesse);
	Wuerfel(groesse, farben[4],farben[5],farben[6],farben[7]);
	glPopMatrix();
	glPushMatrix();
	glScalef(0.8*groesse,0.6*groesse,0.8*groesse);
	Wuerfel(groesse, farben[8],farben[9],farben[10],farben[11]);
	glPopMatrix();
	glPushMatrix();
	glTranslatef(0.0*groesse,-0.5*groesse,0.0*groesse);
	glScalef(0.9*groesse,0.6*groesse,0.7*groesse);
	Wuerfel(groesse, farben[12],farben[13],farben[14],farben[15]);
	glPopMatrix();
}