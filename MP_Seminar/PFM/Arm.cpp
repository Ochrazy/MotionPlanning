#include "Arm.h"


Arm::Arm(void)
{
}


Arm::~Arm(void)
{
}


void Arm::paint(GLfloat groesse, GLfloat* farben, GLfloat *coo, int lr)
{	
	// Arm Links
	if(lr==1)
	{
		// Oberarm + Unterarm + Hand
		glPushMatrix();
		glTranslatef(coo[3*7+0], coo[3*7+1], coo[3*7+2]);
		glRotatef(coo[3*7+6],coo[3*7+3], coo[3*7+4], coo[3*7+5]);
		glTranslatef(0.0,-2.05,0.0);
		oberarm.paint(1.0, farben);
		// Unterarm + Hand
		glPushMatrix();
		glTranslatef(coo[4*7+0], coo[4*7+1], coo[4*7+2]);
		glRotatef(coo[4*7+6],coo[4*7+3], coo[4*7+4], coo[4*7+5]);
		glTranslatef(0.0,-0.8,0.0);
		unterarm.paint(1.0, farben);
		// Hand
		glPushMatrix();
		glTranslatef(coo[5*7+0], coo[5*7+1], coo[5*7+2]);
		glRotatef(coo[5*7+6],coo[5*7+3], coo[5*7+4], coo[5*7+5]);
		glTranslatef(0.0,-0.4,0.0);
		hand.paint(1.0, farben);
		glPopMatrix();
		glPopMatrix();
		glPopMatrix();

	
	}
	else // Arm Rechts
	{
	
		// Oberarm + Unterarm + Hand
		glPushMatrix();
		glTranslatef(coo[6*7+0], coo[6*7+1], coo[6*7+2]);
		glRotatef(coo[6*7+6],coo[6*7+3], coo[6*7+4], coo[6*7+5]);
		glTranslatef(0.0,-2.05,0.0);
		oberarm.paint(1.0, farben);
		// Unterarm + Hand
		glPushMatrix();
		glTranslatef(coo[7*7+0], coo[7*7+1], coo[7*7+2]);
		glRotatef(coo[7*7+6],coo[7*7+3], coo[7*7+4], coo[7*7+5]);
		glTranslatef(0.0,-0.8,0.0);
		unterarm.paint(1.0, farben);
		// Hand
		glPushMatrix();
		glTranslatef(coo[8*7+0], coo[8*7+1], coo[8*7+2]);
		glRotatef(coo[8*7+6],coo[8*7+3], coo[8*7+4], coo[8*7+5]);
		glTranslatef(0.0,-0.4,0.0);
		hand.paint(1.0, farben);
		glPopMatrix();
		glPopMatrix();
		glPopMatrix();
	
	}
	
}
