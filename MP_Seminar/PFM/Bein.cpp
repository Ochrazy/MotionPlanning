#include "Bein.h"


Bein::Bein(void)
{
}


Bein::~Bein(void)
{
}


void Bein::paint(GLfloat groesse, GLfloat* farben, GLfloat *coo, int lr)
{
	if(lr==1)
	{
		glPushMatrix();
		glTranslatef(coo[9*7+0], coo[9*7+1], coo[9*7+2]);
		glRotatef(coo[9*7+6],coo[9*7+3], coo[9*7+4], coo[9*7+5]);
		glTranslatef(0.0,-1.4,0.0);
		oberschenkel.paint(1.0, farben);
    

		glPushMatrix();
		glTranslatef(coo[10*7+0], coo[10*7+1], coo[10*7+2]);
		glRotatef(coo[10*7+6],coo[10*7+3], coo[10*7+4], coo[10*7+5]);
		glTranslatef(0.0,-1.4,0.0);
		unterschenkel.paint(1.0, farben);

		glPushMatrix();
		glTranslatef(coo[11*7+0], coo[11*7+1], coo[11*7+2]);
		glRotatef(coo[11*7+6],coo[11*7+3], coo[11*7+4], coo[11*7+5]);
		glTranslatef(0.0,-0.2,0.0);
		fuss.paint(1.0, farben);
		glPopMatrix();
		glPopMatrix();
		glPopMatrix();
	}
	else
	{
		glPushMatrix();
		glTranslatef(coo[12*7+0], coo[12*7+1], coo[12*7+2]);
		glRotatef(coo[12*7+6],coo[12*7+3], coo[12*7+4], coo[12*7+5]);
		glTranslatef(0.0,-1.4,0.0);
		oberschenkel.paint(1.0, farben);
    

		glPushMatrix();
		glTranslatef(coo[13*7+0], coo[13*7+1], coo[13*7+2]);
		glRotatef(coo[13*7+6],coo[13*7+3], coo[13*7+4], coo[13*7+5]);
		glTranslatef(0.0,-1.4,0.0);
		unterschenkel.paint(1.0, farben);

		glPushMatrix();
		glTranslatef(coo[14*7+0], coo[14*7+1], coo[14*7+2]);
		glRotatef(coo[14*7+6],coo[14*7+3], coo[14*7+4], coo[14*7+5]);
		glTranslatef(0.0,-0.2,0.0);
		fuss.paint(1.0, farben);
		glPopMatrix();
		glPopMatrix();
		glPopMatrix();
	}



}