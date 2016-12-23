#include "Roboter.h"

Roboter::Roboter(void) {
	counter = 0;
	coo[0*7+0] = 0;
	coo[0*7+1] = 4;
	coo[0*7+2] = 0;
	coo[0*7+3] = 0;
	coo[0*7+4] = 0;
	coo[0*7+5] = 0;
	coo[0*7+6] = 0;
	//kopf
	coo[2*7+0] = 0;
	coo[2*7+1] = 4;
	coo[2*7+2] = 0;
	coo[2*7+3] = 0;
	coo[2*7+4] = 0;
	coo[2*7+5] = 0;
	coo[2*7+6] = 0;

	//oberkörper   
	coo[1*7+0] = 0;
	coo[1*7+1] = -2.7;
	coo[1*7+2] = 0;
	coo[1*7+3] = 0;
	coo[1*7+4] = 0;
	coo[1*7+5] = 0;
	coo[1*7+6] = 0;

	//oberarm-l
	coo[3*7+0] = -2;
	coo[3*7+1] = 3;
	coo[3*7+2] = 0;
	coo[3*7+3] = 0;
	coo[3*7+4] = 0;
	coo[3*7+5] = 0;
	coo[3*7+6] = 0;
	//oberarm-r
	coo[6*7+0] = 2;
	coo[6*7+1] = 3;
	coo[6*7+2] = 0;
	coo[6*7+3] = 0;
	coo[6*7+4] = 0;
	coo[6*7+5] = 0;
	coo[6*7+6] = 0;
	//unterarm-l
	coo[4*7+0] = 0;
	coo[4*7+1] = -1.1;
	coo[4*7+2] = 0;
	coo[4*7+3] = 0;
	coo[4*7+4] = 0;
	coo[4*7+5] = 0;
	coo[4*7+6] = 0;
	//unterarm-r
	coo[7*7+0] = 0;
	coo[7*7+1] = -1.1;
	coo[7*7+2] = 0;
	coo[7*7+3] = 0;
	coo[7*7+4] = 0;
	coo[7*7+5] = 0;
	coo[7*7+6] = 0;
	//hand-l
	coo[5*7+0] = 0;
	coo[5*7+1] = -1.2;
	coo[5*7+2] = 0;
	coo[5*7+3] = 0;
	coo[5*7+4] = 0;
	coo[5*7+5] = 0;
	coo[5*7+6] = 0;
	//hand-r
	coo[8*7+0] = 0;
	coo[8*7+1] = -1.2;
	coo[8*7+2] = 0;
	coo[8*7+3] = 0;
	coo[8*7+4] = 0;
	coo[8*7+5] = 0;
	coo[8*7+6] = 0;
	//oberschenkel-l
	coo[9*7+0] = -0.7;
	coo[9*7+1] = -2.5;
	coo[9*7+2] = 0;
	coo[9*7+3] = 0;
	coo[9*7+4] = 0;
	coo[9*7+5] = 0;
	coo[9*7+6] = 0;
	//oberschenkel-r
	coo[12*7+0] = 0.7;
	coo[12*7+1] = -2.5;
	coo[12*7+2] = 0;
	coo[12*7+3] = 0;
	coo[12*7+4] = 0;
	coo[12*7+5] = 0;
	coo[12*7+6] = 0;
	//unterschenkel-l
	coo[10*7+0] = 0;
	coo[10*7+1] = -2;
	coo[10*7+2] = 0;
	coo[10*7+3] = 0;
	coo[10*7+4] = 0;
	coo[10*7+5] = 0;
	coo[10*7+6] = 0;
	//unterschenkel-r
	coo[13*7+0] = 0;
	coo[13*7+1] = -2;
	coo[13*7+2] = 0;
	coo[13*7+3] = 0;
	coo[13*7+4] = 0;
	coo[13*7+5] = 0;
	coo[13*7+6] = 0;
	//fuss-l
	coo[11*7+0] = 0;
	coo[11*7+1] = -2;
	coo[11*7+2] = 0;
	coo[11*7+3] = 0;
	coo[11*7+4] = 0;
	coo[11*7+5] = 0;
	coo[11*7+6] = 0;
	//fuss-r
	coo[14*7+0] = 0;
	coo[14*7+1] = -2;
	coo[14*7+2] = 0;
	coo[14*7+3] = 0;
	coo[14*7+4] = 0;
	coo[14*7+5] = 0;
	coo[14*7+6] = 0;
}

void Roboter::init(GLfloat groesse, GLfloat * farben, vector <vector<Animation> > anima) {
	anima_ = anima;
	this->farben = farben;
}

Roboter::~Roboter(void) {
}

void Roboter::paint() {

	// Roboter
	glPushMatrix();
	glTranslatef(coo[0*7+0], coo[0*7+1], coo[0*7+2]);
	glRotatef(coo[0*7+6],coo[0*7+3], coo[0*7+4], coo[0*7+5]);
	// Körper
	glPushMatrix();
	glTranslatef(coo[1*7+0], coo[1*7+1], coo[1*7+2]);
	glRotatef(coo[1*7+6],coo[1*7+3], coo[1*7+4], coo[1*7+5]);
	glTranslatef(0.0,+2.7,0.0);
	koerper.paint(1.0, farben);
	// Kopf
	glPushMatrix();
	glTranslatef(coo[2*7+0], coo[2*7+1], coo[2*7+2]);
	glRotatef(coo[2*7+6],coo[2*7+3], coo[2*7+4], coo[2*7+5]);
	kopf.paint(1.0, farben);
	glPopMatrix();

	// Arm links
	armL.paint(1.0, farben, coo, 1);

	// Arm rechts
	armR.paint(1.0, farben, coo,0);

	glPopMatrix();

	// Bein links

	beinL.paint(1.0, farben, coo,1);

	// Bein rechts
	beinR.paint(1.0, farben, coo,0);

	glPopMatrix();


}

void Roboter::calculate() {
	int check = 0;
	if (!anima_.empty()) {
		if (counter + 1 <= anima_.size()) {

			for (int i = 0; i < anima_[counter].size(); i++) {// alle Körperteile updaten in forschleife
				
				if(updatePart(anima_[counter][i].kt_, i))
				{
					check++;   
				}
			}
			if(check ==0)
			{
			
				counter++;
				
			}
		}
	}
}

bool Roboter::updatePart(int part, int sequence) {

	if ((anima_[counter][sequence].tor_ == 0) && 
		((anima_[counter][sequence].x_ - 0.1 >= coo[part*7+0] ||
		anima_[counter][sequence].y_ - 0.1 >= coo[part*7+1] ||
		anima_[counter][sequence].z_ - 0.1 >= coo[part*7+2])||
		(anima_[counter][sequence].x_ + 0.1 <= coo[part*7+0] || 
		anima_[counter][sequence].y_ + 0.1 <= coo[part*7+1] ||
		anima_[counter][sequence].z_ + 0.1 <= coo[part*7+2]))) {
		coo[part*7+0] = coo[part*7+0] + speed * (anima_[counter][sequence].x_ - coo[part*7+0]);
		coo[part*7+1] = coo[part*7+1] + speed * (anima_[counter][sequence].y_ - coo[part*7+1]);
		coo[part*7+2] = coo[part*7+2] + speed * (anima_[counter][sequence].z_ - coo[part*7+2]);
		return true;
	}
	else if ((anima_[counter][sequence].tor_ == 1) &&
		((anima_[counter][sequence].grad_ - 0.1 >= coo[part*7+6]) ||
		(anima_[counter][sequence].grad_ + 0.1 <= coo[part*7+6]))) {
		coo[part*7+3] = anima_[counter][sequence].x_;
		coo[part*7+4] = anima_[counter][sequence].y_;
		coo[part*7+5] = anima_[counter][sequence].z_;
		coo[part*7+6] = coo[part*7+6] + speed * (anima_[counter][sequence].grad_ - coo[part*7+6]);
		return true;
	}
	return false;
}

GLfloat * Roboter::getPosition()
{
	GLfloat pos[3];
	pos[0] =  coo[0*7+0];
	pos[1] =	coo[0*7+1];
	pos[2] =coo[0*7+2];
	return pos;
}