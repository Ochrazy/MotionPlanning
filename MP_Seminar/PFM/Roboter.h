#pragma once
#include <GL/freeglut.h>
#include "Animation.h"
#include <vector>
#include <string>
using namespace std;
#include "Arm.h"
#include "Bein.h"
#include "Kopf.h"
#include "Koerper.h"
#include "Oberarm.h"
#include "Unterarm.h"
#include "Fuss.h"
#include "Hand.h"
#include "Oberschenkel.h"
#include "Unterschenkel.h"
#define speed 0.05

/* Szenengraph
*	0.) Roboter
*	1.)	Körper+Kopf+Arme
*	2.) Kopf
*	3.) Linker Arm
*	4.) Linker Unterarm + Linke Hand
*	5.) Linke Hand
*	6.) Rechter Arm
*	7.) Rechter Unterarmv+ Rechte Hand
*	8.) Rechte Hand
*	9.) Linkes Bein
*	10.) Linker Unterschenkel + Linker Fuss
*	11.) Linker Fuss
*	12.) Rechtes Bein
*	13.) Rechter Unterschenkel + Rechter Fuss
*	14.) Rechter Fuss
*
*	Parameter Linke Seite = 1 Rechte Seite = 0
*	Parameter Translation = 0 Rotation = 0
*
*/



class Roboter {
public:
    Roboter(void);
    ~Roboter(void);
    void init(GLfloat groesse, GLfloat* farben, vector <vector<Animation> > anima);
    void paint();
    void calculate();
    bool updatePart(int part, int sequence);
	void beinHoch();
	GLfloat * getPosition();
    vector <vector<Animation> > anima_;
    GLfloat *farben;
    int counter;
    Kopf kopf;
    Koerper koerper;
	Arm armL;
	Arm armR;
	Bein beinL;
	Bein beinR;

    GLfloat coo[105];
};

