/* -----------------------------------------------------------	*/
/* Graph.-DV-Praktikum Startprogramm                           */
/* -----------------------------------------------------------	*/
/* Datei: main.cpp          Barlyschew, Porada                   */
/* -----------------------------------------------------------	*/

#include <iostream>      
#include <GL/freeglut.h>  //laedt auch glut.h und gl.h
#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include "Animation.h"
#include "Roboter.h"
#include "Wuerfel.h"

#include "SkyBox.h"
#include <Windows.h>

using namespace std;

Roboter roboter;
Wuerfel box;
SkyBox skybox;
vector<vector<Animation> > animationen;

int animationUpdateInMS = 5;
int width = 1024, height = 768;
GLfloat mass = 1.0; // Mass fuer die Ausdehnung des Modells



GLfloat farben[] = { 0.0, 0.0, 1.0, 1.0, //Kopf
1.0, 0.0, 0.0, 1.0, //Oberkörper1
1.0, 1.0, 1.0, 1.0, //Oberkörper2
0.5, 0.5, 0.5, 1.0//Oberkörper3
};

struct Camera{
	GLdouble xpos , ypos , zpos , xrot , yrot , angle, xeye, yeye, zeye, lux, luy, luz;
	void camera()
	{
		xpos = 0 , ypos = 0 , zpos = 15 , xrot = 0 , yrot = 0 , angle = 0, xeye  = 0, yeye = 0, zeye = 0, lux = 0, luy = 1, luz = 0;
	}
} camera;

struct Keys
{
	int right, left, xclick , yclick, xmouse, ymouse , zoom,yaw,pitch;
	void keys()
	{
		right = 0, left = 0, xclick = 0, yclick = 0, xmouse = 0, ymouse = 0, zoom = 0, yaw = 0, pitch = 0;
	}
} keys;


void MouseFunc(int, int, int, int); // Maus-Tasten und -Bewegung abfragen
void MouseMotion(int, int); // Maus-Bewegungen mit gedrückter Maus-Taste
void PassivMouseMotion(int, int); // Maus-Bewegungen ohne gedrückte Maus-Taste
void SpecialFunc(int, int, int); // Funktions- und Pfeil-Tasten abfragen
void menue();



void Init() {
	// Hier finden jene Aktionen statt, die zum Programmstart einmalig 
	// durchgefuehrt werden muessen
	glClearColor(0.33f, 0.225f, 0.0f, 1.0f); // Hintergrundfarbe definieren 
	glEnable(GL_DEPTH_TEST); //Aktiviert eine server-seitige F�higkeit. GL_DEPTH_TEST ist die F�higkeit Tiefenvergleiche zu t�tigen und der Tiefenpuffer aktualisiert
	glClearDepth(1.0); // legt den Wert fest, mit dem der Tiefenpuffer beim L�schen des selbigen gef�llt wird. 1.0 ist das maximum 0.0 ist das minimum
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_NORMALIZE); // Farbverlaeufe glaetten    
	GLfloat mat_specular[] = {0.5f, 0.5f, 0.5f, 0.5f}; // Array fuer Glanz-Richtung
	GLfloat mat_shininess[] = {50.0f}; // Array fuer die Leucht-Intensitaet
	GLfloat white_light[] = {1.0f, 1.0f, 1.0f, 1.0f}; // Array fuer die Farbe des Lichts
	GLfloat ambientLight[] = {1.0f, 1.0f, 1.0f, 0.0f}; // Array fuer die Farbe des Ambient-Lichts
	glLightfv(GL_LIGHT0, GL_SPECULAR, white_light); // Farbe und Art des Licht an die Lichtquelle binden
	glLightfv(GL_LIGHT0, GL_DIFFUSE, white_light); // Farbe und Art des Licht an die Lichtquelle binden
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular); // Farbe und Art des Licht an das Material binden
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess); // Farbe und Art des Licht an das Material binden
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambientLight); // Farbe und Art des Licht an die Scene binden
	//	glGenTextures(2, texture_id);							// Create Two Textures / Textur-Namen anlegen
	//	Image *image1 = Load_BMP("erde_texture.bmp");			// BMP-Bild laden, prüfen und Bitmap erzeugen
	//    Load_Texture(image1,0);									// Convert To A Texture
	//	delete image1;											// Image Objekt loeschen
	//	Image *image2 = Load_BMP("mond_texture.bmp");			// BMP-Bild laden, prüfen und Bitmap erzeugen
	//	Load_Texture(image2,1);									// Convert To A Texture
	//	delete image2;											// Image Objekt loeschen
	//	Image *image3 = Load_BMP("wolken_texture.bmp");			// BMP-Bild laden, prüfen und Bitmap erzeugen
	//	Load_Texture(image3,2);									// Convert To A Texture
	//	delete image3;											// Image Objekt loeschen
	//skybox.SkyboxInit();
	//roboter.init(mass, farben, animationen);
	
	
}

void RenderScene(void) {
	// Hier befindet sich der Code der in jedem frame ausgefuehrt werden muss
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Puffer loeschen
	glLoadIdentity(); //Ersetzt die aktuelle Matrix durch die Einheitsmatrix
	gluLookAt(camera.xpos, camera.ypos, camera.zpos,
		camera.xeye, camera.yeye, camera.zeye,
		camera.lux, camera.luy, camera.luz); // Kamera von oben
	glPushMatrix();
	
	//skybox.paint();
	//roboter.paint();
	
	GLUquadricObj *quadratic;
	quadratic = gluNewQuadric();
	glPushMatrix();
	glRotatef(90.0f, 1.0f, 0.0f, 0.0f);
	glTranslatef(0.0f, 0.0f, -1.5f);
	gluCylinder(quadratic, 0.2f, 0.2f, 1.5f, 32, 32);
	glPopMatrix();
	glScalef(10.0*mass, 0.1*mass, 10.0*mass);
	Wuerfel(mass, farben[0], farben[1], farben[2], farben[3]);
	glPopMatrix();
	glutSwapBuffers();

}

void Reshape(int width, int height) {
	// Hier finden die Reaktionen auf eine Veraenderung der Groesse des 
	// Graphikfensters statt
	glMatrixMode(GL_PROJECTION); // Matrix f�r Transf. Frustum->viewport
	glLoadIdentity(); //Ersetzt die aktuelle Matrix durch die Einheitsmatrix
	glViewport(0, 0, width, height); //glViewport Beschreibt das aktuelle Betrachtungsfenster. 
	//    glOrtho(-mass * 44.0, +mass * 44.0, -mass * 44.0, +mass * 44.0, 0.0, 14.0 * mass); // definiert das Frustum und die parameter sind wie folgt definiert (links, rechts, unten, oben, vodere und hintere begrenzung) 
	gluPerspective(45.0, width/height , 0.001, 1130); // definiert das Frustum und die parameter sind wie folgt definiert (links, rechts, unten, oben, vodere und hintere begrenzung) 

	glMatrixMode(GL_MODELVIEW); // Modellierungs/Viewing-Matrix

}

void Animate(int value) {
	// Hier werden Berechnungen durchgefuehrt, die zu einer Animation der Szene  
	// erforderlich sind. Dieser Prozess laeuft im Hintergrund und wird alle 
	// 1000 msec aufgerufen. Der Parameter "value" wird einfach nur um eins 
	// inkrementiert und dem Callback wieder uebergeben. 
	//std::cout << "value=" << value << std::endl; // Ausgabe auf der Console
	// RenderScene aufrufen.
	roboter.calculate();
	glutPostRedisplay();
	// Timer wieder registrieren - Animate wird so nach 100 msec mit value+=1 aufgerufen.
	glutTimerFunc(animationUpdateInMS, Animate, ++value);




}


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


int main(int argc, char **argv) {
	camera.camera();
	keys.keys();

	vector<Animation> steps;
	steps.push_back(Animation(0, 0, 0.0, 5.0, 0.0, 0.0));
	steps.push_back(Animation(0, 1, 0.0, 1.0, 0.0, 180.0));
	steps.push_back(Animation(3, 1, 0.0, 0.0, 1.0, -90.0));
	steps.push_back(Animation(6, 1, 0.0, 0.0, 1.0, +90.0));
	animationen.push_back(steps);
	steps.clear();
	//steps.push_back(Animation(6,1,0.0,0.0,1.0,45));
	//steps.push_back(Animation(7,1,0.0,0.0,1.0,-45));
	//steps.push_back(Animation(3,1,0.0,0.0,1.0,-45));
	//steps.push_back(Animation(4,1,0.0,0.0,1.0,+45));
	//animationen.push_back(steps);
	//steps.clear();
	//steps.push_back(Animation(6,1,0.0,0.0,1.0,90));
	//steps.push_back(Animation(7,1,0.0,0.0,1.0,-90));
	//steps.push_back(Animation(3,1,0.0,0.0,1.0,-90));
	//steps.push_back(Animation(4,1,0.0,0.0,1.0,+90));
	//animationen.push_back(steps);
	//steps.clear();
	//steps.push_back(Animation(7,1,0.0,0.0,1.0,0));
	//steps.push_back(Animation(4,1,0.0,0.0,1.0,0));
	//animationen.push_back(steps);
	//steps.clear();
	//steps.push_back(Animation(0,1,0.0,1.0,0.0,180));
	//animationen.push_back(steps);
	//steps.clear();
	//steps.push_back(Animation(0,1,0.0,1.0,0.0,360));
	//steps.push_back(Animation(6,1,0.0,0.0,1.0,90));
	//steps.push_back(Animation(7,1,0.0,0.0,1.0,-90));
	//steps.push_back(Animation(3,1,0.0,0.0,1.0,-90));
	//steps.push_back(Animation(4,1,0.0,0.0,1.0,+90));
	//animationen.push_back(steps);
	//steps.clear();
	
	menue();
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH); //
	glutInitWindowSize(width, height); // Die Fenstergr��e wird initialisiert
	glutCreateWindow("Multi-Robot Motion Planning");
	glutDisplayFunc(RenderScene);
	glutReshapeFunc(Reshape);
	
	Init();

	glutMouseFunc(MouseFunc); // Maus-Tasten und -Bewegung abfragen
	glutMotionFunc(MouseMotion); // Maus-Bewegungen mit gedrückter Maus-Taste (AUS)
	glutPassiveMotionFunc(PassivMouseMotion); // Maus-Bewegungen ohne gedrückte Maus-Taste (AUS)
	glutSpecialFunc(SpecialFunc); // Funktion für Sondertasten (F1...F12++)
	
    // TimerCallback registrieren; wird nach 10 msec aufgerufen mit Parameter 0  
	glutTimerFunc(animationUpdateInMS, Animate, 0);
	glutMainLoop();
	return 0;
}

void MouseFunc(int button, int state, int x, int y) { // Maus-Tasten und -Bewegung abfragen
	switch (button) { // Reaktion auf gedrueckte bzw. losgelassene Maustasten
	case GLUT_LEFT_BUTTON: 
		if (state == GLUT_DOWN)
		{
			keys.left = 1;
			keys.yclick=y;
			keys.xclick=x;
			cout << "X:"  << x << "Y:" << y << endl;	
		}
		else if(state == GLUT_UP)
		{
			keys.left = 0;
			cout << "X:"  << x << "Y:" << y << endl;
		}
		break;
	case GLUT_RIGHT_BUTTON:
		if (state == GLUT_DOWN)
		{
			keys.right = 1;
			keys.yclick=y;
			keys.xclick=x;
			cout << "X:"  << x << "Y:" << y << endl;	
		}
		else if(state == GLUT_UP)
		{
			keys.right = 0;
			cout << "X:"  << x << "Y:" << y << endl;
		}
		break;
	default: break;
	}
}

void MouseMotion(int x, int y) { // Maus-Bewegungen mit gedrückter Maus-Taste
	if (keys.right && keys.left)
	{	

	} 
	else if(keys.right)
	{
		
		keys.zoom = (keys.ymouse-y);
		if(keys.zoom < 0)
		{
			camera.zpos++;

		} else if(keys.zoom > 0)
		{
			camera.zpos--;
		}
		keys.ymouse = y;
	}
	else if(keys.left)
	{
		
		keys.pitch = (x - keys.xmouse);
		keys.yaw = (y - keys.ymouse);
		if(keys.pitch > 0)
		{
			camera.xeye++;
		}else if(keys.pitch < 0)
		{
			camera.xeye--;
		}

		if(keys.yaw > 0)
		{
			camera.yeye--;
		}else if(keys.yaw < 0)
		{
			camera.yeye++;
		}
		keys.ymouse = y;
		keys.xmouse = x;
	}
}

void PassivMouseMotion(int x, int y) { // Maus-Bewegungen ohne gedrückte Maus-Taste
	// wird nicht benutzt, zu hecktisch :)
	/*
	int mid_x = width >> 1;
	int mid_y = height >> 1;
	float angle_y = 0.0f;
	float angle_z = 0.0f;

	if ((x == mid_x) && (y == mid_y)) return;

	SetCursorPos(mid_x, mid_y);	// Set the mouse cursor in the center of the window						

	// Get the direction from the mouse cursor, set a resonable maneuvering speed
	angle_y = (float)((mid_x - x)) / 100000;
	angle_z = (float)((mid_y - y)) / 100000;

	// The higher the value is the faster the camera looks around.
	camera.yeye += angle_z * 1;

	// limit the rotation around the x-axis
	if ((camera.yeye - camera.ypos) > 8)  camera.yeye = camera.ypos + 8;
	if ((camera.yeye - camera.ypos) <-8)  camera.yeye = camera.ypos - 8;

	GLdouble x_, y_, z_;
	x_ = camera.xeye - camera.xpos;
	y_ = camera.yeye - camera.ypos;
	z_ = camera.zeye - camera.zpos;

	camera.zeye = (float)(camera.zpos + sin(-angle_y)* x_ + cos(-angle_y)* z_);
	camera.xeye = (float)(camera.xpos + cos(-angle_y)* x_ - sin(-angle_y)* z_);
	*/
}


void SpecialFunc(int key, int x, int y) { // Funktions- und Pfeil-Tasten abfragen
	GLdouble ev = 0.0; // variable für den Einheitsvektor
	GLdouble h1 = 0.0;
	GLdouble vx = 0.0, vy = 0.0, vz = 0.0;
	GLdouble sv1x = 0.0, sv1z = 0.0, sv2x = 0.0, sv2z = 0.0; // hilfs variablen
	switch (key) {
	case GLUT_KEY_F1: // alles wird zurueckgesetzt
		break;
	case GLUT_KEY_F2:;break;
	case GLUT_KEY_F3:;break;
	case GLUT_KEY_F4:;break;
	case GLUT_KEY_F5:;break;
	case GLUT_KEY_F6:;break;
	case GLUT_KEY_F7:;break;
	case GLUT_KEY_F8:break;
	case GLUT_KEY_F9:break;
	case GLUT_KEY_F10:
		break;
	case GLUT_KEY_F11:
		break;
	case GLUT_KEY_F12:
		break;

	case GLUT_KEY_UP:
		// vx ist der Vektor eye - pos
		vx=camera.xeye-camera.xpos;
		vy=camera.yeye-camera.ypos;
		vz=camera.zeye-camera.zpos;
		// ev ist unser Faktor für den einheitsvektor
		h1=(vx*vx)+(vy*vy)+(vz*vz);
		ev= sqrt(h1);
		// verschiebe die Kamera um die jeweilige kordinate geteilt duch den Einheitsvektor
		// somit machen wir nur einen Schritt der Länge 1
		camera.xpos +=vx/ev;
		camera.ypos +=vy/ev;
		camera.zpos +=vz/ev;
		// verschiebe den Kamerablickpunkt um die jeweilige kordinate geteilt duch den Einheitsvektor
		// somit machen wir nur einen Schritt der Länge 1
		camera.xeye +=vx/ev;
		camera.yeye +=vy/ev;
		camera.zeye +=vz/ev;
		break;
	case GLUT_KEY_DOWN:
		vx=camera.xeye-camera.xpos;
		vy=camera.yeye-camera.ypos;
		vz=camera.zeye-camera.zpos;
		h1=(vx*vx)+(vy*vy)+(vz*vz);
		ev= sqrt(h1);
		camera.xpos -=vx/ev;
		camera.ypos -=vy/ev;
		camera.zpos -=vz/ev;

		camera.xeye -=vx/ev;
		camera.yeye -=vy/ev;
		camera.zeye -=vz/ev;
		break;
	case GLUT_KEY_LEFT:
		// hier bertachten wir nur die x z Ebene
		vx=camera.xeye-camera.xpos;
		vz=camera.zeye-camera.zpos;
		sv1x=(vz*(-1));
		sv1z=vx;
		sv2x=vz;
		sv2z=(vx*(-1));

		h1=(sv1x*sv1x)+(sv1z*sv1z);
		ev= sqrt(h1);

		camera.xpos -=sv1x/ev;
		camera.zpos -=sv1z/ev;

		camera.xeye -=sv1x/ev;
		camera.zeye -=sv1z/ev;

		break;
	case GLUT_KEY_RIGHT:
		vx=camera.xeye-camera.xpos;
		vz=camera.zeye-camera.zpos;
		sv1x=(vz*(-1));
		sv1z=vx;
		sv2x=vz;
		sv2z=(vx*(-1));

		h1=(sv1x*sv1x)+(sv1z*sv1z);
		ev= sqrt(h1);

		camera.xpos +=sv1x/ev;
		camera.zpos +=sv1z/ev;

		camera.xeye +=sv1x/ev;
		camera.zeye +=sv1z/ev;
		break;
	case GLUT_KEY_PAGE_UP:break;
	case GLUT_KEY_PAGE_DOWN:break;
	case GLUT_KEY_HOME:break;
	case GLUT_KEY_END:break;
	case GLUT_KEY_INSERT: break;
	}
}



void menue()
{
	std::cout << "========================================================" << std::endl;
	std::cout << "   Projekt Roboter - Arnold Porada, Nikita Balyschew" << std::endl;
	std::cout << "--------------------------------------------------------" << std::endl;
	std::cout << " < F1 >                      RESET" << std::endl;
	std::cout << " < F2 >                      MODUS - Kamera frei bewegen" << std::endl;
	std::cout << " < F3 >                      MODUS - UFO verfolgen" << std::endl;
	std::cout << " < F4 >                      FPS = 10, 25 or 60" << std::endl;
	std::cout << " < F5 >                      FPS ++" << std::endl;
	std::cout << " < F6 >                      FPS --" << std::endl;
	std::cout << " < F7 >                      SPEED ++" << std::endl;
	std::cout << " < F8 >                      SPEED --" << std::endl;
	std::cout << " < F9 >                      Animation anhalten" << std::endl;
	std::cout << " < F10 >                     Markante POS 1" << std::endl;
	std::cout << " < F11 >                     Markante POS 2" << std::endl;
	std::cout << " < F12 >                     Markante POS 3" << std::endl;
	std::cout << " < HOME,END >                Zoom +/-" << std::endl;
	std::cout << " < MOUSE LEFT,RIGHT >        Animation anhalten" << std::endl;
	std::cout << " < UP,DOWN,LEFT,RIGHT,BILD > Kamera Steuerung" << std::endl;
	std::cout << "========================================================" << std::endl << std::endl;
}