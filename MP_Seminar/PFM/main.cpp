/* -----------------------------------------------------------	*/
/* Graph.-DV-Praktikum Startprogramm                           */
/* -----------------------------------------------------------	*/
/* Datei: main.cpp          Barlyschew, Porada                   */
/* -----------------------------------------------------------	*/

#define _USE_MATH_DEFINES


#include <Windows.h>
#include "Globals.h"

char** arguments;
int argumentcount;
using namespace std;

double baseTimeInSecons = 3.0;

bool check_local_minimum(vector<Point>, Point, int i);
void MouseFunc(int, int, int, int); // Maus-Tasten und -Bewegung abfragen
void MouseMotion(int, int); // Maus-Bewegungen mit gedrückter Maus-Taste
void PassivMouseMotion(int, int); // Maus-Bewegungen ohne gedrückte Maus-Taste
void SpecialFunc(int, int, int); // Funktions- und Pfeil-Tasten abfragen
void menue();

void Init() {
	// Hier finden jene Aktionen statt, die zum Programmstart einmalig 
	// durchgefuehrt werden muessen
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Hintergrundfarbe definieren 
	glEnable(GL_DEPTH_TEST); //Aktiviert eine server-seitige F�higkeit. GL_DEPTH_TEST ist die F�higkeit Tiefenvergleiche zu t�tigen und der Tiefenpuffer aktualisiert
	glClearDepth(1.0); // legt den Wert fest, mit dem der Tiefenpuffer beim L�schen des selbigen gef�llt wird. 1.0 ist das maximum 0.0 ist das minimum
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_NORMALIZE); // Farbverlaeufe glaetten    
	GLfloat mat_specular[] = {0.5f, 0.5f, 0.5f, 0.5f}; // Array fuer Glanz-Richtung
	GLfloat mat_shininess[] = {50.0f}; // Array fuer die Leucht-Intensitaet
	GLfloat white_light[] = {1.0f, 1.0f, 1.0f, 1.0f}; // Array fuer die Farbe des Lichts
	GLfloat ambientLight[] = {0.05f, 0.05f, 0.05f, 0.0f}; // Array fuer die Farbe des Ambient-Lichts
	glLightfv(GL_LIGHT0, GL_SPECULAR, white_light); // Farbe und Art des Licht an die Lichtquelle binden
	glLightfv(GL_LIGHT0, GL_DIFFUSE, white_light); // Farbe und Art des Licht an die Lichtquelle binden
	glMaterialfv(GL_FRONT, GL_SPECULAR, mat_specular); // Farbe und Art des Licht an das Material binden
	glMaterialfv(GL_FRONT, GL_SHININESS, mat_shininess); // Farbe und Art des Licht an das Material binden
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambientLight); // Farbe und Art des Licht an die Scene binden
	GLfloat lightpos[] = { 0.125, 0.25, 1., 0. };
	glLightfv(GL_LIGHT0, GL_POSITION, lightpos);
	glShadeModel(GL_FLAT);

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

	// skybox.SkyboxInit();
	// roboter.init(mass, farben, animationen);

	// Hindernisse initialisieren
	// Hierbei wird für jedes Hinderniss die Größe ( Skalierung ) und die Position im Raum gesetzt
	
	if (strcmp(arguments[1], "pfm") != 0)
		return;

	if (strcmp(arguments[3], "0") == 0) {

		aHindernis[0].SetCenter(2., 2., 2.);
		aHindernis[0].SetRadius(0.00);
		aHindernis[0].SetRepulsivness(0);

		aHindernis[1].SetCenter(2., 2., 2.);
		aHindernis[1].SetRadius(0.00);
		aHindernis[1].SetRepulsivness(0);

		aHindernis[2].SetCenter(2., 2., 2.);
		aHindernis[2].SetRadius(0.00);
		aHindernis[2].SetRepulsivness(0);

		aHindernis[3].SetCenter(2., 2., 2.);
		aHindernis[3].SetRadius(0.00);
		aHindernis[3].SetRepulsivness(0);

	}else if(strcmp(arguments[3], "1") == 0){
	
	aHindernis[0].SetCenter(0.4, 0.4, 0.); 
	aHindernis[0].SetRadius(0.05);
	aHindernis[0].SetRepulsivness(20);

	aHindernis[1].SetCenter(0.4, 0.6, 0.0);
	aHindernis[1].SetRadius(0.05);
	aHindernis[1].SetRepulsivness(20);

	aHindernis[2].SetCenter(0.6, 0.4, 0.0);
	aHindernis[2].SetRadius(0.05);
	aHindernis[2].SetRepulsivness(20);

	aHindernis[3].SetCenter(0.6, 0.6, 0.0);
	aHindernis[3].SetRadius(0.05);
	aHindernis[3].SetRepulsivness(20);

	}else if (strcmp(arguments[3], "2") == 0) {

	aHindernis[0].SetCenter(0.5, 0.3, 0.);
	aHindernis[0].SetRadius(0.05);
	aHindernis[0].SetRepulsivness(20);

	aHindernis[1].SetCenter(0.5, 0.7, 0.0);
	aHindernis[1].SetRadius(0.05);
	aHindernis[1].SetRepulsivness(20);

	aHindernis[2].SetCenter(0.7, 0.5, 0.0);
	aHindernis[2].SetRadius(0.05);
	aHindernis[2].SetRepulsivness(20);

	aHindernis[3].SetCenter(0.3, 0.5, 0.0);
	aHindernis[3].SetRadius(0.05);
	aHindernis[3].SetRepulsivness(20);
	
	}
	else if (strcmp(arguments[3], "3") == 0) {

		
		std::uniform_real_distribution<double> distribution(0.15, 0.85);
		for(int i = 0; i < atoi(arguments[5]); ++i){

			myclock::duration d = myclock::now() - beginning;
			unsigned seed2 = d.count();
			std::default_random_engine generator(seed2);
			aHindernis[i].SetCenter(distribution(generator), distribution(generator), 0.);
			aHindernis[i].SetRadius(0.05);
			aHindernis[i].SetRepulsivness(20);
		}
	}
	for (int i = 0; i < nRob; ++i)
	{
		if (i < atoi(arguments[4])) {
			radius[i] = 0.05;
			roboterPot[i].SetRadius(radius[i]);

			goal_reached[i] = false;
			local_minimum_reached[i] = false;
		
			// Initialize start, goal, actPoint and heading
			if (i == 0) {
				pot[i].setStartPosition(1.0, 1.0);
				pot[i].setGoalPosition(0.0, 0.0);
				roboterPot[i].SetRepulsivness(1);
			}else if (i == 1) {
				pot[i].setStartPosition(0.0, 0.0);
				pot[i].setGoalPosition(1.0, 1.0);
				roboterPot[i].SetRepulsivness(4);
			}else if ( i == 2) {
				pot[i].setStartPosition(1.0, 0.0);
				pot[i].setGoalPosition(0.0, 1.0);
				roboterPot[i].SetRepulsivness(8);
			}
			else if (i == 3) {
				pot[i].setStartPosition(0.0, 1.0);
				pot[i].setGoalPosition(1.0, 0.0);
				roboterPot[i].SetRepulsivness(16);
			}
			total_counter[i] = 0;
			roboterPot[i].SetCenter(pot[i].getStartPosition());
			pot[i].setActPoint(pot[i].getStartPosition());
			path[i].push_back(pot[i].getStartPosition());
		}
	}
}

void RenderScene(void) 
{
	// Hier befindet sich der Code der in jedem frame ausgefuehrt werden muss
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Puffer loeschen
	glLoadIdentity(); //Ersetzt die aktuelle Matrix durch die Einheitsmatrix
	gluLookAt(camera.xpos, camera.ypos, camera.zpos,
		camera.xeye, camera.yeye, camera.zeye,
		camera.lux, camera.luy, camera.luz); // Kamera von oben
	glPushMatrix();
	
	//skybox.paint();
	//roboter.paint();
	
	if (currentAlgorithm == Algorithm::RRTConnect)
	{
		for (int i = 0; i < atoi(arguments[2]); ++i) {
			glPushMatrix();
			glColor4f(farben[i * 4 + 0], farben[i * 4 + 1], farben[i * 4 + 2], farben[i * 4 + 3]);
			glTranslatef(robPos[i].x, robPos[i].y, 0.f);
			//glTranslatef(0.05, 0.025, 0.0);
			glScalef(0.1, 0.05, 0.05);
			glutSolidCube(1.f);
			glPopMatrix();
		}

		if(atoi(arguments[5]) > 0)
		{
			/*tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.1 + 0.0, 0.2 + 0.55, 0)));
			tf_obstacle.push_back(fcl::Transform3f(fcl::Vec3f(0.1, 0.25, 0)));

			for (auto i : tf_obstacle)
				obj_obstacle.push_back(std::make_shared<fcl::Box>(0.2, 0.4, 0.05));*/


			glPushMatrix();
			glColor4f(0.5, 0.5, 0.5, 1.);
			glTranslatef(0.1, 0.27, 0.f);
			//glTranslatef(0.1, 0.2, 0.0);
			glScalef(0.2, 0.4, 0.05);
			glutSolidCube(1.f);
			glPopMatrix();

			glPushMatrix();
			glColor4f(0.5, 0.5, 0.5, 1.);
			glTranslatef(0.1, 0.73, 0.f);
			//glTranslatef(0.1, 0.2, 0.0);
			glScalef(0.2, 0.4, 0.05);
			glutSolidCube(1.f);
			glPopMatrix();
		}
	}
	else if (currentAlgorithm == Algorithm::PotentialFieldMethod)
	{
		for (int i = 0; i < atoi(arguments[4]); ++i){
			glPushMatrix();
			glColor4f(farben[i * 4 + 0], farben[i * 4 + 1], farben[i * 4 + 2], farben[i * 4 + 3]);
			glTranslatef(robPos[i].x, robPos[i].y, 0.f);
			glutSolidCylinder(roboterPot[i].GetRadius(), 0.05f, 32, 32);
			glPopMatrix();

		}
		
		for (int i = 0; i < atoi(arguments[5]); ++i) {
			glPushMatrix();
			glColor4f(farben[28 + 0], farben[28 + 1], farben[28 + 2], farben[28 + 3]);
			glTranslatef(aHindernis[i].GetCenter().x, aHindernis[i].GetCenter().y, 0.f);
			glutSolidCylinder(aHindernis[i].GetRadius(), 0.05f, 32, 32);
			glPopMatrix();
		}
	}

	glPushMatrix();
	glColor4f(0.f, 1.f, 0.f, 1.f);
	glTranslatef(0.5f, 0.5f, 0.f);
	glScalef(1., 1., 0.0);
	glutWireCube(1.f);
	glPopMatrix();

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
	gluPerspective(45.0, width/height , 0.001, 1000); // definiert das Frustum und die parameter sind wie folgt definiert (links, rechts, unten, oben, vodere und hintere begrenzung) 

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
	// Startzeit
	DWORD dwStart = GetTickCount();
	for (int i = 0; i < nRob; ++i)
	{
		if (i < atoi(arguments[4]) && !goal_reached[i] && !local_minimum_reached[i])
		//if (!goal_reached && !local_minimum_reached)
		{
			Cylinder obstacles[nHind];
			int count = 0;
			double dist = DBL_MAX;
			int nearestObst;

			for (int j = 0; j < nRob; ++j)
			{
				if (j != i && j < atoi(arguments[4]))
				{
				/*	double tmpDist;
					Point tmpPoint;
					if ((tmpDist = roboterPot[j].distance(roboterPot[i], &tmpPoint)) < dist) {
						dist = tmpDist;
						nearestObst = j;
						obstacles[count] = roboterPot[j];
						}*/
					obstacles[count] = roboterPot[j];
					//obstacles[count].SetCenter(robPos[j].x, robPos[j].y, robPos[j].z);
					++count;
				}
				
			}

			for (int j = nRob; j < nHind +1 ; ++j)
			{
				/*double tmpDist;
				Point tmpPoint;
				if ((tmpDist = aHindernis[j - nRob].distance(roboterPot[i], &tmpPoint)) < dist) {
					dist = tmpDist;
					nearestObst = j;
					obstacles[count] = aHindernis[j - nRob];
				}*/
				if (j  < atoi(arguments[5]) + nRob) {
					obstacles[count] = aHindernis[j - nRob];
					//obstacles[count].SetCenter(robPos[j].x, robPos[j].y, robPos[j].z);
					++count;
				}
					
			}

			if(strcmp(arguments[2], "normal") == 0){
				goal_reached[i] = pot[i].update_cylinder(obstacles, &(roboterPot[i]), count);
			}
			else if (strcmp(arguments[2], "navigation") == 0){

				Cylinder* obstaclesTMP = new Cylinder[count + 1] ;
				obstaclesTMP[0] = Cylinder(Point(0.5, 0.5, 0.0), 0.75, 0.05);
				++count;
				for (int i = 1; i < count; i++)
				{
					obstaclesTMP[i] = obstacles[i - 1];
				}

				goal_reached[i] = pot[i].update_cylinder_navigation(obstaclesTMP, &(roboterPot[i]), count , atof(arguments[6]));
			}

			robPos[i] = roboterPot[i].GetCenter();
			//robPos[i] = pot[i].getRobPos();
			//roboterPot[i].SetCenter(robPos[i].x, robPos[i].y, robPos[i].z);
			local_minimum_reached[i] = check_local_minimum(path[i], robPos[i], i);
			path[i].push_back(pot[i].getRobPos()); // speichern des Aktuellen Punktes in vector<Point> path
			cout << "Robot " << i << ": " << robPos[i].x << " " << robPos[i].y << endl; // Ausgabe auf Konsole
			if (local_minimum_reached[i])
				cout << "Robot " << i << ": " << "reached local minimum" << endl;
			if (goal_reached[i])
				cout << "Robot " << i << ": " << "reached goal" << endl;


		}
	}
	// Zeit für das Aufstellen des Konfigurationsraumes ausgeben ( in ms )
	DWORD dwElapsed = GetTickCount() - dwStart;
	printf("\nBerechnung dauerte %d ms\n", dwElapsed);
	glutPostRedisplay();

	int count = 0;
	for (int i = 0; i < atoi(arguments[4]); ++i) {
		if (goal_reached[i]) {
			++count;
		}
	}
	for (int i = 0; i < atoi(arguments[4]); ++i) {
		if (local_minimum_reached[i]) {
			++count;
		}
	}

	if (count == atoi(arguments[4])) {
		Init();
	}

	// Timer wieder registrieren - Animate wird so nach 100 msec mit value+=1 aufgerufen.
	glutTimerFunc(animationUpdateInMS, Animate, ++value);
}


void AnimateProgram(int value)
{
	static int waypointIndex = 0;
	// TimeSync
	static int steps = 0;
	double timeInSecons = baseTimeInSecons;

	static int maxSteps = (1.0 / (animationUpdateInMS / 1000.0));
	if (steps > (maxSteps*timeInSecons))
	{
		steps = 0;
		waypointIndex++;
		if (waypointIndex > inputProgram->program.size() - 2)
			waypointIndex = 0;

		timeInSecons = baseTimeInSecons;
		double baseSpeed = 0.25;
		Point startWaypoint(inputProgram->program[waypointIndex][0].x, inputProgram->program[waypointIndex][0].y, 0.);
		Point endWaypoint(inputProgram->program[waypointIndex + 1][0].x, inputProgram->program[waypointIndex + 1][0].y, 0.);
		double dis = startWaypoint.Distance(endWaypoint);

		if (dis > baseSpeed)
		{
			timeInSecons -= (dis - baseSpeed) * 2.0;
		}
		else
		{
			timeInSecons -= (baseSpeed - dis) * 10.0;
		}
		if (timeInSecons < 0.1)
			timeInSecons = 0.1;
	}
	for (int nRob = 0; nRob < inputProgram->program[0].size(); nRob++)
	{
		Point startWaypoint(inputProgram->program[waypointIndex][nRob].x, inputProgram->program[waypointIndex][nRob].y, 0.);
		Point endWaypoint(inputProgram->program[waypointIndex + 1][nRob].x, inputProgram->program[waypointIndex + 1][nRob].y, 0.);
		Point direction = (endWaypoint - startWaypoint).Normalize();
		double lengthStep = startWaypoint.Distance(endWaypoint) * (animationUpdateInMS / 1000.0);
		robPos[nRob] = startWaypoint + direction * (lengthStep / timeInSecons) * steps;
	}

	steps++;

	//// SpeedSync
	//double maxSpeed = 0.0025;
	//for (int nRob = 0; nRob < inputProgram->program[0].size(); nRob++)
	//{
	//	Point startWaypoint(inputProgram->program[waypointIndex][nRob].x, inputProgram->program[waypointIndex][nRob].y, 0.);
	//	Point endWaypoint(inputProgram->program[waypointIndex + 1][nRob].x, inputProgram->program[waypointIndex + 1][nRob].y, 0.);
	//	Point direction = (endWaypoint - startWaypoint).Normalize();
	//	double lengthStep = startWaypoint.Distance(endWaypoint) * (animationUpdateInMS / 1000.0);
	//	robPos[nRob] = startWaypoint + direction * maxSpeed * step;// (lengthStep / timeInSecons) * steps;
	//}

	//step++;

	glutPostRedisplay();
	// Timer wieder registrieren - Animate wird so nach 100 msec mit value+=1 aufgerufen.
	glutTimerFunc(animationUpdateInMS, AnimateProgram, ++value);
}

int main(int argc, char* argv[]) {

	// Debug command line argument (rrt):
	//argc = 2;
	//argv[1] = "rrt";

	//argc = 7;
	//argv[1] = "pfm";
	//argv[2] = "navigation";
	//argv[3] = "2";
	//argv[4] = "3";
	//argv[5] = "3";
	//argv[6] = "10";

	arguments = argv;
	argumentcount = argc;
	
	camera.camera();
	keys.keys();
	
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

	if (argc > 1)
	{
		// Load program
		if (strcmp(argv[1], "rrt") == 0)
		{
			inputProgram = new Program();
			currentAlgorithm = Algorithm::RRTConnect;
			std::ifstream file;
			file.open(argv[4]);
			baseTimeInSecons = atof(argv[3]);

			std::string input;
			file >> input; // "ProgramFile"

			while (!file.eof())
			{
				file >> input; // "PTP_AX "
				if (input.compare("EndProgramFile") == 0)
					break;

				// 2 Robots
				Point2D point;
				vector<Point2D> prog;
				for (int nRob = 0; nRob < atoi(argv[2]); nRob++)
				{
					file >> point.x >> point.y;
					prog.push_back(point);
				}

				inputProgram->program.push_back(prog);
			}
			file.close();
		} else if (strcmp(argv[1],"pfm") == 0) {
			currentAlgorithm = Algorithm::PotentialFieldMethod;
		}
		// TimerCallback registrieren; wird nach animationUpdateInMS aufgerufen mit Parameter 0 für Frame 0
		
		if (currentAlgorithm == Algorithm::RRTConnect)
			glutTimerFunc(animationUpdateInMS, AnimateProgram, 0);
		else glutTimerFunc(animationUpdateInMS, Animate, 0);
		glutMainLoop();
	}
	else
	{ 
		cout << "provide parameter \"pfm\" or \"rrt\"" << endl;
	}
	return 0;
}

bool check_local_minimum(vector<Point> path, Point act, int i)
{
	
	if (total_counter[i] > 200)
		if (act.Distance(path[total_counter[i] - 200]) < 0.003)
		{
			return true;
		}

	total_counter[i]++;

	return false;
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

void PassivMouseMotion(int x, int y) {
}

void SpecialFunc(int key, int x, int y) { // Funktions- und Pfeil-Tasten abfragen
	GLdouble ev = 0.0; // variable für den Einheitsvektor
	GLdouble h1 = 0.0;
	GLdouble vx = 0.0, vy = 0.0, vz = 0.0;
	GLdouble sv1x = 0.0, sv1z = 0.0, sv2x = 0.0, sv2z = 0.0; // hilfs variablen
	switch (key) {
		case GLUT_KEY_F1: // alles wird zurueckgesetzt
			break;
		case GLUT_KEY_F2:
			break;
		case GLUT_KEY_F3:
			break;
		case GLUT_KEY_F4:
			break;
		case GLUT_KEY_F5:
			break;
		case GLUT_KEY_F6:
			break;
		case GLUT_KEY_F7:
			break;
		case GLUT_KEY_F8:
			break;
		case GLUT_KEY_F9:
			break;
		case GLUT_KEY_F10:
			Init();
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

void menue() {
	std::cout << "========================================================" << std::endl;
	std::cout << "   Projekt Roboter - Johannes Wambach, Nikita Balyschew" << std::endl;
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