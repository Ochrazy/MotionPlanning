/******************************************************************************
    file:       main.cpp
    created:
******************************************************************************/

#include <iostream>
#include <fstream>
#include <vector>
#include <Windows.h>

#include "Point.h"
#include "Box.h"
#include "Bug0.h"

using namespace std;

// Ausgabe einer EASYROB *.prg Datei zur Simulation der Ergebnisse und Verifikation
void write_program_file(vector<Point> path)
{
    ofstream myfile;
    myfile.open("Bug0.prg");
    // Iterate and print values of path
    vector<Point>::const_iterator i;
    myfile << "ProgramFile" << endl;
    //<< "EndInit" << endl << "SPEED_PTP_OV   80.0000" << endl << "ACCEL_PTP_OV   100.0000" << endl
    //<< "OV_PRO         100.0000" << endl;
    for (i = path.begin(); i != path.end(); ++i)
    {
        //myfile << "JUMP_TO_AX  " << i->x << "  " << i->y << "  " << i->z << endl;
        myfile << "PTP_AX " << i->x << "  " << i->y << endl;
    }
    myfile << "EndProgramFile" << endl;
    myfile.close();
}

/*
 *  main
 */
int main(void)
{
    const double ds = 0.01f;
    const int nHind = 2;    // Anzahl der Hindernisse
    const int nRob  = 1;    // Anzahl der Roboterglieder
                            // Create a vector containing integers
    vector<Point> path;

    Box aHindernis[nHind];  // Unsere Hindernisse
    Box Roboter[nRob];      // Roboter

    // Hindernisse initialisieren
    // Hierbei wird für jedes Hinderniss die Größe ( Skalierung ) und
    // die Position im Raum gesetzt
    aHindernis[0].Scale( 0.20f, 0.20f, 0.10f );     // QUADER1   0.20000    0.2000    0.10000
    aHindernis[0].Set(0.1f, 0.1f, 0.0f);      // REFPOS   0.200000   0.2000000    0.0000000    0.0000000    0.0000000    0.0000000

    aHindernis[1].Scale( 0.2f, 0.2f, 0.10f );       // QUADER2   0.2000    0.20000    0.10000
    aHindernis[1].Set( 0.2f, 0.4f, 0.0f );    // REFPOS   0.200000   0.4000000    0.0000000    0.0000000    0.0000000    0.0000000

    // Roboter initialisieren
    Roboter[0].Scale( 0.05f, 0.05f, 0.20f );       // QUADER    0.05000    0.05000    0.20000
    //Roboter[0].Translate( -0.025f, -0.025f, 0.0f );  // REFPOS   -0.02500000   -0.02500000    0.0000000    0.0000000    0.0000000    0.0000000

    // Startzeit
    DWORD dwStart = GetTickCount();

    // Initialize start, goal, actPoint and heading
    Bug0 Bug0("Bug0");
    Bug0.setStartPosition(0.2f, 0.f); // EASYROB
    Bug0.setGoalPosition(0.3f, 0.7); // EASYROB
    Roboter[0].Set(Bug0.getStartPosition());
    Bug0.setIntermediatePoint(Bug0.getGoalPosition()); // EASYROB
    Bug0.setActPoint(Bug0.getStartPosition());
    Bug0.setHeading((Bug0.getGoalPosition() - Bug0.getStartPosition()).Normalize());
    path.push_back(Bug0.getStartPosition());

    Point robPos;
    bool goal_reached = false;
    while (!goal_reached)
    {
      goal_reached = Bug0.update(aHindernis, Roboter, nHind);
      robPos = Bug0.getRobPos();
	  Roboter[0].Set(robPos);
      path.push_back(Bug0.getRobPos()); // speichern des Aktuellen Punktes in vector<Point> path
      cout << robPos.x << " " << robPos.y << endl; // Ausgabe auf Konsole
    }

    // Zeit für das Aufstellen des Konfigurationsraumes ausgeben ( in ms )
    DWORD dwElapsed = GetTickCount() - dwStart;
    write_program_file(path);
    printf("\nBerechnung dauerte %d ms\n", dwElapsed);

    return 0;
}
