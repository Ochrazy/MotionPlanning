#include <GL\freeglut.h>
//#include <GL\SOIL.h>
#include <string>
#include "SOIL.h"
using namespace std;
class SkyBox
{
public:
	SkyBox(void);
	~SkyBox(void);
	int click_x;
	int click_y;
	int mouse_x;
	int mouse_y;
	GLfloat pitch;
	GLfloat kanten;
	GLfloat distance;
	GLfloat heading;
	bool keys[256];
	int LoadGLTextures();
	GLuint texture[6];
	void paint();
	void SkyboxInit(void);
};

