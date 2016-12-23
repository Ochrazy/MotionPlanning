#include "SkyBox.h"


SkyBox::SkyBox(void)
{
}


SkyBox::~SkyBox(void)
{
}


// Texturen

////BMP Laden
//AUX_RGBImageRec *SkyBox::LoadBMP(char *Filename) // Lädt ein BMP
//{
//  FILE *File=NULL;
//  if (!Filename){return NULL;}
//  File=fopen(Filename,"r"); 
//  if (File)
//  {
//    fclose(File);
//    return auxDIBImageLoad(Filename); 
//  }
//  return NULL;
//}

int SkyBox::LoadGLTextures()                                    // Load Bitmaps And Convert To Textures
{
	/* load an image file directly as a new OpenGL texture */

	texture[0] = SOIL_load_OGL_texture
		(
		"landscape10.bmp",
		SOIL_LOAD_AUTO,
		SOIL_CREATE_NEW_ID,
		SOIL_FLAG_INVERT_Y
		);
	texture[1] = SOIL_load_OGL_texture
		(
		"landscape11.bmp",
		SOIL_LOAD_AUTO,
		SOIL_CREATE_NEW_ID,
		SOIL_FLAG_INVERT_Y
		);
	texture[2] = SOIL_load_OGL_texture
		(
		"landscape12.bmp",
		SOIL_LOAD_AUTO,
		SOIL_CREATE_NEW_ID,
		SOIL_FLAG_INVERT_Y
		);
	texture[3] = SOIL_load_OGL_texture
		(
		"landscape13.bmp",
		SOIL_LOAD_AUTO,
		SOIL_CREATE_NEW_ID,
		SOIL_FLAG_INVERT_Y
		);
	texture[4] = SOIL_load_OGL_texture
		(
		"landscape14.bmp",
		SOIL_LOAD_AUTO,
		SOIL_CREATE_NEW_ID,
		SOIL_FLAG_INVERT_Y
		);
	texture[5] = SOIL_load_OGL_texture
		(
		"landscape15.bmp",
		SOIL_LOAD_AUTO,
		SOIL_CREATE_NEW_ID,
		SOIL_FLAG_INVERT_Y
		);

	for(int i = 0; i < 6; i++)
	{
		if(texture[i] == 0)
		{
			free(texture);
			return false;
		}

	}


	return true;                                        // Return Success
}

//Textur in Graka laden
//int Skybox::LoadGLTextures(char filename[], int ListIndex) // Bitmaps laden und konvertieren
//{
//  int Status=FALSE; 
//  AUX_RGBImageRec *TextureImage[1]; 
//  memset(TextureImage,0,sizeof(void *)*1); 
//  if (TextureImage[0]=LoadBMP(filename))
//  {
//    Status=TRUE;
//        //Textur erstellen
//        glGenTextures(1, &texture[0]);
//        glBindTexture(GL_TEXTURE_2D, texture[0]);
//        glTexImage2D(GL_TEXTURE_2D, 0, 3, TextureImage[0]->sizeX, 
//                     TextureImage[0]->sizeY, 0, GL_RGB, 
//                     GL_UNSIGNED_BYTE, TextureImage[0]->data);
//        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);// Warp_s und warp t werden auf clamp gesetzt damit nur ein einzelnen Bild auf die Struktur gemaped wird 
//        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);//
//        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);//Bei Verkleinerung textur lineare filterung (Ist der Mitterwert der umliegenden pixel)
//        glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);//beio Vergrößerung textur lineare filterung (Ist der Mitterwert der umliegenden pixel)
//
//    glNewList (ListIndex, GL_COMPILE);// Werden nur abgespeichert mit GL_COMPILE_AND_EXECUTE wirds auch ausgeführt
//        glBindTexture(GL_TEXTURE_2D, teture[0]);
//    glEndList();
//  }
//
//  //Speicher freigeben
//  if (TextureImage[0])
//  {
//    if (TextureImage[0]->data)
//    {
//      free(TextureImage[0]->data);
//    }
//    free(TextureImage[0]);
//  }
//
//  return Status;
//}


//----------------------------------------------------------------------------------
//    Skybox-Demo
//----------------------------------------------------------------------------------




void SkyBox::SkyboxInit(void)
{

	//Kanten hervorheben
	float sz=10.0f;
	glNewList (1, GL_COMPILE);    
	glLineWidth(3);
	glColor3f(1,1,1);
	glBegin(GL_LINES);
	glVertex3f(-sz, -sz, sz);
	glVertex3f(-sz, sz, sz);
	glVertex3f(-sz, sz, sz);
	glVertex3f(sz, sz, sz);
	glVertex3f(sz, sz, sz);
	glVertex3f(sz, -sz, sz);
	glVertex3f(sz, -sz, sz);
	glVertex3f(-sz, -sz, sz);

	glVertex3f(-sz, -sz, -sz);
	glVertex3f(-sz, sz, -sz);
	glVertex3f(-sz, sz, -sz);
	glVertex3f(sz, sz, -sz);
	glVertex3f(sz, sz, -sz);
	glVertex3f(sz, -sz, -sz);
	glVertex3f(sz, -sz, -sz);
	glVertex3f(-sz, -sz, -sz);

	glVertex3f(-sz, -sz, sz);
	glVertex3f(-sz, -sz, -sz);
	glVertex3f(-sz, sz, sz);
	glVertex3f(-sz, sz, -sz);
	glVertex3f(sz, sz, sz);
	glVertex3f(sz, sz, -sz);
	glVertex3f(sz, -sz, sz);
	glVertex3f(sz, -sz, -sz);
	glEnd();
	glEndList();



	//Texturen laden
	LoadGLTextures();

	//Skybox anlegen
	sz=10.0f;
	glNewList (2, GL_COMPILE);
	glEnable(GL_TEXTURE_2D);

	glBindTexture(GL_TEXTURE_2D, texture[0]);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);// Warp_s und warp t werden auf clamp gesetzt damit nur ein einzelnen Bild auf die Struktur gemaped wird 
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);//
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);//Bei Verkleinerung textur lineare filterung (Ist der Mitterwert der umliegenden pixel)
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);//beio Vergrößerung textur lineare filterung (Ist der Mitterwert der umliegenden pixel)

	glBegin(GL_QUADS);   
	glColor3f (1,1,1);
	// Das vordere QUAD
	glNormal3f(0,0,1);
	glTexCoord2d(0, 0); glVertex3f(-sz, -sz, sz); 
	glTexCoord2d(1, 0); glVertex3f( sz, -sz, sz); 
	glTexCoord2d(1, 1); glVertex3f( sz, sz, sz); 
	glTexCoord2d(0, 1); glVertex3f(-sz, sz, sz); 
	glEnd();

	glBindTexture(GL_TEXTURE_2D, texture[1]);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);// Warp_s und warp t werden auf clamp gesetzt damit nur ein einzelnen Bild auf die Struktur gemaped wird 
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);//
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);//Bei Verkleinerung textur lineare filterung (Ist der Mitterwert der umliegenden pixel)
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);//beio Vergrößerung textur lineare filterung (Ist der Mitterwert der umliegenden pixel)

	glBegin(GL_QUADS);    
	// Das rechte QUAD
	glNormal3f(1,0,0);
	glTexCoord2d(1, 0); glVertex3f( sz, -sz, -sz); 
	glTexCoord2d(1, 1); glVertex3f( sz, sz, -sz); 
	glTexCoord2d(0, 1); glVertex3f( sz, sz, sz); 
	glTexCoord2d(0, 0); glVertex3f( sz, -sz, sz); 
	glEnd();

	glBindTexture(GL_TEXTURE_2D, texture[2]);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);// Warp_s und warp t werden auf clamp gesetzt damit nur ein einzelnen Bild auf die Struktur gemaped wird 
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);//
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);//Bei Verkleinerung textur lineare filterung (Ist der Mitterwert der umliegenden pixel)
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);//beio Vergrößerung textur lineare filterung (Ist der Mitterwert der umliegenden pixel)

	glBegin(GL_QUADS);    
	// Das hintere QUAD
	glNormal3f(0,0,-1);
	glTexCoord2d(1, 0); glVertex3f(-sz, -sz, -sz);
	glTexCoord2d(1, 1); glVertex3f(-sz, sz, -sz);
	glTexCoord2d(0, 1); glVertex3f( sz, sz, -sz);
	glTexCoord2d(0, 0); glVertex3f( sz, -sz, -sz);
	glEnd();

	glBindTexture(GL_TEXTURE_2D, texture[3]);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);// Warp_s und warp t werden auf clamp gesetzt damit nur ein einzelnen Bild auf die Struktur gemaped wird 
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);//
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);//Bei Verkleinerung textur lineare filterung (Ist der Mitterwert der umliegenden pixel)
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);//beio Vergrößerung textur lineare filterung (Ist der Mitterwert der umliegenden pixel)

	glBegin(GL_QUADS);    
	// Das linke QUAD
	glNormal3f(-1,0,0);
	glTexCoord2d(0, 0); glVertex3f(-sz, -sz, -sz); 
	glTexCoord2d(1, 0); glVertex3f(-sz, -sz, sz); 
	glTexCoord2d(1, 1); glVertex3f(-sz, sz, sz); 
	glTexCoord2d(0, 1); glVertex3f(-sz, sz, -sz); 
	glEnd();

	glBindTexture(GL_TEXTURE_2D, texture[4]);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);// Warp_s und warp t werden auf clamp gesetzt damit nur ein einzelnen Bild auf die Struktur gemaped wird 
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);//
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);//Bei Verkleinerung textur lineare filterung (Ist der Mitterwert der umliegenden pixel)
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);//beio Vergrößerung textur lineare filterung (Ist der Mitterwert der umliegenden pixel)

	glBegin(GL_QUADS);    
	// Das obere QUAD
	glNormal3f(0,1,0);
	glTexCoord2d(0, 1); glVertex3f(-sz, sz, -sz); 
	glTexCoord2f(0, 0); glVertex3f(-sz, sz, sz); 
	glTexCoord2f(1, 0); glVertex3f( sz, sz, sz); 
	glTexCoord2f(1, 1); glVertex3f( sz, sz, -sz); 
	glEnd();

	glBindTexture(GL_TEXTURE_2D, texture[5]);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP);// Warp_s und warp t werden auf clamp gesetzt damit nur ein einzelnen Bild auf die Struktur gemaped wird 
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP);//
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);//Bei Verkleinerung textur lineare filterung (Ist der Mitterwert der umliegenden pixel)
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);//beio Vergrößerung textur lineare filterung (Ist der Mitterwert der umliegenden pixel)

	glBegin(GL_QUADS);    
	// Das untere QUAD
	glNormal3f(0,-1,0);
	glTexCoord2d(1, 1); glVertex3f( sz, -sz, sz); 
	glTexCoord2d(0, 1); glVertex3f(-sz, -sz, sz);
	glTexCoord2d(0, 0); glVertex3f(-sz, -sz, -sz); 
	glTexCoord2d(1, 0); glVertex3f( sz, -sz, -sz); 
	glEnd();

	glDisable(GL_TEXTURE_2D);
	glEndList();    
}



void SkyBox::paint(void)
{
	//

	float dx;
	float dy;

	//User-Eingaben verarbeiten
	if (keys[65]) //A gedrückt: Außerhalb der Box
	{
		distance=35;
	}

	if (keys[73]) //I gedrückt: Innerhalb der Box
	{
		distance=0;
	}

	if (keys[75]) //k gedrückt: Kanten an/aus
	{
		kanten=!kanten;
		keys[75]=false;
	}

	//Maus-Aktion
	if (keys[MK_LBUTTON]) //wenn Mousebutton gedrückt
	{
		//relative Mausposition zum Clickpunkt finden
		dx=float(mouse_x-click_x);
		dy=float(mouse_y-click_y);
		//und in Bewegung einrechnen
		heading+=dx/100;
		pitch+=dy/100;
		if (pitch>90){pitch=90;}
		if (pitch<-90){pitch=-90;}
	}

	//Zeichnen
	//glLoadIdentity();
	glTranslatef (0,0,-distance);
	glRotatef (pitch,1,0,0);//Dreheung
	glRotatef (heading,0,1,0);//Neigung

	if(distance==0)
	{
		//Skybox von innen - ohne DepthTest
		glDisable(GL_DEPTH_TEST);
		glCallList(2);
		glEnable(GL_DEPTH_TEST);
	}
	else
	{
		//Skybox von außen - mit Depth Test
		glCallList(2);
	}


	if(kanten) //Kanten zeichnen?
	{
		glCallList(1);
	}

} 