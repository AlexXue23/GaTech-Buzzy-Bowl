/*
Author: Tianhao Xue
Class: ECE6122
Last Date Modified: 12/3/2019
Description:
	This program is a 3D simulation or a half-time show using UAVs. The UAVs remain on the ground
	for 5 seconds after the beginning of the simulation. The UAVs then launch from the ground and
	go towards the point (0, 0, 50 m) above the ground with a maximum velocity of 2 m/s. As they 
	approach within 10m from the point, they began to fly in random paths along the surface of a
	virtual sphere of radius 10 m while attempting to maintain a speed between 2 to 10 m/s. The 
	simulation ends once all of the UAV have come within 10 m of the point, (0, 0, 50 m), and the
	UAVs have flown along the surface for 60 seconds.
*/
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include "mpi.h"
#include "iomanip"
#include <cmath>
#include <math.h>
#include <cstdlib>
#include <GL/glut.h>
#include <chrono>
#include <thread>
#include "ECE_Bitmap.h"

using namespace std;
// Message buffers sending between UAVs and main process
double sendBuffer[6] = { 0.0 };    // Single Uav's x,y,z,vx,vy,vz values
double rcvBuffer[16][6] = { 0.0 };    // 15 Uavs' x,y,z,vx,vy,vz values

// Camera z-axis location
GLdouble eyeZ = 100.0;

// Declare texture-related variables
typedef struct Image Image;
GLuint texture[1];
BMP inBitmap;
GLdouble fieldLength = 120.0;
GLdouble fieldWidth = 60.0;

// Declare UAV-related variables
double x = 0.0, y = 0.0, z = 0.0,  // location
	   fx = 0.0, fy = 0.0, fz = 0.0,  // force
	   v = 0.0, vx = 0.0, vy = 0.0, vz = 0.0,  // velocity
       distanceShpere = 0.0, distanceUav = 0.0,	 // Distance from sphere center and distance from other UAV 
	   g = 10.0,  // Gravity force
	   fa = 0.0, fb = 0.0, fc = 0.0, ft = 0.0,  // Tangential force
	   fs = 0.0;  // Force based on Hooke's Law 
bool surfaceFlag = 0,  // Flag of touching the surface of sphere
	 colorFlag = 0;  // Flag of changing the color of UAV
int countFlyTime = 0,  // Record time after UAVs have come into shpere
    countTime = 0,  // Record time 
    colorMagnitude = 255;  // The magnitude of color

//----------------------------------------------------------------------
// Reshape callback
//----------------------------------------------------------------------
void changeSize(int w, int h)
{
	float ratio = ((float)w) / ((float)h); // window aspect ratio
	glMatrixMode(GL_PROJECTION); // projection matrix is active
	glLoadIdentity(); // reset the projection
	gluPerspective(60.0, ratio, 0.1, 1000.0); // perspective transformation
	glMatrixMode(GL_MODELVIEW); // return to modelview mode
	glViewport(0, 0, w, h); // set viewport (drawing area) to entire window
}

//----------------------------------------------------------------------
// Setup texture properties
//----------------------------------------------------------------------
void setTexture()
{
	glDepthFunc(GL_LESS);
	inBitmap.read("ff.bmp");  // Read texture file "ff.bmp"
	glGenTextures(1, texture); 	// Create textures
	// Set up texture
	glBindTexture(GL_TEXTURE_2D, texture[0]);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, inBitmap.bmp_info_header.width, inBitmap.bmp_info_header.height, 0, GL_RGB, GL_UNSIGNED_BYTE, &inBitmap.data[0]);
	// Scale linearly when image is bigger than texture
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	// Scale linearly when image is smaller than texture
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	glEnable(GL_TEXTURE_2D);
}

//----------------------------------------------------------------------
// Draw the entire scene
//----------------------------------------------------------------------
void renderScene()
{
	// Clear color and depth buffers
	glClearColor(0.4, 0.6, 0.8, 1.0); // background color
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// Reset transformations
	glLoadIdentity();

	// Set Camera
	gluLookAt(
		50.0, 0.0, eyeZ, //  camera location
		0.0, 0.0, 25.0,	  //  center point
		0.0, 0.0, 1.0);   //  up vector
	glMatrixMode(GL_MODELVIEW);

	// Draw football field
	glPushMatrix();
	glBindTexture(GL_TEXTURE_2D, texture[0]); // Bind texture
	glBegin(GL_QUADS);
	glTexCoord2d(0.025, 0.01); // Set texture coordinates.
	glVertex3d(0.914 * fieldWidth / 2.0, -0.914 * fieldLength / 2.0, 0.0); // Set vertices of square
	glTexCoord2d(0.025, 0.99);
	glVertex3d(-0.914 * fieldWidth / 2.0, -0.914 * fieldLength / 2.0, 0.0);
	glTexCoord2d(0.975, 0.99);
	glVertex3d(-0.914 * fieldWidth / 2.0, 0.914 * fieldLength / 2.0, 0.0);
	glTexCoord2d(0.975, 0.01);
	glVertex3d(0.914 * fieldWidth / 2.0, 0.914 * fieldLength / 2.0, 0.0);
	glEnd();
	glBindTexture(GL_TEXTURE_2D, 0); // Detach
	glPopMatrix();

	// Draw Sphere above the ground
	glColor3ub(10, 8, 8);
	glPushMatrix();
	glTranslatef(0.0, 0.0, 50.0);
	glutWireSphere(10.0, 20, 20);
	glPopMatrix();

	// Draw UAVs
	for (int i = 1; i < 16; ++i)
	{
		// Change the magnitude of color every 20 time steps
		if (countTime % 20 == 0)
		{
			if (colorFlag)
			{
				colorMagnitude++;
			}
			else
			{
				colorMagnitude--;
			}
		}
		if (colorMagnitude == 255)
		{
			colorFlag = 0;
		}
		if (colorMagnitude == 128)
		{
			colorFlag = 1;
		}
		// Color: Magenta according to my last name: Xue
		glColor3ub(colorMagnitude, 0, colorMagnitude); 
		glPushMatrix();
		glTranslatef(rcvBuffer[i][0], rcvBuffer[i][1], rcvBuffer[i][2]);
		// Shape: Sphere according to my first name: Tianhao
		glutSolidSphere(0.5, 30, 30);
		glPopMatrix();
	}
	glutSwapBuffers(); // Make it all visible

	// Communicate with UAVs
	MPI_Allgather(sendBuffer, 6, MPI_DOUBLE, rcvBuffer, 6, MPI_DOUBLE, MPI_COMM_WORLD);
	countTime++;

	// Let program end after 600 time steps (main process need one more loop to make sure it end at the same time)
	if (countTime == 601)
	{
		MPI_Finalize();
		this_thread::sleep_for(chrono::milliseconds(10)); // Wait until other processes end
		exit(0);
	}
}

//----------------------------------------------------------------------
// User-input callbacks
// ¡®d¡¯ or ¡®D¡¯: moves the eye location down 
// ¡®u¡¯ or ¡®U¡¯: moves the eye location up 
//----------------------------------------------------------------------
void processKeys(unsigned char key, int xx, int yy)
{
	switch (key)
	{
	case 'u':
	case 'U':
		eyeZ += 5.0;
		break;
	case 'd':
	case 'D':
		eyeZ -= 5.0;
		break;
	}
}

//----------------------------------------------------------------------
// timerFunction  - called whenever the timer fires
//----------------------------------------------------------------------
void timer(int id)
{
	renderScene();
	glutTimerFunc(100, timer, 0);
}

//----------------------------------------------------------------------
// mainOpenGL  - standard GLUT initializations and callbacks   
//----------------------------------------------------------------------
void mainOpenGL(int argc, char** argv)
{

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowPosition(200, 50);
	glutInitWindowSize(400, 400);

	glutCreateWindow("Tianhao Xue's Final Project");
	glEnable(GL_DEPTH_TEST);
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glShadeModel(GL_SMOOTH);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_NORMALIZE);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	// Set texture 
	setTexture();
	glutReshapeFunc(changeSize);
	glutDisplayFunc(renderScene);

	// Redisplay every 0.1s
	timer(0);
	glutIgnoreKeyRepeat(1); // ignore key repeat when holding key down
	glutKeyboardFunc(processKeys); // process standard key clicks
	glutMainLoop();
}

/////////////////////////////////////////////////////////////////////////
// MAIN FUNCTION
/////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
	int numTasks, rank;
	int rc = MPI_Init(&argc, &argv);
	if (rc != MPI_SUCCESS)
	{
		printf("Error starting MPI program. Terminating.\n");
		MPI_Abort(MPI_COMM_WORLD, rc);
	}
	MPI_Comm_size(MPI_COMM_WORLD, &numTasks);
	MPI_Comm_rank(MPI_COMM_WORLD, &rank);

	// initialize UAV's location
	for (int i = 0; i < 5; ++i)
	{
		for (int j = 0; j < 3; ++j)
		{
			rcvBuffer[5 * j + i + 1][0] = 27.0 * 0.914 * j - 27.0 * 0.914;
			rcvBuffer[5 * j + i + 1][1] = 25.0 * 0.914 * i - 50.0 * 0.914;
		}
	}

	// Main Processor /////////////////////////////////////////////////////////////////////////////
	if (rank == 0)
	{
		cout << "User-input callbacks:" << endl
			<< "press ¡®d¡¯ or ¡®D¡¯: moves the eye location down" << endl
			<< "press ¡®u¡¯ or ¡®U¡¯: moves the eye location up" << endl;
		mainOpenGL(argc, argv);
	}

	// UAV Control Processors ///////////////////////////////////////////////////////////////////////////////////////
	else
	{
		// Initial location, distance, force, and random number seed
		x = rcvBuffer[rank][0];
		y = rcvBuffer[rank][1];
		distanceShpere = sqrt(pow(x, 2) + pow(y, 2) + pow(z - 50.0, 2));  
		fx = -10.0 * x / distanceShpere;  
		fy = -10.0 * y / distanceShpere;
		fz = 10.0 * (50.0 - z) / distanceShpere + 10.0;
		srand(rank);  // Random number seed based on rank

		// Gather other UAVs' location before loop
		sendBuffer[0] = x;
		sendBuffer[1] = y;
		MPI_Allgather(sendBuffer, 6, MPI_DOUBLE, rcvBuffer, 6, MPI_DOUBLE, MPI_COMM_WORLD);

		// Generate 2 random force before loop
		do {
			fa = (double)(rand() % 10) * (double)(rand() % 10);
			fb = (double)(rand() % 10) * (double)(rand() % 10);
		} while (fa * fb == 0.0);

		// Sleep for 5 seconds
		this_thread::sleep_for(chrono::seconds(5));

		// Loop begin (set default loop number to 600)
		for (int i = 0; i < 600; ++i)
		{
			// Check collision
			// When collision happens, 2 UAV may stick together because they only exchange the velocity. That is a normal phenomenon.
			for (int j = 1; j < 16 && j != rank; ++j)
			{
				distanceUav = sqrt(pow(x - rcvBuffer[j][0], 2) + pow(y - rcvBuffer[j][1], 2) + pow(z - rcvBuffer[j][2], 2));
				if (distanceUav <= 1.01)
				{
					vx = rcvBuffer[j][3];
					vy = rcvBuffer[j][4];
					vz = rcvBuffer[j][5];
				}
			}

			// update location (based on old v and f)
			x += vx * 0.1 + 0.005 * fx;  
			y += vy * 0.1 + 0.005 * fy;
			z += vz * 0.1 + 0.005 * (fz - g);
			distanceShpere = sqrt(pow(x, 2) + pow(y, 2) + pow(z - 50.0, 2)); // update distance

			// Maximum Speed = 2 m/s before UAVs touch the sphere 
			if (surfaceFlag == 0 && v >= 2.0)
			{
				fx = 0.0;
				fy = 0.0;
				fz = 10.0;
			}

			// Speed down before touching the surface
			if (surfaceFlag == 0 && distanceShpere <= 11.0)
			{
				fx = x / distanceShpere;
				fy = y / distanceShpere;
				fz = (z - 50.0) / distanceShpere + 10.0;
			}

			// Movement on the sphere
			if (distanceShpere <= 10.0) surfaceFlag = 1;
			if (surfaceFlag)
			{
				// Generate Fs based on Hooke's Law
				fs = 5.0 * (distanceShpere - 10.0);
				fx = -x * fs / distanceShpere;
				fy = -y * fs / distanceShpere;
				fz = (50.0 - z) * fs / distanceShpere + 10.0;

				// Update V and Tangential force
				v = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));
				fc = (-fa * x - fb * y) / (z - 50.0);  // Make sure vector (fa,fb,fc) is tangential 
				ft = sqrt(pow(fa, 2) + pow(fb, 2) + pow(fc, 2));
				fa = fa / ft;
				fb = fb / ft;
				fc = fc / ft;
				
				// In first 1s, let UAVs fly in random direction while keeping the speed around 3.5 m/s
				countFlyTime++; 
				if (countFlyTime <= 10)
				{
					if (v < 3.5)
					{
						fx += fa;
						fy += fb;
						fz += fc;
					}
					if (v >= 3.5)
					{
						fx -= fa;
						fy -= fb;
						fz -= fc;
					}
				}

				// After first 1s, all UAVs fly towards the random direction they have chose while keeping 3.5 m/s
				else
				{
					if (v < 3.5)
					{
						fx += vx / v;
						fy += vy / v;
						fz += vz / v;
					}
					if (v >= 3.5)
					{
						fx -= vx / v;
						fy -= vy / v;
						fz -= vz / v;
					}
				}
			}

			// Update velocity
			vx += fx * 0.1;  
			vy += fy * 0.1;
			vz += (fz - g) * 0.1;
			v = sqrt(pow(vx, 2) + pow(vy, 2) + pow(vz, 2));

			// Update sendbuffer
			sendBuffer[0] = x;
			sendBuffer[1] = y;
			sendBuffer[2] = z;
			sendBuffer[3] = vx;
			sendBuffer[4] = vy;
			sendBuffer[5] = vz;

			// Communicate with main process
			MPI_Allgather(sendBuffer, 6, MPI_DOUBLE, rcvBuffer, 6, MPI_DOUBLE, MPI_COMM_WORLD);
		}
	}
	MPI_Finalize();
	this_thread::sleep_for(chrono::milliseconds(100)); // Wait a little time
	return 0;
}