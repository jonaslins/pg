/****************************************************

########## Project 1 Basis ##########

Graphics Processing - PG (if680)
Center of Informatics - CIn
Federal University of Pernambuco - UFPE

@authors
{
	Caio Lins (csnrl at cin.ufpe.br),
	Geovane Pereira (geeosp at cin.ufpe.br),
	Vinicius Emanuel (vems at cin.ufpe.br)
}

Reference for OpenGL commands: https://www.opengl.org/sdk/docs/man2/xhtml/

*****************************************************/


#ifndef _OPENGL_TUTORIAL_H_
#define _OPENGL_TUTORIAL_H_


#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <gl/glut.h>
#include <cmath>
#include <vector>


using namespace std;
using namespace cv;

#define FPS			30
#define ESC			27
#define IDLE		-2
#define MODIFIED	-1

void myreshape(GLsizei w, GLsizei h);

void mydisplay();

void handleMotion(int x, int y);

void handleMouse(int btn, int state, int x, int y);

void handleMouseMove(int mouseX, int mouseY);

void hadleKeyboard(unsigned char key, int x, int y);

void hadleSpecialKeyboard(int key, int x, int y);

void translate(double t);

void rotate(double r);

void cameraTranslate(double t);

void initi();

struct Ponto{
	GLdouble x, y, z;
	Ponto(GLdouble x = 0.0, GLdouble y = 0.0, GLdouble z = 0.0) :x(x), y(y), z(z){}
};


struct Vector1{

	GLdouble x, y, z;
	Vector1(GLdouble x = 0.0, GLdouble y = 0.0, GLdouble z = 0.0) :x(x), y(y), z(z){}


	Vector1 operator==(const Vector1& b)
	{
		return x == b.x && y == b.y && z == b.z;
	}

	Vector1 operator!=(const Vector1& b)
	{
		return  (x != b.x || y != b.y || z != b.z);
	}

	Vector1 operator+(const Vector1& b)
	{
		Vector1 vec;
		vec.x = x + b.x;
		vec.y = y + b.y;
		vec.z = z + b.z;
		return vec;
	}



	Vector1 operator-(const Vector1& b)
	{
		Vector1 vec;
		vec.x = x - b.x;
		vec.y = y - b.y;
		vec.z = z - b.z;
		return vec;
	}
	Vector1 operator*(const Vector1& b)
	{

		double a = y * b.z - z * b.y;
		double bb = z * b.x - x * b.z;
		double c = x * b.y - y * b.x;
		Vector1 vec(a, bb, c);

		return vec;
	}

	Vector1 operator*(const float b)
	{

		
		Vector1 vec(x*b, y*b, z*b);

		return vec;
	}


};




struct Ray{

	Vector1 P0, P1;
	Ray(Vector1 P0, Vector1 P1) : P0(P0), P1(P1){}
	Ray()
	{
	}

};

struct Triangle{

	Vector1 V0, V1, V2;
	Triangle(Vector1 V0, Vector1 V1, Vector1 V2) : V0(V0), V1(V1), V2(V2){}
};

int rayPolygon(Ray R, Triangle T);

#endif //_OPENGL_TUTORIAL_H_
