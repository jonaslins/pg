/*
-----------------------------------------------------------------------------
OpenGL Tutorial
VOXAR Labs
Computer Science Center - CIn
Federal University of Pernambuco - UFPE
http://www.cin.ufpe.br/~voxarlabs


Referencias:
Funcoes de C/C++:
http://www.cplusplus.com/
Copia online do Red Book (OpenGL Programming Guide):
http://fly.cc.fer.hr/~unreal/theredbook/
Referencia online para os comandos OpenGL (Man pages):
http://www.opengl.org/sdk/docs/man/

-----------------------------------------------------------------------------
*/
#include <windows.h>
#include <regex>
#include <iostream>
#include <fstream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "openGL_tutorial.h"

#define F first
#define S second
#define INCREMENTO 0.075f

typedef pair<int, int> pii;
const float TO_RADS = 3.141592654f / 180.0f; // The value of 1 degree in radians

///0,5 graus
#define COS 0.99996192306417128873735516482698 //0.99984769515639123915701155881391
#define SIN 0.00872653549837393496488821397358 //0.01745240643728351281941897851632

bool g_special_keys[128] = { false };
bool g_keys[256] = { false };

GLfloat window_width = 600.0;
GLfloat window_height = 600.0;

//opencv variables
vector<double> rv(3), tv(3);
Mat rvec(rv), tvec(tv);
Mat camMatrix;
double rot[9] = { 0 };
Vec3d eav;

Mat img_object = imread("Resources\\book.png", CV_LOAD_IMAGE_GRAYSCALE);
//Mat img_scene = imread("book2.jpg", CV_LOAD_IMAGE_GRAYSCALE);
VideoCapture cap("Resources\\video.mp4");
Mat img_scene;
//-- Step 1: Detect the keypoints using SURF Detector
int minHessian = 400;
SurfFeatureDetector detector(minHessian);
std::vector<KeyPoint> keypoints_object, keypoints_scene;
SurfDescriptorExtractor extractor;
Mat descriptors_object, descriptors_scene;

//base do sistema = vetores coluna na matriz de rotação (R). obs.: a matriz NÃO está transposta
int selObj = 0;
bool fullscreen = false;

GLfloat Ri[3];
GLfloat Rj[3];
GLfloat Rk[3];



GLfloat C[3];
GLfloat T[3];



double zoom = 0.5;


double lastTime = 0.0;
GLint midWindowX = window_width / 2;         // Middle of the window horizontally
GLint midWindowY = window_height / 2;         // Middle of the window vertically

GLfloat fieldOfView = 45.0f;                 // Define our field of view (i.e. how quickly foreshortening occurs)
GLfloat neaar = 2.0f;                  // The near (Z Axis) point of our viewing frustrum (default 1.0f)
GLfloat faar = 1000.0f;               // The far  (Z Axis) point of our viewing frustrum (default 1500.0f)

// Camera rotation
GLfloat camXRot = 0.0f;
GLfloat camYRot = 0.0f;
GLfloat camZRot = 0.0f;

// Camera position
GLfloat camXPos = 0.0f;
GLfloat camYPos = 0.0f;
GLfloat camZPos = 0.0f;

// Camera movement speed
GLfloat camXSpeed = 0.0f;
GLfloat camYSpeed = 0.0f;
GLfloat camZSpeed = 0.0f;

// How fast we move (higher values mean we move and strafe faster)
GLfloat movementSpeedFactor = 3.0f;

// Hoding any keys down?
bool holdingForward = false;
bool holdingBackward = false;
bool holdingLeftStrafe = false;
bool holdingRightStrafe = false;//*****************************************************************************************************
//*****************************************************************************************************
//*****************************************************************************************************
//*****************************************************************************************************
//***********************//// MODELO ////*****

char s[100000];
float escala;

Ray R;

int quantPontos = 0;
int quantControlPoints = 0;

vector <Ponto> bezier;
vector <vector <Ponto>> controlPoints;
vector <vector <GLfloat>> weights;
vector <Ponto> nurbs;
vector <Ponto> nurbsCurve;

int valN, valM;
int avaliacoes = 100;


void girar(GLfloat x, GLfloat y, GLfloat z);
void girarCamera(GLfloat x, GLfloat y, GLfloat z, bool yox);

float toRads(const float &theAngleInDegrees)
{
	return theAngleInDegrees * TO_RADS;
}
#define BUFSIZE 512
struct obj{

	vector<Ponto> v;
	vector<Ponto> vn;
	int id;
	vector< vector<pii> > pol;
	GLdouble cr = 1, cg = 1, cb = 1; // selected color

	GLfloat oRi[3];
	GLfloat oRj[3];
	GLfloat oRk[3];

	GLfloat oC[3];
	GLfloat oT[3];


	Ponto transforma(Ponto v){
		Ponto u;
		u.x = (oRi[0] * v.x + oRj[0] * v.y + oRk[0] * v.z + oC[0]) * escala;
		u.y = (oRi[1] * v.x + oRj[1] * v.y + oRk[1] * v.z + oC[1]) * escala;
		u.z = (oRi[2] * v.x + oRj[2] * v.y + oRk[2] * v.z + oC[2]) * escala;

		return u;
	}
	Ponto transforma_n(Ponto v){
		Ponto u;
		u.x = (oRi[0] * v.x + oRj[0] * v.y + oRk[0] * v.z);
		u.y = (oRi[1] * v.x + oRj[1] * v.y + oRk[1] * v.z);
		u.z = (oRi[2] * v.x + oRj[2] * v.y + oRk[2] * v.z);

		return u;
	}
	void carregar(const char * arquivo, int i){
		printf("%d\n", i);
		id = i;
		v.clear();
		vn.clear();
		pol.clear();

		freopen(arquivo, "r", stdin);
		float x, y, z;
		int a, b;

		while (gets(s)){
			if (s[0] == 'v'){
				if (s[1] == 'n'){
					sscanf(s + 3, "%f %f %f", &x, &y, &z);
					Ponto p(x, y, z);
					vn.push_back(p);
				}
				else {
					sscanf(s + 2, "%f %f %f", &x, &y, &z);
					Ponto p(x, y, z);
					v.push_back(p);
				}
			}
			else if (s[0] == 'f'){
				int n = strlen(s);
				char t[100];
				int z = 2;
				vector<pii> f;
				while (z<n){
					sscanf(s + z, "%s", t);
					sscanf(t, "%d//%d", &a, &b);
					z += strlen(t) + 1;
					f.push_back(make_pair(a, b));
				}
				pol.push_back(f);
			}
		}
	}
	void readNURBS(string path){
		ifstream file;
		file.open(path);
		char aux = ' ';

		valN = 0;
		valM = 0;

		float pointX, pointY, pointZ, weight;

		controlPoints.clear();
		weights.clear();

		if (file.is_open()) {
			int i = 0, cont = 0;

			while (i++ < 6) {
				file >> aux;
				file.ignore(512, '\n');
			}

			file >> aux;
			file >> valN;

			file >> aux;
			file >> valM;

			i = 0;

			while (i++ < valM) {
				vector <Ponto> temp;
				vector <GLfloat> auxW;
				controlPoints.push_back(temp);
				weights.push_back(auxW);
			}

			i = 0;

			while (i++ < 2) {
				file >> aux;
				file.ignore(512, '\n');
			}

			i = valN; //*valM;

			printf("N %d\nM %d\n\n", valN, valM);

			while (i > 0) {
				file >> pointX >> pointY >> pointZ >> weight;

				controlPoints.at(cont).push_back({ pointX, pointY, pointZ });
				weights.at(cont).push_back(weight);

				Ponto p = controlPoints.at(cont).back();

				printf("%f %f %f %f\n", p.x, p.y, p.z, weights.at(cont).back());

				i--;

				if (i == 0) {
					cont++;

					if (cont < valM)
						i = valN;

					printf("#\n");
				}
			}

			file >> aux;
			file.ignore(512, '\n');
		}
		else
			perror("Erro: ");

		printf("\nQuantidade de pontos: %d\n", controlPoints.size());

		file.close();
	}

	//x∗(P1−P0) + y∗(P2−P0) = P−P0
	void calc(double x, double y, double p0, double p1, double p2){

	}

#define EPSILON  0.0000001
#define MODULUS(p) (sqrt(p.x*p.x + p.y*p.y + p.z*p.z))
#define TWOPI 6.283185307179586476925287
#define RTOD 57.2957795

	void CalcAngleSum(Ponto q)
	{


		for (int i = 0; i<pol.size(); ++i){
			int n = pol[i].size();

			double m1, m2;
			double anglesum = 0, costheta;
			Ponto p1, p2;
			anglesum = 0;
			for (int j = 0, k = (j + 1) % n; j < n; ++j, k = (k + 1) % n){

				Ponto a = v[pol[i][j].F - 1];
				Ponto a1 = v[pol[i][k].F - 1];

				/*Ponto a = transforma_n(vn[pol[i][j]S - 1]);
				Ponto a1 = transforma_n(vn[pol[i][k].S - 1]);*/

				p1.x = a.x - q.x;
				p1.y = a.y - q.y;
				p1.z = a.z - q.z;
				p2.x = a1.x - q.x;
				p2.y = a1.y - q.y;
				p2.z = a1.z - q.z;

				m1 = MODULUS(p1);
				m2 = MODULUS(p2);
				if (m1*m2 <= EPSILON){
					anglesum = TWOPI;
					break;
				}
				else
					costheta = (p1.x*p2.x + p1.y*p2.y + p1.z*p2.z) / (m1*m2);

				anglesum += acos(costheta);
			}
			if (anglesum == TWOPI) printf("angle sum: %lf\n", anglesum);

		}


	}
	void desenha(){
		// Limpa a janela e o depth buffer


		oT[0] = oRi[0] * oC[0] + oRj[0] * C[1] + oRk[0] * oC[2];
		oT[1] = oRi[1] * oC[0] + oRj[1] * C[1] + oRk[1] * oC[2];
		oT[2] = oRi[2] * oC[0] + oRj[2] * C[1] + oRk[2] * oC[2];

		double inc = 0.0001;
		//if (objSelecionado == 1)

		//printf("coeh");
		glColor3f(cr, cg, cb);


		for (int i = 0; i<pol.size(); ++i){
			glBegin(GL_POLYGON);
			//cr +=  inc;
			//if (objSelecionado == 1)

			Vector1 vec[3];


			inc += 0.0002;
			for (int j = 0; j<pol[i].size(); ++j){

				Ponto a = transforma_n(vn[pol[i][j].S - 1]);
				Ponto b = transforma(v[pol[i][j].F - 1]);
				glNormal3f(a.x, a.y, a.z);
				glVertex3f(b.x, b.y, b.z);

				/*if (pol[i].size() == 3){
				vec[j].x = v[pol[i][j].F - 1].x;
				vec[j].y = v[pol[i][j].F - 1].y;
				vec[j].z = v[pol[i][j].F - 1].z;
				vec[j].x = b.x;
				vec[j].y = b.y;
				vec[j].z = b.z;
				//printf("n %lf %lf %lf\n", vec[j].x, vec[j].y, vec[j].z);
				}
				*/

			}

			//Triangle tri(vec[0], vec[1], vec[2]);
			//printf("t %lf %lf %lf\n", vec[0].x, vec[1].x, vec[2].x);

			//Vector I(0,0,0);
			//if (rayPolygon(R, tri) > 0){

			//glColor3f(0, 0,1);
			//}
			glEnd();
		}
	}

};

obj modelos[10];
int numM = 2;

#define SMALL_NUM   0.0000001 // anything that avoids division overflow
// dot product (3D) which allows vector operations in arguments
#define dot(u,v)   ((u).x * (v).x + (u).y * (v).y + (u).z * (v).z)

// fat de i até n
unsigned long long fat(int n, int i){
	unsigned long long ret = 1;

	for (; i <= n; i++)
		ret *= i;

	return ret;
}

// binomial (20, 3)
unsigned long long binomial(int n, int i){
	return (fat(n, n - i + 1) / fat(i, 2));
}

// retorna valor para um ponto da curva referente ao parâmetro dado
GLfloat bernstein(int n, double t, int i) {
	return ((double)binomial(n, i))*pow(1 - t, n - i)*pow(t, i);
}

void evaluateBezierSurface() {
	GLfloat resBerns = 0.0;
	GLfloat coordX = 0.0, coordY = 0.0, coordZ = 0.0;

	for (double u = 0.0; u <= 1.0f; u += 1.0f / (avaliacoes + 1)) {
		for (double v = 0.0; v <= 1.0f; v += 1.0f / (avaliacoes / 2 + 1)) {
			for (int k = 1; k < valM; k++) {
				for (int l = 1; l < valN; l++) {
					resBerns = bernstein(valM, u, k) * bernstein(valN, v, l);

					coordX += resBerns * controlPoints.at(k).at(l).x;
					coordY += resBerns * controlPoints.at(k).at(l).y;
					coordZ += resBerns * controlPoints.at(k).at(l).z;
				}
			}

			bezier.push_back({ coordX, coordY, coordZ });

			coordX = 0.0;
			coordY = 0.0;
			coordZ = 0.0;
		}
	}
}

void displayNurbs() {
	GLfloat berns = 1.0;

	bezier.clear();
	nurbs.clear();
	nurbsCurve.clear();

	glPointSize(3.5);
	glColor3f(0.8, 0.3, 0.7);
	glBegin(GL_POINTS);
	for (int l = 0; l < controlPoints.size(); l++)
	for (int h = 0; h < controlPoints.at(l).size(); h++) {
		glVertex3f(controlPoints.at(l).at(h).x, controlPoints.at(l).at(h).y, controlPoints.at(l).at(h).z);
	}
	glEnd();

	evaluateBezierSurface();

	glColor3f(0.8, 1.0, 0.2);
	glLineWidth(1.5);
	glBegin(GL_LINE_STRIP);
	for (int w = 0; w < bezier.size(); w++) {
		glVertex3f(bezier.at(w).x, bezier.at(w).y, bezier.at(w).z);
	}
	glEnd();
}
int rayPolygon(Ray R, Triangle T)
{
	Vector1    u, v, n;              // triangle vectors
	Vector1    dir, w0, w;           // ray vectors
	double     r, a, b;              // params to calc ray-plane intersect

	// get triangle edge vectors and plane normal
	//printf("t %lf %lf %lf\n", T.V0.x, T.V1.x, T.V2.x);

	u = T.V1 - T.V0;
	v = T.V2 - T.V0;
	n = u * v;
	//printf("n %lf %lf %lf\n", n.x, n.y, n.z);

	Vector1 zero(0, 0, 0);// cross product
	if (n.x == 0 && n.y  && n.z)             // triangle is degenerate
		return -1;                  // do not deal with this case

	dir = R.P1 - R.P0;              // ray direction vector
	w0 = R.P0 - T.V0;
	a = -dot(n, w0);
	b = dot(n, dir);
	if (fabs(b) < SMALL_NUM) {     // ray is  parallel to triangle plane
		if (a == 0)                 // ray lies in triangle plane
			return 2;
		else return 0;              // ray disjoint from plane
	}

	// get intersect point of ray with triangle plane
	r = a / b;
	if (r < 0.0)                    // ray goes away from triangle
		return 0;                   // => no intersect
	// for a segment, also test if (r > 1.0) => no intersect

	Vector1 I = R.P0 + (dir * r);            // intersect point of ray and plane

	// is I inside T?
	float    uu, uv, vv, wu, wv, D;
	uu = dot(u, u);
	uv = dot(u, v);
	vv = dot(v, v);
	w = I - T.V0;
	wu = dot(w, u);
	wv = dot(w, v);
	D = uv * uv - uu * vv;

	// get and test parametric coords
	float s, t;
	s = (uv * wv - vv * wu) / D;
	if (s < 0.0 || s > 1.0)         // I is outside T
		return 0;
	t = (uv * wu - uu * wv) / D;
	if (t < 0.0 || (s + t) > 1.0)  // I is outside T
		return 0;

	return 1;                       // I is in T
}

void myinit()
{
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);

	//matriz de parametros 
	Ri[0] = 1.0f;
	Rj[1] = 1.0f;
	Rk[2] = 1.0f;
	Ri[1] = Ri[2] = Rj[0] = Rj[2] = Rk[0] = Rk[1] = 0;

	//posicao da camera em relação ao mundo
	C[0] = 0.0f;
	C[1] = -5.0f;
	C[2] = -30.0f;
	T[0] = T[1] = T[2] = 0;

	//objeto em relação a camera
	for (int i = 0; i < numM; i++){
		modelos[i].oRi[0] = 1.0f;
		modelos[i].oRj[1] = 1.0f;
		modelos[i].oRk[2] = 1.0f;
		modelos[i].oRi[1] = modelos[i].oRi[2] = modelos[i].oRj[0] = modelos[i].oRj[2] = modelos[i].oRk[0] = modelos[i].oRk[1] = 0;

		//onde o objeto esta em relação as coordenadas de mundo
		modelos[i].oC[0] = 0.0f;
		modelos[i].oC[1] = 0.0f;
		modelos[i].oC[2] = 0.0f;
		modelos[i].oT[0] = modelos[i].oT[1] = modelos[i].oT[2] = 0;
		escala = 1;
	}

	//initNurbs();


	GLfloat luzAmbiente[4] = { 0.2, 0.2, 0.2, 1.0 };
	GLfloat luzDifusa[4] = { 0, 1, 0, 1.0 };	   // "cor" 
	GLfloat luzEspecular[4] = { 0.8, 0.8, 0.8, 1.0 };// "brilho" 
	GLfloat posicaoLuz[4] = { 50.0, 1.0, 1.0, 0.0 };

	// Capacidade de brilho do material
	GLfloat especularidade[4] = { 1.0, 1.0, 1.0, 1.0 };
	GLint especMaterial = 70;

	// Especifica que a cor de fundo da janela será preta
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

	// Habilita o modelo de colorização de Gouraud
	glShadeModel(GL_SMOOTH);

	// Define a refletância do material 
	glMaterialfv(GL_FRONT, GL_SPECULAR, especularidade);
	// Define a concentração do brilho
	glMateriali(GL_FRONT, GL_SHININESS, especMaterial);




	// Ativa o uso da luz ambiente 
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, luzAmbiente);

	// Define os parâmetros da luz de número 0
	glLightfv(GL_LIGHT0, GL_AMBIENT, luzAmbiente);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, luzDifusa);
	glLightfv(GL_LIGHT0, GL_SPECULAR, luzEspecular);
	glLightfv(GL_LIGHT0, GL_POSITION, posicaoLuz);

	// Habilita a definição da cor do material a partir da cor corrente
	glEnable(GL_COLOR_MATERIAL);
	//Habilita o uso de iluminação
	glEnable(GL_LIGHTING);
	// Habilita a luz de número 0
	glEnable(GL_LIGHT0);
	// Habilita o depth-buffering
	glEnable(GL_DEPTH_TEST);


	GLfloat luzAmbiente2[4] = { 0, 0.2, 0.2, 1.0 };
	GLfloat luzDifusa2[4] = { 1, 0, 0, 1.0 };	   // "cor" 
	GLfloat luzEspecular2[4] = { 0.1, 1, 1, 1.0 };// "brilho" 
	GLfloat posicaoLuz2[4] = { -50.0, 1.0, 1.0, 1.0 };

	// Define os parâmetros da luz de número 1
	glLightfv(GL_LIGHT1, GL_AMBIENT, luzAmbiente2);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, luzDifusa2);
	glLightfv(GL_LIGHT1, GL_SPECULAR, luzEspecular2);
	glLightfv(GL_LIGHT1, GL_POSITION, posicaoLuz2);

	// Habilita a luz de número 1
	glEnable(GL_LIGHT1);

}

void myreshape(GLsizei w, GLsizei h)
{
	if (w == 0)
		h = 1;

	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45, (float)w / h, 0.1, 100);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void displayPalette(){
	//glClear(GL_COLOR_BUFFER_BIT);
	glDisable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glDisable(GL_TEXTURE_2D);
	glDisable(GL_LIGHTING);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho(window_width, 0.0, 0.0, window_height, -1.0, 10.0); //bottom right
	glMatrixMode(GL_MODELVIEW);
	//glPushMatrix();        ----Not sure if I need this
	glLoadIdentity();
	glDisable(GL_CULL_FACE);

	glClear(GL_DEPTH_BUFFER_BIT);


	double size = 100;
	double borderSize = 20;
	double border = size / borderSize;
	double x = border;
	double y = border;


	glColor3f(1, 1, 1);
	glBegin(GL_POLYGON);
	glColor3f(1, 1, 1);

	glVertex2f(x + size + border, y + size + border);           //top right
	glVertex2f(x + size + border, y - border);          //bottom right
	glVertex2f(x - border, y - border);         //bottom left
	glVertex2f(x - border, y + size + border); //top left
	glEnd();

	glColor3f(0, 0, 0);
	glBegin(GL_POLYGON);

	glColor3f(1, 0, 0);   //red
	glVertex2f((x + size) / 2, y + size);       //top

	glColor3f(1, .38, .01);       //orange
	glVertex2f(x + size, y + size);           //top right

	glColor3f(1, 1, 0);           //yellow
	glVertex2f(x + size, y);          //bottom right

	glColor3f(0, 1, 0);           //green
	glVertex2f((x + size) / 2, y);           //bottom

	glColor3f(0, 0, 1);           //blue
	glVertex2f(x, y);         //bottom left

	glColor3f(.8, 0, .8);         //purple
	glVertex2f(x, y + size);          //top left
	glEnd();


	// Making sure we can render 3d again
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
}

int mousex, mousey;

// Function to deal with mouse position changes, called whenever the mouse cursorm moves
double mX = 0, mY = 0;

void handleMouseMove(int mouseX, int mouseY)
{
	int dx = mouseX - midWindowX;
	int dy = mouseY - midWindowY;
	mY = dy;
	printf("dx %d dy %d\n", dx, dy);
	static bool wrap = false;
	double v = 0.2;
	if (!wrap) {


		// Do something with dx and dy here

		girarCamera(0, dx*v, 0, true);
		girarCamera(dy*v, 0, 0, false);


		// move mouse pointer back to the center of the window
		wrap = true;
		glutWarpPointer(midWindowX, midWindowX);
	}
	else {
		wrap = false;
	}

}

void displayCubes(){
	glColor3f(1, 1, 1);
	glPushMatrix();
	glTranslatef(0.0f, 0.0f, -30.0f);
	glutSolidCube(4);
	glPopMatrix();
	/*
	glPushMatrix();
	glTranslatef(-10.0f, 0.0f, 0.0f);
	glutSolidCube(4);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(10.0f, 0.0f, 0.0f);
	glutSolidCube(4);
	glPopMatrix();*/

	glPushMatrix();
	glTranslatef(-10.0f, 0.0f, -30.0f);
	glutSolidCube(4);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(10.0f, 0.0f, -30.0f);
	glutSolidCube(4);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(-20.0f, 0.0f, -30.0f);
	glutSolidCube(4);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(20.0f, 0.0f, -30.0f);
	glutSolidCube(4);
	glPopMatrix();

}

void displayGridAndAxis(){

	//render grid
	glLineWidth(1);
	glColor3f(1, 1, 1);
	glBegin(GL_LINES);
	for (int i = -30; i <= 30; ++i) {
		glVertex3f(i, -0.1, -30);
		glVertex3f(i, -0.1, 30);

		glVertex3f(30, -0.1, i);
		glVertex3f(-30, -0.1, i);
	}
	glEnd();


	//render world axis
	glPushMatrix();
	glLineWidth(3);
	glColor3f(1, 0, 0); //RED = x
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(5, 0, 0);
	glEnd();
	glColor3f(0, 1, 0); //GREEN = y
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 5, 0);
	glEnd();
	glColor3f(0, 0, 1); //BLUE = z
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, 5);
	glEnd();
	glPopMatrix();

}


void runOpenCV(){

	cap >> img_scene;
	

	if (cap.get(CV_CAP_PROP_POS_FRAMES) == 410)
	{
		cap.set(CV_CAP_PROP_POS_FRAMES, 5);
	}
	if (img_scene.empty())
		return;

	Mat gray;

	// convert RGB image to gray
	cvtColor(img_scene, img_scene, CV_BGR2GRAY);
	detector.detect(img_scene, keypoints_scene);
	extractor.compute(img_scene, keypoints_scene, descriptors_scene);


	//-- Step 3: Matching descriptor vectors using FLANN matcher
	FlannBasedMatcher matcher;
	std::vector< DMatch > matches;
	matcher.match(descriptors_object, descriptors_scene, matches);

	double max_dist = 0; double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	for (int i = 0; i < descriptors_object.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	//printf("-- Max dist : %f \n", max_dist);
	//printf("-- Min dist : %f \n", min_dist);

	//-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
	std::vector< DMatch > good_matches;

	for (int i = 0; i < descriptors_object.rows; i++)
	{
		if (matches[i].distance < 3 * min_dist)
		{
			good_matches.push_back(matches[i]);
		}
	}

	Mat img_matches;
	drawMatches(img_object, keypoints_object, img_scene, keypoints_scene,
		good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
		vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	//-- Localize the object
	std::vector<Point2f> obj;
	std::vector<Point2f> scene;

	for (int i = 0; i < good_matches.size(); i++)
	{
		//-- Get the keypoints from the good matches
		obj.push_back(keypoints_object[good_matches[i].queryIdx].pt);
		scene.push_back(keypoints_scene[good_matches[i].trainIdx].pt);
	}
	//ERRO AQUIIIIII NAO EXISTEM MATCHES
	Mat H = findHomography(obj, scene, CV_RANSAC);

	//-- Get the corners from the image_1 ( the object to be "detected" )
	vector<Point2f> obj_corners(4);
	obj_corners[0] = cvPoint(0, 0); obj_corners[1] = cvPoint(img_object.cols, 0);
	obj_corners[2] = cvPoint(img_object.cols, img_object.rows); obj_corners[3] = cvPoint(0, img_object.rows);
	vector<Point2f> scene_corners(4);

	perspectiveTransform(obj_corners, scene_corners, H);

	//-- Draw lines between the corners (the mapped object in the scene - image_2 )
	line(img_matches, scene_corners[0] + Point2f(img_object.cols, 0), scene_corners[1] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
	line(img_matches, scene_corners[1] + Point2f(img_object.cols, 0), scene_corners[2] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
	line(img_matches, scene_corners[2] + Point2f(img_object.cols, 0), scene_corners[3] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
	line(img_matches, scene_corners[3] + Point2f(img_object.cols, 0), scene_corners[0] + Point2f(img_object.cols, 0), Scalar(0, 255, 0), 4);
	
	//-- Show detected matches
	imshow("Good Matches & Object detection", img_matches);
	
	
	// CODIGO QUE TAVA DANDO ERRO QUE RODRIGO PEDIU PRA IGNORAR

	/**
	rvec = Mat(rv);
	double _d[9] = { 1, 0, 0, 0, -1, 0, 0, 0, -1 };
	Rodrigues(Mat(3, 3, CV_64FC1, _d), rvec);
	tv[0] = 0; tv[1] = 0; tv[2] = 1;
	tvec = Mat(tv);

	
	double _cm[9] = { 20, 0, 160,
	0, 20, 120,
	0, 0, 1 };
	camMatrix = Mat(3, 3, CV_64FC1, _cm);

	
	
	// MATRIZ DE PARAMETROS INTRINSECOS
	double _cm[9] = { 350.47574, 0.00000, 158.25000,
		0.00000, 363.04709, 120.75000,
		0.00000, 0.00000, 1.00000 };

	camMatrix = Mat(3, 3, CV_64FC1, _cm);

	double _dc[] = { 0, 0, 0, 0 };	
	
	
	solvePnPRansac(obj, scene, camMatrix, Mat(1, 4, CV_64FC1, _dc), rvec, tvec, true);
	//std::cout<<tvec;

	Mat rotM(3, 3, CV_64FC1, rot);
	Rodrigues(rvec, rotM);
	double* _r = rotM.ptr<double>();
	printf("rotation mat: \n %.3f %.3f %.3f\n%.3f %.3f %.3f\n%.3f %.3f %.3f\n",
		_r[0], _r[1], _r[2], _r[3], _r[4], _r[5], _r[6], _r[7], _r[8]);


	Mat tmp, tmp1, tmp2, tmp3, tmp4, tmp5;
	double _pm[12] = { _r[0], _r[1], _r[2], 0,
		_r[3], _r[4], _r[5], 0,
		_r[6], _r[7], _r[8], 0 };
	decomposeProjectionMatrix(Mat(3, 4, CV_64FC1, _pm), tmp, tmp1, tmp2, tmp3, tmp4, tmp5, eav);

	printf("euler angles: %.5f %.5f %.5f\n", eav[0], eav[1], eav[2]);


	*/



}

void mydisplay()
{

	runOpenCV();

	//GLUUUUT
	/*-----------------------------------------------------------------------------------------------*/
	/*-----------------------------------------------------------------------------------------------*/
	/*-----------------------------------------------------------------------------------------------*/
	/*A IDEIA É SUBSTITUIR AS MATRIZES Ri Rj Rk por _r[0...8] e T[] por tvec*/
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_TEXTURE_2D);
	glEnable(GL_LIGHTING);

	glClearColor(0, 0, 0, 0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	//calculando R*C

	T[0] = Ri[0] * C[0] + Rj[0] * C[1] + Rk[0] * C[2];
	T[1] = Ri[1] * C[0] + Rj[1] * C[1] + Rk[1] * C[2];
	T[2] = Ri[2] * C[0] + Rj[2] * C[1] + Rk[2] * C[2];




	//obs.: tanto R quanto T estão transpostas por conta do formato de modelview exigido pelo OpenGL
	GLfloat parametrosExtrinsecos[16] = {
		Ri[0], Ri[1], Ri[2], 0.0f,
		Rj[0], Rj[1], Rj[2], 0.0f,
		Rk[0], Rk[1], Rk[2], 0.0f,
		T[0], T[1], T[2], 1.0f };



	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(parametrosExtrinsecos);

	glRotatef(mY, 0.0f, 1.0f, 0.0f);


	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	GLfloat aspectRatio = (window_width > window_height) ? float(window_width) / float(window_height) : float(window_height) / float(window_width);
	GLfloat fH = tan(float(fieldOfView / 2)) * neaar;
	GLfloat fW = fH * aspectRatio;
	glFrustum(-1, 1, -1, 1, neaar, faar);

	displayGridAndAxis();
	displayCubes();
	for (int i = 0; i < numM; i++)
		modelos[i].desenha();
	displayPalette();
	//displayNurbs();
	glFlush();
	glutPostRedisplay();
}
//n usa
void norma(GLfloat * a){
	float sq = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
	a[0] *= sq;
	a[1] *= sq;
	a[2] *= sq;
}

void girarCamera(GLfloat x, GLfloat y, GLfloat z, bool yox){

	GLfloat tt[3][3], uu[3][3], vv[3][3];

	/*if (x == y) x += 0.000001, y += 0.0002;
	if (x == z) x += 0.000003, z += 0.00004;
	if (y == z) y += 0.00005, z += 0.006;*/

	tt[0][0] = x*x*(1 - COS) + COS;		tt[0][1] = x*y*(1 - COS) - SIN*z;	tt[0][2] = x*z*(1 - COS) + SIN*y;
	tt[1][0] = y*x*(1 - COS) + SIN*z;	tt[1][1] = y*y*(1 - COS) + COS;		tt[1][2] = y*z*(1 - COS) - SIN*x;
	tt[2][0] = z*x*(1 - COS) - SIN*y;	tt[2][1] = z*y*(1 - COS) + SIN*x;	tt[2][2] = z*z*(1 - COS) + COS;
	//if (yox)
	for (int i = 0; i<3; ++i)
		uu[i][0] = Ri[i],//xglobal
		uu[i][1] = Rj[i],
		uu[i][2] = Rk[i];
	/*else
	for (int i = 0; i<3; ++i)
	uu[i][0] = Ri[i],//xglobal
	uu[i][1] = Rj[i],
	uu[i][2] = Rk[i];*/

	//vv=tt*uu
	for (int i = 0; i<3; ++i)
	for (int j = 0; j<3; ++j){
		vv[i][j] = 0;
		for (int k = 0; k<3; ++k)
			vv[i][j] += tt[i][k] * uu[k][j];
	}

	for (int i = 0; i<3; ++i)
		Ri[i] = vv[i][0],
		Rj[i] = vv[i][1],
		Rk[i] = vv[i][2];


	/*T[0] = Ri[0] * C[0] + Rj[0] * C[1] + Rk[0] * C[2];
	T[1] = Ri[1] * C[0] + Rj[1] * C[1] + Rk[1] * C[2];
	T[2] = Ri[2] * C[0] + Rj[2] * C[1] + Rk[2] * C[2];

	*/


	//obs.: tanto R quanto T estão transpostas por conta do formato de modelview exigido pelo OpenGL
	/*GLfloat parametrosExtrinsecos[16] = {
	Ri[0], Ri[1], Ri[2], 0.0f,
	Rj[0], Rj[1], Rj[2], 0.0f,
	Rk[0], Rk[1], Rk[2], 0.0f,
	T[0], T[1], T[2], 1.0f };
	//C[0],  C[1],  C[2],  1.0f};
	//T[0]+C[0], T[1]+C[1],  T[2]+C[2],  1.0f};

	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(parametrosExtrinsecos);
	glFlush();
	glutPostRedisplay();*/

}

void girar(GLfloat x, GLfloat y, GLfloat z){

	GLfloat tt[3][3], uu[3][3], vv[3][3];


	tt[0][0] = x*x*(1 - COS) + COS;		tt[0][1] = x*y*(1 - COS) - SIN*z;	tt[0][2] = x*z*(1 - COS) + SIN*y;
	tt[1][0] = y*x*(1 - COS) + SIN*z;	tt[1][1] = y*y*(1 - COS) + COS;		tt[1][2] = y*z*(1 - COS) - SIN*x;
	tt[2][0] = z*x*(1 - COS) - SIN*y;	tt[2][1] = z*y*(1 - COS) + SIN*x;	tt[2][2] = z*z*(1 - COS) + COS;

	for (int i = 0; i<3; ++i)
		uu[i][0] = modelos[selObj].oRi[i],
		uu[i][1] = modelos[selObj].oRj[i],
		uu[i][2] = modelos[selObj].oRk[i];

	//vv=tt*uu
	for (int i = 0; i<3; ++i)
	for (int j = 0; j<3; ++j){
		vv[i][j] = 0;
		for (int k = 0; k<3; ++k)
			vv[i][j] += tt[i][k] * uu[k][j];
	}

	for (int i = 0; i<3; ++i)
		modelos[selObj].oRi[i] = vv[i][0],
		modelos[selObj].oRj[i] = vv[i][1],
		modelos[selObj].oRk[i] = vv[i][2];
}

void teclado(unsigned char key, int, int) {
	g_keys[(int)key] = true;
	//aumentar objeto
	if (key == '=' || key == '+'){
		escala = escala * 1.1;


	}
	//diminuir objeto
	if (key == '_' || key == '-'){
		escala = escala * 0.9;

	}


	//girar objeto no eixo x
	if (key == '7'){
		girar(1.0f, 0.0f, 0.0f);

	}
	//girar objeto no eixo y
	if (key == '8'){
		girar(0.0f, 1.0f, 0.0f);

	}
	//girar objeto no eixo z
	if (key == '9'){
		girar(0.0f, 0.0f, 1.0f);

	}
	//transladar objeto no eixo x+ 2
	if (key == '2'){
		modelos[selObj].oC[0] = modelos[selObj].oC[0] + INCREMENTO;


	}
	//transladar objeto no eixo x- 1
	if (key == '1'){
		modelos[selObj].oC[0] = modelos[selObj].oC[0] - INCREMENTO;

	}
	//transladar objeto no eixo y+ 4
	if (key == '4'){
		modelos[selObj].oC[1] = modelos[selObj].oC[1] + INCREMENTO;

	}
	//transladar objeto no eixo y- 3
	if (key == '3'){
		modelos[selObj].oC[1] = modelos[selObj].oC[1] - INCREMENTO;

	}
	//transladar objeto no eixo z+ 6
	if (key == '6'){
		modelos[selObj].oC[2] = modelos[selObj].oC[2] + INCREMENTO;

	}
	//transladar objeto no eixo z- 5
	if (key == '5'){
		modelos[selObj].oC[2] = modelos[selObj].oC[2] - INCREMENTO;

	}

	//, ou < selecionar obj anterior
	if (key == ',' || key == '<'){
		selObj = ((--selObj + numM) % numM);
	}
	//. ou > selecionar proximo obj
	if (key == '.' || key == '>'){
		selObj = (++selObj % numM);
	}

	//camera para frente
	if (key == 'w'){
		C[0] += Ri[2] * INCREMENTO;
		C[1] += Rj[2] * INCREMENTO;
		C[2] += Rk[2] * INCREMENTO;
	}
	//camera para esquerda
	if (key == 'a'){
		C[0] += Ri[0] * INCREMENTO;
		C[1] += Rj[0] * INCREMENTO;
		C[2] += Rk[0] * INCREMENTO;
	}
	//camera para tras
	if (key == 's'){
		C[0] -= Ri[2] * INCREMENTO;
		C[1] -= Rj[2] * INCREMENTO;
		C[2] -= Rk[2] * INCREMENTO;
	}
	//camera para direita
	if (key == 'd'){
		C[0] -= Ri[0] * INCREMENTO;
		C[1] -= Rj[0] * INCREMENTO;
		C[2] -= Rk[0] * INCREMENTO;
	}

	if (key == 'j'){
		C[2] = C[2] - (15 * INCREMENTO);
		neaar += INCREMENTO;
		printf("%f \n %f \n", C[2], neaar);

	}

	if (key == 'n'){
		if (C[2] < -21){
			C[2] = C[2] + (15 * INCREMENTO);
			neaar -= INCREMENTO;
		}
		printf("%f \n %f \n", C[2], neaar);

	}
	if (key == 'f'){
		if (!fullscreen){
			glutFullScreen();
			fullscreen = true;
		}
		else if (fullscreen){
			glutReshapeWindow(1200, 900);
			glutPositionWindow(0, 0);
			fullscreen = false;
		}
	}

	//Sleep(10);
	glutPostRedisplay();
}

void handleMouse(int btn, int state, int x, int y){
	printf("%d, %d ", btn, state);

	if (GLUT_MIDDLE_BUTTON == GLUT_DOWN)
	{
		printf("asdfasdf");


	}
	if (btn == GLUT_LEFT_BUTTON && state == GLUT_DOWN){
		GLint viewport[4];
		GLdouble modelview[16];
		GLdouble projection[16];
		GLfloat winX, winY, winZ;
		GLdouble posX, posY, posZ;

		glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
		glGetDoublev(GL_PROJECTION_MATRIX, projection);
		glGetIntegerv(GL_VIEWPORT, viewport);

		winX = (float)x;
		winY = (float)viewport[3] - (float)y;
		glReadPixels(x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);

		gluUnProject(winX, winY, neaar, modelview, projection, viewport, &posX, &posY, &posZ);
		//Ponto perto(posX, posY, posZ);
		printf("\nnear %lf %lf %lf\n", posX, posY, posZ);
		Vector1 perto(posX, posY, posZ);
		gluUnProject(winX, winY, faar, modelview, projection, viewport, &posX, &posY, &posZ);
		printf("far %lf %lf %lf\n", posX, posY, posZ);
		Vector1 longe(posX, posY, posZ);

		R.P0 = perto;
		R.P1 = longe;


		unsigned char current[3];

		glReadPixels(x, window_height - y, 1, 1, GL_RGB, GL_UNSIGNED_BYTE, current);

		double r = (double)current[0] / 255;
		double g = (double)current[1] / 255;
		double b = (double)current[2] / 255;
		printf("(%d, %d) >> %lf %lf %lf\n", x, y, r, g, b);


		if (x >= 490 && y >= 490) {
			modelos[selObj].cr = r; modelos[selObj].cg = g; modelos[selObj].cb = b;
		}


	}

}

int main(int argc, char **argv)
{

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
	glutInitWindowSize(window_width, window_height);
	glutCreateWindow("Projeto 1 - PG 2014.2");

	char objStr[10][20] = { "lion.obj", "camel.obj", "whale.obj", "lion.obj", "lion.obj", "lion.obj", "lion.obj", "lion.obj", "lion.obj" };

	for (int i = 0; i < numM; i++)
		modelos[i].carregar(objStr[i], i);

	detector.detect(img_object, keypoints_object);

	//-- Step 2: Calculate descriptors (feature vectors)
	extractor.compute(img_object, keypoints_object, descriptors_object);

	glutDisplayFunc(mydisplay);
	glutReshapeFunc(myreshape);
	glutKeyboardFunc(teclado);
	glutMouseFunc(handleMouse);
	glutMotionFunc(handleMouseMove);

	myinit();

	glutMainLoop();
	return 0;
}
