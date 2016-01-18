/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		Implementations of functions in raytracer.h, 
		and the main function which specifies the 
		scene to be rendered.	

***********************************************************/


#include "raytracer.h"
#include "bmp_io.h"
#include <cmath>
#include <iostream>
#include <cstdlib>

// Feature toggle
bool SHADOWS_ENABLED = true;
bool REFLECTION_ENABLED = true;
bool ANTIALIASING_ENABLED = true;
bool SOFTSHADOWS_ENABLED = false;

Raytracer::Raytracer() : _lightSource(NULL) {
	_root = new SceneDagNode();
}

Raytracer::~Raytracer() {
	delete _root;
}

SceneDagNode* Raytracer::addObject( SceneDagNode* parent, 
		SceneObject* obj, Material* mat ) {
	SceneDagNode* node = new SceneDagNode( obj, mat );
	node->parent = parent;
	node->next = NULL;
	node->child = NULL;
	
	// Add the object to the parent's child list, this means
	// whatever transformation applied to the parent will also
	// be applied to the child.
	if (parent->child == NULL) {
		parent->child = node;
	}
	else {
		parent = parent->child;
		while (parent->next != NULL) {
			parent = parent->next;
		}
		parent->next = node;
	}
	
	return node;;
}

LightListNode* Raytracer::addLightSource( LightSource* light ) {
	LightListNode* tmp = _lightSource;
	_lightSource = new LightListNode( light, tmp );
	return _lightSource;
}

void Raytracer::rotate( SceneDagNode* node, char axis, double angle ) {
	Matrix4x4 rotation;
	double toRadian = 2*M_PI/360.0;
	int i;
	
	for (i = 0; i < 2; i++) {
		switch(axis) {
			case 'x':
				rotation[0][0] = 1;
				rotation[1][1] = cos(angle*toRadian);
				rotation[1][2] = -sin(angle*toRadian);
				rotation[2][1] = sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'y':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][2] = sin(angle*toRadian);
				rotation[1][1] = 1;
				rotation[2][0] = -sin(angle*toRadian);
				rotation[2][2] = cos(angle*toRadian);
				rotation[3][3] = 1;
			break;
			case 'z':
				rotation[0][0] = cos(angle*toRadian);
				rotation[0][1] = -sin(angle*toRadian);
				rotation[1][0] = sin(angle*toRadian);
				rotation[1][1] = cos(angle*toRadian);
				rotation[2][2] = 1;
				rotation[3][3] = 1;
			break;
		}
		if (i == 0) {
		    node->trans = node->trans*rotation; 	
			angle = -angle;
		} 
		else {
			node->invtrans = rotation*node->invtrans; 
		}	
	}
}

void Raytracer::translate( SceneDagNode* node, Vector3D trans ) {
	Matrix4x4 translation;
	
	translation[0][3] = trans[0];
	translation[1][3] = trans[1];
	translation[2][3] = trans[2];
	node->trans = node->trans*translation; 	
	translation[0][3] = -trans[0];
	translation[1][3] = -trans[1];
	translation[2][3] = -trans[2];
	node->invtrans = translation*node->invtrans; 
}

void Raytracer::scale( SceneDagNode* node, Point3D origin, double factor[3] ) {
	Matrix4x4 scale;
	
	scale[0][0] = factor[0];
	scale[0][3] = origin[0] - factor[0] * origin[0];
	scale[1][1] = factor[1];
	scale[1][3] = origin[1] - factor[1] * origin[1];
	scale[2][2] = factor[2];
	scale[2][3] = origin[2] - factor[2] * origin[2];
	node->trans = node->trans*scale; 	
	scale[0][0] = 1/factor[0];
	scale[0][3] = origin[0] - 1/factor[0] * origin[0];
	scale[1][1] = 1/factor[1];
	scale[1][3] = origin[1] - 1/factor[1] * origin[1];
	scale[2][2] = 1/factor[2];
	scale[2][3] = origin[2] - 1/factor[2] * origin[2];
	node->invtrans = scale*node->invtrans; 
}

Matrix4x4 Raytracer::initInvViewMatrix( Point3D eye, Vector3D view, 
		Vector3D up ) {
	Matrix4x4 mat; 
	Vector3D w;
	view.normalize();
	up = up - up.dot(view)*view;
	up.normalize();
	w = view.cross(up);

	mat[0][0] = w[0];
	mat[1][0] = w[1];
	mat[2][0] = w[2];
	mat[0][1] = up[0];
	mat[1][1] = up[1];
	mat[2][1] = up[2];
	mat[0][2] = -view[0];
	mat[1][2] = -view[1];
	mat[2][2] = -view[2];
	mat[0][3] = eye[0];
	mat[1][3] = eye[1];
	mat[2][3] = eye[2];

	return mat; 
}

void Raytracer::traverseScene( SceneDagNode* node, Ray3D& ray ) {
	SceneDagNode *childPtr;

	// Applies transformation of the current node to the global
	// transformation matrices.
	_modelToWorld = _modelToWorld*node->trans;
	_worldToModel = node->invtrans*_worldToModel; 
	if (node->obj) {
		// Perform intersection.
		if (node->obj->intersect(ray, _worldToModel, _modelToWorld)) {
			ray.intersection.mat = node->mat;
		}
	}
	// Traverse the children.
	childPtr = node->child;
	while (childPtr != NULL) {
		traverseScene(childPtr, ray);
		childPtr = childPtr->next;
	}

	// Removes transformation of the current node from the global
	// transformation matrices.
	_worldToModel = node->trans*_worldToModel;
	_modelToWorld = _modelToWorld*node->invtrans;
}

void Raytracer::computeShading( Ray3D& ray ) {
	LightListNode* curLight = _lightSource;
	for (;;) {
		if (curLight == NULL) break;
		// Each lightSource provides its own shading function.

		// Implement shadows here if needed.
		Colour totalColor(0.0, 0.0, 0.0);

		// Shadowing
		if (SHADOWS_ENABLED) {
			Vector3D shadowDir;
			for (float i = 0 ; i < 1.5; i = i + 0.05) {
				shadowDir = curLight->light->get_position() - ray.intersection.point;
				shadowDir[0] += i;
				shadowDir[1] += i;
				shadowDir[2] += i;
				shadowDir.normalize();
				
				Point3D shadowOrigin = ray.intersection.point + 0.01*shadowDir;
				
				Ray3D shadowRay(shadowOrigin , shadowDir);
				traverseScene(_root, shadowRay);
				
				// Compute non-shadow colour
				curLight->light->shade(ray);
				
				if (!shadowRay.intersection.none) 
				{
					ray.col = (0.7f)*ray.col;
					
					/*  
					totalColor[0] = totalColor[0] + ((1.0f/40.0f))*ray.col[0];            
					totalColor[1] = totalColor[1] + ((1.0f/40.0f))*ray.col[1];            
					totalColor[2] = totalColor[2] + ((1.0f/40.0f))*ray.col[2];
					totalColor.clamp();
					ray.col = totalColor;
					*/
				}	
			}
			curLight = curLight->next;
		}
		
		else {
			curLight->light->shade(ray);
			curLight = curLight->next; 
		}
	 
	}
}

void Raytracer::initPixelBuffer() {
	int numbytes = _scrWidth * _scrHeight * sizeof(unsigned char);
	_rbuffer = new unsigned char[numbytes];
	_gbuffer = new unsigned char[numbytes];
	_bbuffer = new unsigned char[numbytes];
	for (int i = 0; i < _scrHeight; i++) {
		for (int j = 0; j < _scrWidth; j++) {
			_rbuffer[i*_scrWidth+j] = 0;
			_gbuffer[i*_scrWidth+j] = 0;
			_bbuffer[i*_scrWidth+j] = 0;
		}
	}
}

void Raytracer::flushPixelBuffer( char *file_name ) {
	bmp_write( file_name, _scrWidth, _scrHeight, _rbuffer, _gbuffer, _bbuffer );
	delete _rbuffer;
	delete _gbuffer;
	delete _bbuffer;
}


float dampFactor = 0.0;
int depth = 0;
const double EPSILON = 0.0001;
double reflectivePercentage;
double refractivePercentage;


Colour Raytracer::shadeRay( Ray3D& ray, double refractionIndex ) {
	Colour col; 
	Colour reflectColour;
	Colour refractColour;
	traverseScene(_root, ray); 
	LightListNode* curLight = _lightSource;
	
	//printf("in shadeRay");
	if (depth > 10)
		return col;
	
	// Don't bother shading if the ray didn't hit 
	// anything.
	if (!ray.intersection.none) {
		computeShading(ray); 
		col = ray.col;  
	

		// You'll want to call shadeRay recursively (with a different ray, 
		// of course) here to implement reflection/refraction effects.
		
		// Reflection
		if (REFLECTION_ENABLED) {
			if (ray.intersection.mat->specular[0] > 0 && ray.intersection.mat->specular[1] > 0 && ray.intersection.mat->specular[2] > 0) {
		
				Vector3D N = ray.intersection.normal;
				N.normalize(); 
				Vector3D D = ray.dir;
				D.normalize();
				Vector3D reflectD = -2*(D.dot(N))*N + D;
				reflectD.normalize();
				
				Ray3D reflectRay = Ray3D (ray.intersection.point + 0.01*reflectD, reflectD);
				depth += 1;
				reflectColour = shadeRay(reflectRay, refractionIndex);
			}
		}
		col = col + reflectColour;
		col.clamp();
	}

	return col; 
}	

void Raytracer::render( int width, int height, Point3D eye, Vector3D view, 
		Vector3D up, double fov, char* fileName ) {
	Matrix4x4 viewToWorld;
	_scrWidth = width;
	_scrHeight = height;
	double factor = (double(height)/2)/tan(fov*M_PI/360.0);

	initPixelBuffer();
	viewToWorld = initInvViewMatrix(eye, view, up);

	// Construct a ray for each pixel.
	for (int i = 0; i < _scrHeight; i++) {
		for (int j = 0; j < _scrWidth; j++) {
			//Antialiasing
			if (true) {
				// Multiple pixel samples are chosen within a range from the target pixel 
				Colour totalColor(0.0,0.0,0.0);
				int sample = 4;
				for (int h=0; h <sample; h++){
					for (int k=0; k<sample; k++){
			  
						double sampDist1 = (h + 1.0*rand()/RAND_MAX)/sample;
						double sampDist2 = (k + 1.0*rand()/RAND_MAX)/sample;
					
						Point3D origin(0, 0, 0);
						Point3D imagePlane;
						imagePlane[0] = (-double(width)/2+ j + sampDist1)/factor;
						imagePlane[1] = (-double(height)/2 + i + sampDist2)/factor;
						imagePlane[2] = -1;
			
						// TODO: Convert ray to world space and call 
						// shadeRay(ray) to generate pixel colour. 	
						
						Ray3D ray;
						// Convert ray to world space
						ray.origin = viewToWorld * origin;
						ray.dir = viewToWorld * (imagePlane - origin);
						ray.dir.normalize();
						
						// Sample pixel colors are summed up 
						depth = 0;
						Colour col = shadeRay(ray, 1.0);
						totalColor = totalColor + col;
					}
				}
			  
				// Divide total color by number of samples factor to get an averaged color
				Colour colour(totalColor[0]/pow(sample,2), totalColor[1]/pow(sample,2), totalColor[2]/pow(sample,2));
				colour = Colour(fmin(1.0, colour[0]), fmin(1.0, colour[1]), fmin(1.0, colour[2]));
				
				_rbuffer[i*width+j] += int(colour[0]*255);
				_gbuffer[i*width+j] += int(colour[1]*255);
				_bbuffer[i*width+j] += int(colour[2]*255);
			}
			else {
				Point3D origin(0, 0, 0);
				Point3D imagePlane;
				imagePlane[0] = (-double(width)/2 + 0.5 + j)/factor;
				imagePlane[1] = (-double(height)/2 + 0.5 + i)/factor;
				imagePlane[2] = -1;
	
				// TODO: Convert ray to world space and call 
				// shadeRay(ray) to generate pixel colour. 	
				
				Ray3D ray;
				// Convert ray to world space
				ray.origin = viewToWorld * origin;
				ray.dir = viewToWorld * (imagePlane - origin);
				ray.dir.normalize();

				depth = 0;
				Colour colour = shadeRay(ray, 1.0);
				
				_rbuffer[i*width+j] += int(colour[0]*255);
				_gbuffer[i*width+j] += int(colour[1]*255);
				_bbuffer[i*width+j] += int(colour[2]*255);
			}
			
		}
	}

	flushPixelBuffer(fileName);
}

int main(int argc, char* argv[])
{	
	// Build your scene and setup your camera here, by calling 
	// functions from Raytracer.  The code here sets up an example
	// scene and renders it from two different view points, DO NOT
	// change this if you're just implementing part one of the 
	// assignment.  
	Raytracer raytracer;
	int width = 320; 
	int height = 240;
	//int width = 640; 
	//int height = 480;

	if (argc == 3) {
		width = atoi(argv[1]);
		height = atoi(argv[2]);
	}

	// Camera parameters.
	//Point3D eye(0, 0, 1);
	Point3D eye(0, 0, -0.7);
	Vector3D view(0, 0, -1);
	Vector3D up(0, 1, 0);
	double fov = 60;

	Material test( Colour(0.3, 0.3, 0.3), Colour(0.0, 0.0, 0.0), 
			Colour(0.628281, 0.555802, 0.366065), 
			50, 1.5 );
	Material test_jade( Colour(0, 0, 0), Colour(0.54, 0.89, 0.63), 
			Colour(0.0, 0.316228, 0.316228), 
			12.8, 1.0 );
	Material test_perl( Colour(0.25, 0.20725, 0.20725), Colour(1.0, 0.829, 0.829), 
			Colour(0.0, 0.296648, 0.296648), 
			11.264, 1.0 );
	Material test_ruby( Colour(0.2745, 0.01175, 0.01175), Colour(0.61424, 0.04136, 0.04136), 
			Colour(0.0, 0.1, 0.1), 
			76.8, 1.0 );
	Material test_tin( Colour(0.105882, 0.058824, 0.113725), Colour(0.427451, 0.470588, 0.541176), 
			Colour(0.0, 0.1, 0.1), 
			9.84615, 1.0 );


	// Defines a point light source.
	raytracer.addLightSource( new PointLight(Point3D(1.2, 1.0, -1.4), Colour(0.9, 0.9, 0.9) ) );
	
	SceneDagNode* head = raytracer.addObject( new UnitSphere(), &test );
	SceneDagNode* body = raytracer.addObject( new UnitCylinder(), &test );
	SceneDagNode* wing1 = raytracer.addObject( new UnitTriangle(), &test );
	SceneDagNode* wing2 = raytracer.addObject( new UnitTriangle(), &test );
	
	SceneDagNode* plane1 = raytracer.addObject( new UnitSquare(), &test_perl );
	SceneDagNode* plane2 = raytracer.addObject( new UnitSquare(), &test_perl );
	SceneDagNode* plane3 = raytracer.addObject( new UnitSquare(), &test_perl );
	SceneDagNode* plane4 = raytracer.addObject( new UnitSquare(), &test_ruby );
	SceneDagNode* plane5 = raytracer.addObject( new UnitSquare(), &test_tin );
	//SceneDagNode* plane6 = raytracer.addObject( new UnitSquare(), &test_perl );
	
	// Apply some transformations to the unit square.
	double factor1[3] = { 0.8, 0.8, 0.8 };
	double factor2[3] = { 6.0, 5.0, 6.0 };
	double factor3[3] = { 26.0, 5.0, 26.0 };
	double body_factor[3] = { 0.5, 0.5, 1.5 };
	double wing_factor[3] = { 2.8, 1.4, 1.1 };
	
	// Angel
	//
	// Head
	raytracer.scale(head, Point3D(0, 0, 0), factor1);
	raytracer.translate(head, Vector3D(0, 1, -6));	
	 
	// Body
	raytracer.translate(body, Vector3D(0, -0.7, -4.8));	
	raytracer.rotate(body, 'x', 90);
	raytracer.scale(body, Point3D(0, 0, 0), body_factor);
	
	// Wing1
	raytracer.scale(wing1, Point3D(0, 0, 0), wing_factor);
	raytracer.translate(wing1, Vector3D(-0.3, 0, -4));
	raytracer.rotate(wing1, 'z', -20);
	raytracer.rotate(wing1, 'y', 20);
	raytracer.translate(wing1, Vector3D(1, 0, 0));
	
	// Wing2
	raytracer.scale(wing2, Point3D(0, 0, 0), wing_factor);
	raytracer.translate(wing2, Vector3D(0.3, 0, -4));
	raytracer.rotate(wing2, 'y', 180);
	raytracer.rotate(wing2, 'z', -20);
	raytracer.rotate(wing2, 'y', -20);
	raytracer.translate(wing2, Vector3D(1, 0, 0));
	
	// Background box
	//
	//plane1
	raytracer.translate(plane1, Vector3D(0, 0, -7));	
	raytracer.scale(plane1, Point3D(0, 0, 0), factor2);
	
	// plane2
	raytracer.scale(plane2, Point3D(0, 0, 0), factor2);
	raytracer.rotate(plane2, 'x', 90);
	raytracer.translate(plane2, Vector3D(0, -0.68, -0.5));
	
	// plane3
	raytracer.scale(plane3, Point3D(0, 0, 0), factor2);
	raytracer.rotate(plane3, 'x', 90);
	raytracer.translate(plane3, Vector3D(0, -0.68, 0.5));
	
	// plane4
	raytracer.scale(plane4, Point3D(0, 0, 0), factor2);
	raytracer.rotate(plane4, 'y', 90);
	raytracer.translate(plane4, Vector3D(0.68, 0, -0.5));
	
	// plane5
	raytracer.scale(plane5, Point3D(0, 0, 0), factor2);
	raytracer.rotate(plane5, 'y', 90);
	raytracer.translate(plane5, Vector3D(0.68, 0, 0.5));
	
	/*
	// plane6
	raytracer.scale(plane6, Point3D(0, 0, 0), factor2);
	raytracer.translate(plane6, Vector3D(0, 0, -0.1));
	*/


	// Render the scene, feel free to make the image smaller for
	// testing purposes.	
	//Point3D eye2(4, 2, 1);
	//Vector3D view2(-4, -2, -6);
	raytracer.render(width, height, eye, view, up, fov, "view.bmp");
	
	// Render it from a different point of view.
	//Point3D eye2(4, 2, 1);
	//Vector3D view2(-4, -2, -6);
	//raytracer.render(width, height, eye2, view2, up, fov, "view2.bmp");
	
	//Point3D eye3(8, 2, -6);
	//Vector3D view3(-4, -1, -1);
	//raytracer.render(width, height, eye3, view3, up, fov, "view3.bmp");
	
	
	return 0;
}

