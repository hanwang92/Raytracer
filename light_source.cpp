#include <cmath>
#include "light_source.h"

void PointLight::shade( Ray3D& ray ) {
	// It is assumed at this point that the intersection information in ray 
	// is available.  So be sure that traverseScene() is called on the ray 
	// before this function.  

	// Calculate normalized ray vectors
	Vector3D N = ray.intersection.normal;
	N.normalize();
	Vector3D L = _pos - ray.intersection.point;
	L.normalize();
	Vector3D V = ray.dir;
	V.normalize();
	Vector3D R = (2 * (-N.dot(L)) * N + L);
	R.normalize();
    
	// Calculate diffuse, specular, ambient lights
	Colour diffuse  = std::max(0.0,N.dot(L)) * ray.intersection.mat->diffuse * _col_diffuse;
	Colour specular = pow(std::max(0.0,V.dot(R)),ray.intersection.mat->specular_exp) * ray.intersection.mat->specular * _col_specular;
	Colour ambient  = ray.intersection.mat->ambient * _col_ambient;
	
	// Signature
	//ray.col = ray.intersection.mat->diffuse;
	
	// Diffuse
	//ray.col = diffuse + ambient;
	
	// Phong Shading
	ray.col = diffuse + ambient + specular;
	
	ray.col.clamp();
}

