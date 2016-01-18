/***********************************************************
     Starter code for Assignment 3

     This code was originally written by Jack Wang for
		    CSC418, SPRING 2005

		implements scene_object.h

***********************************************************/

#include <cmath>
#include <iostream>
#include "bmp_io.h"
#include "scene_object.h"
#include "util.h"

using namespace std;

bool UnitTriangle::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {
	
  	// Create ray in object space and compute its intersection point
	Ray3D ray_obj;
	ray_obj.origin = worldToModel*ray.origin;
	ray_obj.dir = worldToModel*ray.dir;
	float t = -(ray_obj.origin[2]/ray_obj.dir[2]);
	if (t <= 0) {
		return false;
	}
	float x = ray_obj.origin[0] + t*ray_obj.dir[0];
	float y = ray_obj.origin[1] + t*ray_obj.dir[1];
	Point3D intersect_p(x, y, 0.0);
	Vector3D norm(0.0, 0.0, 1.0);
	
	// Check if the intersection point falls in the unit triangle
	if ( (x <= 0.5 && x >= -0.5) && (y + x <= 0.5) && (y - x >= 0.5) && (y <= 0.5 && y >= -0.5)) {
		if (ray.intersection.none || t < ray.intersection.t_value) {
			ray.intersection.t_value = t;
			ray.intersection.point = modelToWorld*intersect_p;
			ray.intersection.normal = worldToModel.transpose()*norm;
			ray.intersection.normal.normalize();
			ray.intersection.none = false;
			return true;
		}
	}
	return false;
}

bool UnitSquare::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {
	// TODO: implement intersection code for UnitSquare, which is
	// defined on the xy-plane, with vertices (0.5, 0.5, 0), 
	// (-0.5, 0.5, 0), (-0.5, -0.5, 0), (0.5, -0.5, 0), and normal
	// (0, 0, 1).
	//
	// Your goal here is to fill ray.intersection with correct values
	// should an intersection occur.  This includes intersection.point, 
	// intersection.normal, intersection.none, intersection.t_value.   
	//
	// HINT: Remember to first transform the ray into object space  
	// to simplify the intersection test.

	// Create ray in object space and compute its intersection point
	Ray3D ray_obj;
	ray_obj.origin = worldToModel*ray.origin;
	ray_obj.dir = worldToModel*ray.dir;
	float t = -(ray_obj.origin[2]/ray_obj.dir[2]);
	if (t <= 0) {
		return false;
	}
	float x = ray_obj.origin[0] + t*ray_obj.dir[0];
	float y = ray_obj.origin[1] + t*ray_obj.dir[1];
	Point3D intersect_p(x, y, 0.0);
	Vector3D norm(0.0, 0.0, 1.0);
	
	// Check if the intersection point falls in the unit square
	if ( (x <= 0.5) && (x >= -0.5) && (y <= 0.5) && (y >= -0.5) ) {
		if ( t < ray.intersection.t_value || ray.intersection.none ) {
			ray.intersection.t_value = t;
			ray.intersection.point = modelToWorld*intersect_p;
			ray.intersection.normal = worldToModel.transpose()*norm;
			ray.intersection.none = false;
			return true;
		}
	}
	return false;
}


bool UnitSphere::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {
	// TODO: implement intersection code for UnitSphere, which is centred 
	// on the origin.  
	//
	// Your goal here is to fill ray.intersection with correct values
	// should an intersection occur.  This includes intersection.point, 
	// intersection.normal, intersection.none, intersection.t_value.   
	//
	// HINT: Remember to first transform the ray into object space  
	// to simplify the intersection test.
	
	// Create ray in object space and compute its intersection point
	Ray3D ray_obj;
	ray_obj.origin = worldToModel*ray.origin;
	ray_obj.dir = worldToModel*ray.dir;
  
	//Use quadratic formula to find t
	Point3D ctr(0, 0, 0);
	float t, t1, t2;
	float A = ray_obj.dir.dot(ray_obj.dir);
	float B = 2*(ray_obj.dir.dot(ray_obj.origin - ctr));
	float C = (ray_obj.origin - ctr).dot(ray_obj.origin - ctr) - 1;
	float D = B*B - 4*A*C;
	if ( D < 0 )
		return false;
	
	// Find appropriate t value
	t1 = (-B + sqrt(D))/(2.0 * A);
	t2 = (-B - sqrt(D))/(2.0 * A);
	
	if ( t1 < 0 && t2 < 0 )
		return false;
	else if ( t1 < 0 )
		t = t2;
	else if ( t2 < 0)
		t = t1;
	else
		t = fmin(t2,t1);

	// Find intersect point
	float x = ray_obj.origin[0] + t*ray_obj.dir[0];
	float y = ray_obj.origin[1] + t*ray_obj.dir[1];
	float z = ray_obj.origin[2] + t*ray_obj.dir[2];
	Point3D intersect_p(x, y, z);
	Vector3D norm(x, y, z);
	norm.normalize();

	if ( t < ray.intersection.t_value || ray.intersection.none ) {
		ray.intersection.t_value = t;
		ray.intersection.point = modelToWorld*intersect_p;
		ray.intersection.normal = worldToModel.transpose()*norm;
		ray.intersection.none = false;
		return true;
	}

	return false;
}


bool UnitCylinder::intersect( Ray3D& ray, const Matrix4x4& worldToModel,
		const Matrix4x4& modelToWorld ) {

	// Create ray in object space and setup variables
	Ray3D ray_obj;
	ray_obj.origin = worldToModel*ray.origin;
        ray_obj.dir = worldToModel*ray.dir;
	Point3D ctr(0,0,0);
	double t1;
	double t2;
	double t1_tmp;
	double t2_tmp;
	
	//Use quadratic formula to find intersection 
	double A = ray_obj.dir[0]*ray_obj.dir[0] + ray_obj.dir[1]*ray_obj.dir[1];
	double B = ray_obj.origin[0]*ray_obj.dir[0] + ray_obj.origin[1]*ray_obj.dir[1];
	double C = ray_obj.origin[0]*ray_obj.origin[0] + ray_obj.origin[1]*ray_obj.origin[1] - 1;
	double D = B*B - A*C;
	Point3D intersect_p;
	Vector3D norm1;
	
	//If the discriminant is negative there is no intersection
	if (D < 0)
		return false;

	// Find the solutions for t1
	t1_tmp = (-0.5 - ray_obj.origin[2])/ray_obj.dir[2];
	t2_tmp = (0.5 - ray_obj.origin[2])/ray_obj.dir[2];
	if (t1_tmp < t2_tmp){
		t1 = t1_tmp;
		Point3D norm_tmp(0, 0, -1);
		norm1 = norm_tmp - ctr;
		norm1.normalize();
	}
	else{
		t1 = t2_tmp;
		Point3D norm_tmp(0, 0, 1);
		norm1 = norm_tmp - ctr;
		norm1.normalize();
	}

	//Use t1 to check if intersects with top or bot circles
	intersect_p = ray_obj.origin + t1 * ray_obj.dir;
	if (t1*t1 < 0.001){
		return false;
	}
	
	if (intersect_p[0]*intersect_p[0] + intersect_p[1] * intersect_p[1] <= 1)
	{
		if (!ray.intersection.none > ray.intersection.t_value){
			return false;
		}
		ray.intersection.point = intersect_p;
		ray.intersection.normal = norm1;
		ray.intersection.t_value = t1;
		ray.intersection.none = false;
		return true;
	}
	
	// Find the solutions for t2 
	t1_tmp = -B/A + sqrt(D) / A;
	t2_tmp = -B/A - sqrt(D) / A;
	if (t1_tmp < 0 && t2_tmp < 0)
		return false;
	else if (t1_tmp > 0 && t2_tmp < 0)
		t2 = t1_tmp;
	else
		t2 = t2_tmp;

	 // Use t2 to check if intersects with the side
	intersect_p = ray_obj.origin + t2 * ray_obj.dir;
	if (t2 * t2 < 0.001)
		return false;
	
	if (intersect_p[2] < 0.5 && intersect_p[2] > -0.5)
	{
		if (!ray.intersection.none > ray.intersection.t_value)
			return false;
		
		ray.intersection.point = modelToWorld * intersect_p;
		Point3D norm;
		norm[0] = intersect_p[0];
		norm[1] = intersect_p[1];
		norm[2] = 0;
		ray.intersection.normal = modelToWorld * (norm - ctr);
		ray.intersection.t_value = t2;
		ray.intersection.none = false;
		return true;
	}
	else
		return false;

}

