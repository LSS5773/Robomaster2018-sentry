#ifndef _MATRIX_H_
#define _MATRIX_H_

#include <math.h>

struct Point {
	float x;
	float y;
	float z;
};

float norm(struct Point p);
float dot(struct Point p0, struct Point p1);
float a2r(float angle);
struct Point multiply(float matrix[][4], struct Point pr);
void multiply_matrix(float out[][4], float in[][4]);
void matrix_Identity(float matrix[][4]);
void matrix_Rx(float matrix[][4], float angle);
void matrix_Ry(float matrix[][4], float angle);
void matrix_Rz(float matrix[][4], float angle);
void rotate_x(float matrix[][4], float angle);
void rotate_y(float matrix[][4], float angle);
void rotate_z(float matrix[][4], float angle);
void trans(float matrix[][4], float x, float y, float z);

#endif
