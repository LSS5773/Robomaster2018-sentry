#include "matrix.h"
// 2018-05-05 by LSS

float norm(struct Point p) { return sqrt(p.x*p.x+p.y*p.y+p.z*p.z); }
float dot(struct Point p0, struct Point p1) { return p0.x*p1.x+p0.y*p1.y+p0.z*p1.z; }

float a2r(float angle) { return angle*3.1415926f/180.0f; }
struct Point multiply(float matrix[][4], struct Point pr) {
	float r[4], w[3];
	r[0] = pr.x;
	r[1] = pr.y;
	r[2] = pr.z;
	r[3] = 1;
	for(int i=0; i<3; i++) w[i] = matrix[i][0]*r[0] + matrix[i][1]*r[1] + matrix[i][2]*r[2] + matrix[i][3]*r[3];
	struct Point pw;
	pw.x = w[0];
	pw.y = w[1];
	pw.z = w[2];
	return pw;
}
void multiply_matrix(float out[][4], float in[][4]) {
	for(int i=0; i<4; i++) {
		float row[4];
		for(int j=0; j<4; j++) row[j] = out[i][j];
		for(int j=0; j<4; j++) out[i][j] = row[0]*in[0][j] + row[1]*in[1][j] + row[2]*in[2][j] + row[3]*in[3][j];
	}
}
void matrix_Identity(float matrix[][4]) {
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			if(i == j) matrix[i][j] = 1;
			else matrix[i][j] = 0;
}
void matrix_Rx(float matrix[][4], float angle) {
	float radian = a2r(angle);
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			matrix[i][j] = 0;
	matrix[0][0] = 1;
	matrix[1][1] = cos(radian);
	matrix[1][2] = -sin(radian);
	matrix[2][1] = sin(radian);
	matrix[2][2] = cos(radian);
	matrix[3][3] = 1;
}
void matrix_Ry(float matrix[][4], float angle) {
	float radian = a2r(angle);
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			matrix[i][j] = 0;
	matrix[0][0] = cos(radian);
	matrix[0][2] = sin(radian);
	matrix[1][1] = 1;
	matrix[2][0] = -sin(radian);
	matrix[2][2] = cos(radian);
	matrix[3][3] = 1;
}
void matrix_Rz(float matrix[][4], float angle) {
	float radian = a2r(angle);
	for(int i=0; i<4; i++)
		for(int j=0; j<4; j++)
			matrix[i][j] = 0;
	matrix[0][0] = cos(radian);
	matrix[0][1] = -sin(radian);
	matrix[1][0] = sin(radian);
	matrix[1][1] = cos(radian);
	matrix[2][2] = 1;
	matrix[3][3] = 1;
}
void rotate_x(float matrix[][4], float angle) {
	float radian = a2r(angle);
	float R[4][4];
	matrix_Rx(R, angle);
	multiply_matrix(matrix, R);
}
void rotate_y(float matrix[][4], float angle) {
	float radian = a2r(angle);
	float R[4][4];
	matrix_Ry(R, angle);
	multiply_matrix(matrix, R);
}
void rotate_z(float matrix[][4], float angle) {
	float radian = a2r(angle);
	float R[4][4];
	matrix_Rz(R, angle);
	multiply_matrix(matrix, R);
}
void trans(float matrix[][4], float x, float y, float z) {
	float T[4][4];
	matrix_Identity(T);
	T[0][3] = x;
	T[1][3] = y;
	T[2][3] = z;
	multiply_matrix(matrix, T);
}

