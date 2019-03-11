#ifndef GPUCOORD_H
#define GPUCOORD_H

#include <string>
#include <iostream>
#include <despot/GPUcore/CudaInclude.h>

namespace despot {

/* =============================================================================
 * DvcCoord class
 * =============================================================================*/
/**
 * Used in the Unk_Navigation and MARS examples
 */
struct DvcCoord {
	int x, y;

	DEVICE DvcCoord();
	DEVICE DvcCoord(int _x, int _y);

	DEVICE DvcCoord operator*(int v) const;
	DEVICE friend DvcCoord& operator +=(DvcCoord& left, const DvcCoord& right);
	DEVICE friend const DvcCoord operator +(const DvcCoord& first, const DvcCoord& second);
	DEVICE friend bool operator ==(const DvcCoord& first, const DvcCoord& second);
	DEVICE friend bool operator !=(const DvcCoord& first, const DvcCoord& second);

	DEVICE static double EuclideanDistance(DvcCoord c1, DvcCoord c2);
	DEVICE static int ManhattanDistance(DvcCoord c1, DvcCoord c2);
	DEVICE static int DirectionalDistance(DvcCoord c1, DvcCoord c2, int direction);
};

/* =============================================================================
 * Dvc_COORD class
 * =============================================================================*/
/**
 * Used in the CarDriving example. A coordinate in Euclidean space.
 */

struct Dvc_COORD
{
  double x, y;

  DEVICE Dvc_COORD();

  DEVICE Dvc_COORD(double _x, double _y);

  DEVICE bool Valid();
  DEVICE bool operator==(Dvc_COORD rhs);

  DEVICE bool operator<(const Dvc_COORD& other);
  DEVICE bool operator!=(Dvc_COORD rhs);

  DEVICE void operator+=(Dvc_COORD offset);

  DEVICE Dvc_COORD operator+(Dvc_COORD rhs);

  DEVICE Dvc_COORD operator*(int mul);


  DEVICE static double EuclideanDistance(Dvc_COORD lhs, Dvc_COORD rhs);
  DEVICE static double ManhattanDistance(Dvc_COORD lhs, Dvc_COORD rhs);

};

/* =============================================================================
 * Dvc_Vector class
 * =============================================================================*/
/**
 * Used in the CarDriving example. A vector in both Euclidean and Polar form
 */

class Dvc_Vector
{
public:
	DEVICE Dvc_Vector();
	DEVICE Dvc_Vector(float _dw,float _dh);
	DEVICE Dvc_Vector(float angle,float length,int dummy);
	DEVICE float GetAngle();   //[-pi,pi]
	DEVICE float GetLength();
	DEVICE void GetPolar(float &angle,float &length);
	DEVICE void AdjustLength(float length);
	DEVICE void SetAngle(float angle);
	DEVICE Dvc_Vector  operator + (Dvc_Vector  vec);

	DEVICE static float DotProduct(float x1,float y1,float x2,float y2);
	DEVICE static float CrossProduct(Dvc_Vector vec1, Dvc_Vector vec2);
	DEVICE static float Norm(float x,float y);
	DEVICE static void Uniform(float x,float y,float &ux,float &uy);
	DEVICE static void AddVector(float in_angle,float in_length,float &out_angle,float &out_length);

	float dw,dh;
	float angle,length;
};




struct Dvc_Compass {
	enum {
		NORTH, EAST, SOUTH, WEST, NORTHEAST, SOUTHEAST, SOUTHWEST, NORTHWEST
	};

	DEVICE static int Opposite(int dir);
	DEVICE static bool Opposite(int dir1, int dir2);
	DEVICE static DvcCoord GetDirections(int i);
};

} // namespace despot

#endif
