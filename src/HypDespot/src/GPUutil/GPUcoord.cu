#include <despot/GPUutil/GPUcoord.h>
//#include <cstdlib>
//#include <cmath>
#include <cassert>

using namespace std;

namespace despot {

DEVICE DvcCoord::DvcCoord() :
	x(0),
	y(0) {
}
DEVICE DvcCoord::DvcCoord(int _x, int _y) :
	x(_x),
	y(_y) {
}

DEVICE DvcCoord DvcCoord::operator*(int v) const {
	return DvcCoord(this->x * v, this->y * v);
}

DEVICE DvcCoord& operator+=(DvcCoord& left, const DvcCoord& right) {
	left.x += right.x;
	left.y += right.y;
	return left;
}

DEVICE const DvcCoord operator+(const DvcCoord& first, const DvcCoord& second) {
	return DvcCoord(first.x + second.x, first.y + second.y);
}

DEVICE bool operator==(const DvcCoord& first, const DvcCoord& second) {
	return first.x == second.x && first.y == second.y;
}

DEVICE bool operator!=(const DvcCoord& first, const DvcCoord& second) {
	return first.x != second.x || first.y != second.y;
}

/*DEVICE ostream& operator<<(ostream& os, const DvcCoord& coord) {
	os << "(" << coord.x << ", " << coord.y << ")";
	return os;
}*/

/*---------------------------------------------------------------------------*/


DEVICE double DvcCoord::EuclideanDistance(DvcCoord c1, DvcCoord c2) {
	return sqrt((float)(c1.x - c2.x) * (c1.x - c2.x) + (c1.y - c2.y) * (c1.y - c2.y));
}

DEVICE int DvcCoord::ManhattanDistance(DvcCoord c1, DvcCoord c2) {
	return abs(c1.x - c2.x) + abs(c1.y - c2.y);
}

DEVICE int DvcCoord::DirectionalDistance(DvcCoord lhs, DvcCoord rhs, int direction) {
	switch (direction) {
	case Dvc_Compass::NORTH:
		return rhs.y - lhs.y;
	case Dvc_Compass::EAST:
		return rhs.x - lhs.x;
	case Dvc_Compass::SOUTH:
		return lhs.y - rhs.y;
	case Dvc_Compass::WEST:
		return lhs.x - rhs.x;
	default:
		assert(false);
		return -1;
	}
}

DEVICE Dvc_COORD::Dvc_COORD() :x(0),y(0){}

 DEVICE Dvc_COORD::Dvc_COORD(double _x, double _y) : x(_x), y(_y) {}

 DEVICE bool Dvc_COORD::Valid() {
   return x >= 0 && y >= 0;
 }

 DEVICE bool Dvc_COORD::operator==(Dvc_COORD rhs) {
   return x == rhs.x && y == rhs.y;
 }

 DEVICE bool Dvc_COORD::operator<(const Dvc_COORD& other) {
   return x < other.x || (x == other.x && y < other.y);
 }

 DEVICE bool Dvc_COORD::operator!=(Dvc_COORD rhs) {
   return x != rhs.x || y != rhs.y;
 }

 DEVICE void Dvc_COORD::operator+=(Dvc_COORD offset) {
   x += offset.x;
   y += offset.y;
 }

 DEVICE Dvc_COORD Dvc_COORD::operator+(Dvc_COORD rhs) {
   return Dvc_COORD(x + rhs.x, y + rhs.y);
 }

 DEVICE Dvc_COORD Dvc_COORD::operator*(int mul)
 {
   return Dvc_COORD(x * mul, y * mul);
 }


 DEVICE double Dvc_COORD::EuclideanDistance(Dvc_COORD lhs, Dvc_COORD rhs)
 {
   return sqrt((lhs.x - rhs.x) * (lhs.x - rhs.x) +
               (lhs.y - rhs.y) * (lhs.y - rhs.y));
 }
 DEVICE double Dvc_COORD::ManhattanDistance(Dvc_COORD lhs, Dvc_COORD rhs)
 {
   return fabs(lhs.x - rhs.x) + fabs(lhs.y - rhs.y);
 }


DEVICE Dvc_Vector::Dvc_Vector() {dw=0;dh=0;length=1;angle=0;}
DEVICE Dvc_Vector::Dvc_Vector(float _dw,float _dh) {dw=_dw;dh=_dh;}
DEVICE Dvc_Vector::Dvc_Vector(float angle,float length,int dummy)
{
	dw=length*cos(angle);
	dh=length*sin(angle);
}
DEVICE float Dvc_Vector::GetAngle()   //[-pi,pi]
{
	return atan2(dh,dw);
}
DEVICE float Dvc_Vector::GetLength()
{
	return sqrt(dh*dh+dw*dw);
}
DEVICE void Dvc_Vector::GetPolar(float &angle,float &length)
{
	angle=GetAngle();
	length=GetLength();
}
DEVICE void Dvc_Vector::AdjustLength(float length)
{
	if(GetLength()<0.1) return;   //vector length close to 0
	float rate=length/GetLength();
	dw*=rate;
	dh*=rate;
}
DEVICE void Dvc_Vector::SetAngle(float angle)
{
	if(angle>M_PI) angle-=2*M_PI;
	if(angle<M_PI) angle+=2*M_PI;
	dw=length*cos(angle);
	dh=length*sin(angle);
}


DEVICE Dvc_Vector Dvc_Vector::operator + (Dvc_Vector  vec)
{
return Dvc_Vector(dw+vec.dw,dh+vec.dh);
}

DEVICE float Dvc_Vector::DotProduct(float x1,float y1,float x2,float y2)
{
return x1*x2+y1*y2;
}

DEVICE float Dvc_Vector::CrossProduct(Dvc_Vector vec1, Dvc_Vector vec2)
{
return vec1.dw*vec2.dh-vec1.dh*vec2.dw;
}

DEVICE float Dvc_Vector::Norm(float x,float y)
{
return sqrt(x*x+y*y);
}

DEVICE void Dvc_Vector::Uniform(float x,float y,float &ux,float &uy)
{
float l=Norm(x,y);
ux=x/l;
uy=y/l;
}

DEVICE void Dvc_Vector::AddVector(float in_angle,float in_length,float &out_angle,float &out_length)
{
float out_w=in_length*cos(in_angle)+out_length*cos(out_angle);
float out_h=in_length*sin(in_angle)+out_length*sin(out_angle);
out_angle=atan2(out_h,out_w);
out_length=sqrt(out_w*out_w+out_h*out_h);
}

DEVICE DvcCoord Dvc_Compass::GetDirections(int i)
{
	switch(i)
	{
	case 0:return DvcCoord(0, 1);
	case 1:return DvcCoord(1, 0);
	case 2:return DvcCoord(0, -1);
	case 3:return DvcCoord(-1, 0);
	case 4:return DvcCoord(1, 1);
	case 5:return DvcCoord(1, -1);
	case 6:return DvcCoord(-1, -1);
	case 7:return DvcCoord(-1, 1);
	};

	return DvcCoord(0, 0);
	//const string Dvc_Compass::CompassString[] = { "North", "East", "South", "West",
	//	"NE", "SE", "SW", "NW" };
}

DEVICE int Dvc_Compass::Opposite(int dir) {
	return (dir + 2) % 4;
}

DEVICE bool Dvc_Compass::Opposite(int dir1, int dir2) {
	return GetDirections(dir1) + GetDirections(dir2) == DvcCoord(0, 0);
}

} // namespace despot
