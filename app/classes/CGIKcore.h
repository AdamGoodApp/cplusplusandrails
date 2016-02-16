#pragma once
#include <math.h>
#define _USE_MATH_DEFINES

namespace MathTypes
{
	 struct Constants
	{
	public:
		static double pi;// = 3.1415926535897932384626433832795;
		static double half_pi;// = pi / 2;
		static double two_pi;// = pi * 2;
		static double deg2rad;// = pi / 180;

		//bot constants
		static double BOTGEO_EE;// = 34.221; //42.58; //distance between wrist & ee
		static double BOTGEO_Z;// = 189.165; //151.769; //z-height of axis-2 from work surface
		static double BOTGEO_BICEP;// = 300;
		static double BOTGEO_ARM;// = 252.42; //distance between wrist & elbow centres
	};

	 struct AInterval
	{
	public:
		double t0;
		double t1;
		AInterval();
		AInterval(double T0, double T1);
		double Min();
		double Max();
		bool includesParameter(double val);

		static AInterval A1_RNG;
		static AInterval A2_RNG;
		static AInterval A3_RNG;
		static AInterval A4_RNG;
		static AInterval A5_RNG;
		static AInterval A6_RNG;
	};
	
	 struct IKres
	{
	public:
		double a0;
		double a1;
		double a2;
		double a3;
		double a4;
		double a5;
		int reachScore;
		int errorCode;
		int warningCode;
		IKres();
		IKres(double A0, double A1, double A2, double A3, double A4, double A5, int ReachScore, int ErrorCode, int WarningCode);
	};

}

namespace GeometryTypes
{
	 struct APoint
	{
	public:
		double x;
		double y;
		double z;
		APoint();
		APoint(double X, double Y, double Z);
		bool isValid();
		double DistanceTo(APoint pt);

		static APoint origin;
		static APoint invalid;
	};

	 struct AVector
	{
	public:
		double x;
		double y;
		double z;
		AVector();
		AVector(double X, double Y, double Z);
		AVector(APoint from, APoint to);
		double Length();
		AVector Normalize();
		AVector Rotate(double angle, AVector axis);
		bool isValid();

		static AVector XAxis;
		static AVector YAxis;
		static AVector ZAxis;
		static AVector invalid;
	};

	 struct APlane
	{
	public:
		APoint origin;
		AVector xAxis;
		AVector yAxis;
		AVector zAxis;
		APlane();
		APlane(APoint origin, AVector xAxis, AVector yAxis);
		APlane(double originX, double originY, double originZ, double rotX, double rotY, double rotZ);
		bool isValid();
		APlane Rotate(double angle, AVector axis);
		double DistanceTo(APoint pt);
		APoint ClosestPoint(APoint pt);
		APoint PointAt(double u, double v);
		APoint PointAt(double u, double v, double w);

		static APlane worldXY;
		static APlane worldYZ;
		static APlane worldXZ;
		static APlane invalid;
	};

	 struct ALine
	{
	public:
		APoint st;
		APoint en;
		ALine();
		ALine(APoint StartPt, APoint EndPt);
		bool isValid();
		AVector Direction();
		static ALine invalid;
	};

	 struct ACircle
	{
	public:
		APlane plane;
		double radius;
		ACircle();
		ACircle(double Radius);
		ACircle(APlane Plane, double Radius);
		ACircle(APlane Plane, APoint Centre, double Radius);
		bool isValid();

		static ACircle invalid;
	};

	//-------------------------------------------------------------

	inline APoint operator +(APoint p0, APoint p1)
	{
		return APoint(p0.x + p1.x, p0.y + p1.y, p0.z + p1.z);
	}

	inline APoint operator +(APoint p, AVector v)
	{
		return APoint(p.x + v.x, p.y + v.y, p.z + v.z);
	}

	inline APoint operator -(APoint p, AVector v)
	{
		return APoint(p.x - v.x, p.y - v.y, p.z - v.z);
	}

	inline APoint operator *(APoint p, double val)
	{
		return APoint(p.x*val, p.y*val, p.z*val);
	}

	inline APoint operator /(APoint p, double val)
	{
		return APoint(p.x / val, p.y / val, p.z / val);
	}

	inline AVector operator -(APoint to, APoint from)
	{
		return AVector(to.x - from.x, to.y - from.y, to.z - from.z);
	}

	inline AVector operator +(AVector v1, AVector v2)
	{
		return AVector(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
	}

	inline AVector operator -(AVector v1, AVector v2)
	{
		return AVector(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
	}

	inline AVector operator -(AVector v)
	{
		return AVector(-v.x, -v.y, -v.z);
	}

	inline AVector operator /(AVector v, double val)
	{
		return AVector(v.x / val, v.y / val, v.z / val);
	}

	inline AVector operator *(AVector v, double val)
	{
		return AVector(v.x * val, v.y * val, v.z * val);
	}

	inline double VectorAngle(AVector vA, AVector vB)
	{
		vA = vA.Normalize();
		vB = vB.Normalize();

		return acos(vA.x * vB.x + vA.y * vB.y + vA.z * vB.z);
	}

	inline double VectorAngle(AVector vA, AVector vB, APlane vP)
	{
		double a1 = VectorAngle(vA, vB);
		vA = vA.Rotate(1.5707963267948966192313216916398, vP.zAxis);
		double a2 = VectorAngle(vA, vB);
		if (a2 > 1.5707963267948966192313216916398) a1 = (6.283185307179586476925286766559) - a1;
		return a1;
	}

	inline AVector VectorCrossProduct(AVector vA, AVector vB)
	{
		return AVector((vA.y * vB.z) - (vA.z * vB.y), (vA.z * vB.x) - (vA.x * vB.z), (vA.x * vB.y) - (vA.y * vB.x));
	}

	inline double VectorDotProduct(AVector vA, AVector vB)
	{
		return (vA.x * vB.x) + (vA.y * vB.y) + (vA.z * vB.z);
	}

	inline APlane operator +(APlane plane, AVector delta)
	{
		APoint orig = plane.origin + delta;
		return APlane(orig, plane.xAxis, plane.yAxis);
	}

	inline ALine PlaneIntersect(APlane p0, APlane p1)
	{
		if (!p0.isValid() || !p1.isValid()) return ALine::invalid;
		if (VectorAngle(p0.zAxis, p1.zAxis) == 0) return ALine::invalid;

		double K1 = (p0.origin.x * p0.zAxis.x) + (p0.origin.y * p0.zAxis.y) + (p0.origin.z * p0.zAxis.z);
		double K2 = (p1.origin.x * p1.zAxis.x) + (p1.origin.y * p1.zAxis.y) + (p1.origin.z * p1.zAxis.z);
		AVector N1 = p0.zAxis;
		AVector N2 = p1.zAxis;

		double xval = 0;
		double yval = 0;
		double zval = 0;

		if (N1.x != 0)
		{
			yval = (N1.x * K2 - N2.x * K1) / (N1.x * N2.y - N1.y * N2.x);
			xval = (K1 - N1.y * yval) / N1.x;
			zval = 0;



			if (!isfinite(xval) || isnan(xval) || !isfinite(yval) || isnan(yval))
			{
				zval = (N1.x * K2 - N2.x * K1) / (N1.x * N2.z - N1.z * N2.x);
				xval = (K1 - N1.z * zval) / N1.x;
				yval = 0;
			}
		}

		else if (N1.y != 0)
		{
			zval = (N1.y * K2 - N2.y * K1) / (N1.y * N2.z - N1.z * N2.y);
			yval = (K1 - N1.z * zval) / N1.y;
			xval = 0;

			if (!isfinite(zval) || isnan(zval) || !isfinite(yval) || isnan(yval))
			{
				xval = (N1.y * K2 - N2.y * K1) / (N1.y * N2.x - N1.x * N2.y);
				yval = (K1 - N1.x * xval) / N1.y;
				zval = 0;
			}
		}

		else if (N1.z != 0)
		{
			yval = (N1.z * K2 - N2.z * K1) / (N1.z * N2.y - N1.y * N2.z);
			zval = (K1 - N1.y * yval) / N1.z;
			xval = 0;

			if (!isfinite(zval) || isnan(zval) || !isfinite(yval) || isnan(yval))
			{
				xval = (N1.z * K2 - N2.z * K1) / (N1.z * N2.x - N1.x * N2.z);
				zval = (K1 - N1.x * xval) / N1.z;
				yval = 0;
			}
		}

		APoint pt = APoint::invalid;
		pt = APoint(xval, yval, zval);
		AVector dir = VectorCrossProduct(p0.zAxis, p1.zAxis);
		return ALine(pt - dir, pt + dir);
	}

	inline ALine CircleIntersectLine(ACircle C, ALine L)
	{
		if (!C.isValid() || !L.isValid()) return ALine::invalid;
		//if line is non-coplanar with the plane, abort
		if (abs(C.plane.DistanceTo(L.st)) > 0.01 || abs(C.plane.DistanceTo(L.en)) > 0.01) return ALine::invalid;

		//get 2d plane coords of line
		AVector dirSt = L.st - C.plane.origin;
		double stX = VectorDotProduct(dirSt, C.plane.xAxis);
		double stY = VectorDotProduct(dirSt, C.plane.yAxis);
		AVector dirEn = L.en - C.plane.origin;
		double enX = VectorDotProduct(dirEn, C.plane.xAxis);
		double enY = VectorDotProduct(dirEn, C.plane.yAxis);
		double r = C.radius;

		//solve m & b in line equation y = mx + b
		double m = (enY - stY) / (enX - stX);
		double b = stY - (m * stX);

		//solving for X with circle, x = (-2mb +- sqrt(-4(m^2+1)(b^2-r^2))/2(m^2+1)
		double temp0 = m * m + 1; //a
		double temp1 = sqrt((4 * m * m * b * b) - 4 * temp0 * (b * b - r * r)); //b square - 4ac
		double x0 = (-2 * m * b + temp1) / (2 * temp0);
		double x1 = (-2 * m * b - temp1) / (2 * temp0);
		if (isnan(x0) || !isfinite(x0) || isnan(x0) || !isfinite(x0)) return ALine::invalid; //no intersection
		double y0 = m * x0 + b;
		double y1 = m * x1 + b;

		//create points
		AVector delta0 = C.plane.xAxis * x0;
		delta0 = delta0 + C.plane.yAxis * y0;
		APoint pt0 = C.plane.origin + delta0;
		AVector delta1 = C.plane.xAxis * x1;
		delta1 = delta1 + C.plane.yAxis * y1;
		APoint pt1 = C.plane.origin + delta1;

		return ALine(pt0, pt1);
	}

	inline ALine CircleIntersectCircle(ACircle cA, ACircle cB)
	{
		if (!cA.isValid() || !cB.isValid()) return ALine::invalid;

		//test planes for coplanarity
		double dist = abs(cA.plane.DistanceTo(cB.plane.origin));
		dist += abs(cA.plane.DistanceTo(cB.plane.origin + cB.plane.xAxis));
		dist += abs(cA.plane.DistanceTo(cB.plane.origin + cB.plane.yAxis));

		//if planes are not coplanar, solve by PlaneXPlane + CircleXLine
		if (dist > 0.01)
		{
			ALine xL0 = PlaneIntersect(cA.plane, cB.plane);
			if (!xL0.isValid()) return ALine::invalid;
			xL0 = CircleIntersectLine(cA, xL0);
			return xL0;
		}

		//make intersection plane
		ALine cL = ALine(cA.plane.origin, cB.plane.origin);
		AVector xvect = cL.Direction();
		AVector yvect = xvect.Rotate(1.5707963267948966192313216916398, cA.plane.zAxis);

		APlane xpln = APlane(cA.plane.origin, xvect, yvect);
		//do the math
		double d = xpln.origin.DistanceTo(cB.plane.origin);
		double rA = cA.radius;
		double rB = cB.radius;
		double x = (rA * rA + d * d - rB * rB) / (d * 2.0);
		double y = sqrt(rA * rA - x * x);
		return ALine(xpln.PointAt(x, y), xpln.PointAt(x, -y));
	}
}

 struct IKsolv
{
public:
	IKsolv();
	static MathTypes::IKres SolveEvaIK(GeometryTypes::APlane targetPlane, GeometryTypes::APlane basePlane, MathTypes::IKres *lastRes);
	static MathTypes::IKres SolveEvaIK(double toX, double toY, double toZ, double tRoX, double tRoY, double tRoZ, MathTypes::IKres *lastRes);
	static MathTypes::IKres SolveEvaIK(double toX, double toY, double toZ, double tRoX, double tRoY, double tRoZ, double boX, double boY, double boZ, double bRoX, double bRoY, double bRoZ, MathTypes::IKres *lastRes);
};

