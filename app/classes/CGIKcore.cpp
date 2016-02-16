  //#include "stdafx.h"
#include "CGIKcore.h"

#define _USE_MATH_DEFINES
#include <math.h>

using namespace MathTypes;
using namespace GeometryTypes;

namespace MathTypes
{
	double Constants::pi = M_PI;
	double Constants::half_pi = M_PI * 0.5;
	double Constants::two_pi = M_PI * 2.0;
	double Constants::deg2rad = M_PI / 180.0;
	double Constants::BOTGEO_EE = 75.00; //34.221; //42.58; //distance between wrist & ee
	double Constants::BOTGEO_Z = 213.278;// 189.165; //151.769; //z-height of axis-2 from work surface
	double Constants::BOTGEO_BICEP = 300;
	double Constants::BOTGEO_ARM = 225;// 252.42;

	AInterval::AInterval(double T0, double T1)
	{
		this->t0 = T0;
		this->t1 = T1;
	}

	double AInterval::Min()
	{
		return fmin(this->t0, this->t1);
	}

	double AInterval::Max()
	{
		return fmax(this->t0, this->t1);
	}

	bool AInterval::includesParameter(double val)
	{
		if (val >= t0 && val <= t1) return true;
		return false;
	}

	AInterval AInterval::A1_RNG = AInterval(-180 * MathTypes::Constants::deg2rad, 180 * MathTypes::Constants::deg2rad);
	AInterval AInterval::A2_RNG = AInterval(-90 * MathTypes::Constants::deg2rad, 90 * MathTypes::Constants::deg2rad);
	AInterval AInterval::A3_RNG = AInterval(-65 * MathTypes::Constants::deg2rad, 90 * MathTypes::Constants::deg2rad);
	AInterval AInterval::A4_RNG = AInterval(-180 * MathTypes::Constants::deg2rad, 180 * MathTypes::Constants::deg2rad);
	AInterval AInterval::A5_RNG = AInterval(-120 * MathTypes::Constants::deg2rad, 120 * MathTypes::Constants::deg2rad);
	AInterval AInterval::A6_RNG = AInterval(-150 * MathTypes::Constants::deg2rad, 150 * MathTypes::Constants::deg2rad);

	IKres::IKres()
	{
		this->a0 = 0;
		this->a1 = 0;
		this->a2 = 0;
		this->a3 = 0;
		this->a4 = 0;
		this->a5 = 0;
		this->reachScore = 0;
		this->errorCode = 0;
		this->warningCode = 0;
	}

	IKres::IKres(double A0, double A1, double A2, double A3, double A4, double A5, int ReachScore, int ErrorCode, int WarningCode)
	{
		this->a0 = A0;
		this->a1 = A1;
		this->a2 = A2;
		this->a3 = A3;
		this->a4 = A4;
		this->a5 = A5;
		this->reachScore = ReachScore;
		this->errorCode = ErrorCode;
		this->warningCode = WarningCode;
	}
}

namespace GeometryTypes
{
	//------------------------------APOINT-------------------------------------

	APoint::APoint()
	{
		this->x = 0;
		this->y = 0;
		this->z = 0;
	}

	APoint::APoint(double X, double Y, double Z)
	{
		this->x = X;
		this->y = Y;
		this->z = Z;
	}

	bool APoint::isValid()
	{
		if (isnan(x) || isnan(y) || isnan(z)) return false;
		return true;
	}

	double APoint::DistanceTo(APoint pt)
	{
		if (!pt.isValid()) return 0;
		double dx = pt.x - x;
		double dy = pt.y - y;
		double dz = pt.z - z;
		return sqrt(dx * dx + dy * dy + dz * dz);
	}

	APoint APoint::origin = APoint(0, 0, 0);
	APoint APoint::invalid = APoint(NAN, NAN, NAN);

	//------------------------------AVECTOR-------------------------------------

	AVector::AVector()
	{
		x = 0;
		y = 0;
		z = 0;
	}

	AVector::AVector(double X, double Y, double Z)
	{
		x = X;
		y = Y;
		z = Z;
	}

	AVector::AVector(APoint from, APoint to)
	{
		x = to.x - from.x;
		y = to.y - from.y;
		z = to.z - from.z;
	}

	double AVector::Length()
	{
		return sqrt(x * x + y * y + z * z);
	}

	AVector AVector::Normalize()
	{
		double length = Length();
		return AVector(x / length, y / length, z / length);
	}

	AVector AVector::Rotate(double angle, AVector axis)
	{
		if (axis.Length() != 1.0) axis = axis.Normalize();

		double cos0 = cos(angle);
		double cos1 = 1 - cos0;
		double sin0 = sin(angle);
		double xval = ((cos0 + (axis.x * axis.x) * cos1) * x) + (((axis.x * axis.y * cos1) - (axis.z * sin0)) * y) + (((axis.x * axis.z * cos1) + (axis.y * sin0)) * z);
		double yval = (((axis.x * axis.y * cos1) + (axis.z * sin0)) * x) + ((cos0 + (axis.y * axis.y) * cos1) * y) + (((axis.y * axis.z * cos1) - (axis.x * sin0)) * z);
		double zval = (((axis.x * axis.z * cos1) - (axis.y * sin0)) * x) + (((axis.y * axis.z * cos1) + (axis.x * sin0)) * y) + ((cos0 + (axis.z * axis.z) * cos1) * z);
		return AVector(xval, yval, zval);

	}

	bool AVector::isValid()
	{
		if (isnan(x) || isnan(y) || isnan(z)) return false;
		return true;
	}

	AVector AVector::XAxis = AVector(1, 0, 0);
	AVector AVector::YAxis = AVector(0, 1, 0);
	AVector AVector::ZAxis = AVector(0, 0, 1);
	AVector AVector::invalid = AVector(NAN, NAN, NAN);

	//------------------------------APLANE-------------------------------------

	APlane::APlane()
	{
		origin = APoint::origin;
		xAxis = AVector::XAxis;
		yAxis = AVector::YAxis;
		zAxis = AVector::ZAxis;
	}

	APlane::APlane(APoint origin, AVector xAxis, AVector yAxis)
	{
		this->origin = origin;
		this->xAxis = xAxis.Normalize();
		this->yAxis = yAxis.Normalize();
		this->zAxis = VectorCrossProduct(xAxis, yAxis);
	}

	APlane::APlane(double originX, double originY, double originZ, double rotX, double rotY, double rotZ)
	{
		APlane pl = APlane(APoint(originX, originY, originZ), AVector::XAxis, AVector::YAxis);
		pl = pl.Rotate(rotZ, pl.zAxis);
		pl = pl.Rotate(rotY, pl.yAxis);
		pl = pl.Rotate(rotX, pl.xAxis);
		this->origin = pl.origin;
		this->xAxis = pl.xAxis;
		this->yAxis = pl.yAxis;
		this->zAxis = pl.zAxis;
	}

	bool APlane::isValid()
	{
		if (!origin.isValid() || !xAxis.isValid() || !yAxis.isValid() || !zAxis.isValid()) return false;
		return true;
	}

	APlane APlane::Rotate(double angle, AVector axis)
	{
		AVector xA = this->xAxis.Rotate(angle, axis);
		AVector yA = this->yAxis.Rotate(angle, axis);
		return APlane(this->origin, xA, yA);
	}

	double APlane::DistanceTo(APoint pt)
	{
		if (!pt.isValid()) return NAN;
		AVector vect = pt - this->origin;
		return VectorDotProduct(vect, this->zAxis);
	}

	APoint APlane::ClosestPoint(APoint pt)
	{
		if (!pt.isValid()) return APoint::invalid;
		AVector vect = pt - this->origin;
		double dist = VectorDotProduct(vect, this->zAxis);
		AVector delta = this->zAxis * dist;
		pt = pt - delta;
		return pt;
	}

	APoint APlane::PointAt(double u, double v)
	{
		AVector delta = xAxis * u;
		delta = delta + yAxis * v;
		return origin + delta;
	}

	APoint APlane::PointAt(double u, double v, double w)
	{
		AVector delta = xAxis * u;
		delta = delta + yAxis * v;
		delta = delta + zAxis * w;
		return origin + delta;
	}

	APlane APlane::worldXY = APlane(APoint::origin, AVector::XAxis, AVector::YAxis);
	APlane APlane::worldYZ = APlane(APoint::origin, AVector::YAxis, AVector::ZAxis);
	APlane APlane::worldXZ = APlane(APoint::origin, AVector::XAxis, AVector::ZAxis);
	APlane APlane::invalid = APlane(APoint::invalid, AVector::invalid, AVector::invalid);

	//------------------------------ALINE-------------------------------------

	ALine::ALine()
	{
		st = APoint::origin;
		en = APoint::origin;
	}

	ALine::ALine(APoint StartPt, APoint EndPt)
	{
		st = StartPt;
		en = EndPt;
	}

	bool ALine::isValid()
	{
		if (!st.isValid() || !en.isValid()) return false;
		return true;
	}

	AVector ALine::Direction()
	{
		return AVector(st, en);
	}

	ALine ALine::invalid = ALine(APoint::invalid, APoint::invalid);

	//------------------------------ACIRCLE-------------------------------------

	ACircle::ACircle()
	{
		plane = APlane::worldXY;
		radius = 0;
	}

	ACircle::ACircle(double Radius)
	{
		plane = APlane::worldXY;
		radius = Radius;
	}

	ACircle::ACircle(APlane Plane, double Radius)
	{
		plane = Plane;
		radius = Radius;
	}

	ACircle::ACircle(APlane Plane, APoint Centre, double Radius)
	{
		plane = APlane(Centre, Plane.xAxis, Plane.yAxis);
		radius = Radius;
	}

	bool ACircle::isValid()
	{
		if (!plane.isValid() || isnan(radius) || (radius <= 0)) return false;
		return true;
	}

	ACircle ACircle::invalid = ACircle(APlane::invalid, NAN);
}

//_____________________________________________________________________________________________________
//_____________________________________SOLVERS_________________________________________________________


IKsolv::IKsolv(){}

MathTypes::IKres IKsolv::SolveEvaIK(GeometryTypes::APlane targetPlane, GeometryTypes::APlane basePlane, MathTypes::IKres *lastRes)
{
	MathTypes::IKres bangs = MathTypes::IKres();
	bangs.reachScore = 0;
	bangs.errorCode = 0;
	bangs.warningCode = 0;


	//locate wrist
	APoint wrist = targetPlane.origin + (targetPlane.zAxis * Constants::BOTGEO_EE);
	APlane wristPln = APlane(wrist, targetPlane.xAxis, targetPlane.yAxis);
	

	//----------------------------------------------
	//axis 1

	APoint wrist_2d = basePlane.ClosestPoint(wrist);
	double ang1 = VectorAngle(basePlane.yAxis, wrist_2d - basePlane.origin, basePlane);
	ang1 = (ang1 > Constants::pi ? ang1 - Constants::two_pi : ang1);
	if (ang1 < AInterval::A1_RNG.Min())
	{
		ang1 += Constants::pi;
		bangs.warningCode = 100000;
	}
	else if (ang1 > AInterval::A1_RNG.Max())
	{
		ang1 -= Constants::pi;
		bangs.warningCode = 100000;
	}
	bangs.a0 = ang1;
	/*bangs.a0 = (ang1 < AInterval::A1_RNG.Min() ? AInterval::A1_RNG.Min() : (ang1 > AInterval::A1_RNG.Max() ? AInterval::A1_RNG.Max() : ang1));
	if (bangs.a0 != ang1)
	{
		bangs.reachScore = 1;
		bangs.warningCode = 100000;
	}*/

	//----------------------------------------------
	//axis 2

	APlane axis2pln = APlane(basePlane.origin, basePlane.yAxis, basePlane.zAxis);
	axis2pln = axis2pln + basePlane.zAxis * Constants::BOTGEO_Z;
	axis2pln = axis2pln.Rotate(bangs.a0, basePlane.zAxis);

	ACircle circ_shoulder = ACircle(axis2pln, Constants::BOTGEO_BICEP);
	ACircle circ_wrist = ACircle(axis2pln, wrist, Constants::BOTGEO_ARM);
	ALine xL2 = CircleIntersectCircle(circ_shoulder, circ_wrist);

	if (!xL2.isValid())
	{
		bangs.reachScore = 2;
		bangs.errorCode = 010000;
		return bangs;
	}
	double ang2a = VectorAngle(axis2pln.yAxis, xL2.st - axis2pln.origin, axis2pln);
	double ang2b = VectorAngle(axis2pln.yAxis, xL2.en - axis2pln.origin, axis2pln);
	ang2a = (ang2a > Constants::pi ? ang2a - Constants::two_pi : ang2a);
	ang2b = (ang2b > Constants::pi ? ang2b - Constants::two_pi : ang2b);
	double ang2 = (ang2a > ang2b ? ang2a : ang2b);
	bangs.a1 = (ang2 < AInterval::A2_RNG.Min() ? AInterval::A2_RNG.Min() : (ang2 > AInterval::A2_RNG.Max() ? AInterval::A2_RNG.Max() : ang2));
	if (bangs.a1 != ang2)
	{
		bangs.reachScore = 1;
		bangs.warningCode = 010000;
	}
	APoint elbow = (ang2a > ang2b ? xL2.st : xL2.en);

	//APoint elbow = APoint();
	//if (AInterval::A2_RNG.includesParameter(ang2a) && !AInterval::A2_RNG.includesParameter(ang2b))
	//{
	//	elbow = xL2.st;
	//	bangs.a1 = ang2a;
	//}
	//else if (AInterval::A2_RNG.includesParameter(ang2b) && !AInterval::A2_RNG.includesParameter(ang2a))
	//{
	//	elbow = xL2.en;
	//	bangs.a1 = ang2b;
	//}
	//else if (AInterval::A2_RNG.includesParameter(ang2a) && AInterval::A2_RNG.includesParameter(ang2b))
	//{
	//	//Pick position that can be reached faster
	//	double diff0 = abs(lastRes->a1 - ang2a);
	//	double diff1 = abs(lastRes->a1 - ang2b);
	//	bangs.a1 = (diff0 < diff1 ? ang2a : ang2b);
	//	elbow = (diff0 < diff1 ? xL2.st : xL2.en);
	//}
	//else
	//{
	//	//both positions out of range
	//	bangs.reachScore = 2;
	//	bangs.errorCode = 010000;
	//	return bangs;
	//}

	//----------------------------------------------
	//axis 3

	APlane axis3pln = axis2pln;
	axis3pln.origin = elbow;
	axis3pln = axis3pln.Rotate(bangs.a1, axis3pln.zAxis);
	double ang3 = VectorAngle(axis3pln.xAxis, wrist - elbow, axis3pln);
	ang3 = (ang3 > Constants::pi ? ang3 - Constants::two_pi : ang3);
	bangs.a2 = (ang3 < AInterval::A3_RNG.Min() ? AInterval::A3_RNG.Min() : (ang3 > AInterval::A3_RNG.Max() ? AInterval::A3_RNG.Max() : ang3));
	if (bangs.a2 != ang3)
	{
		bangs.reachScore = 1;
		bangs.warningCode += 1000;
	}

	//----------------------------------------------
	//axis 4

	APlane axis4pln = axis3pln.Rotate(bangs.a2, axis3pln.zAxis);
	axis4pln.origin = wrist;
	axis4pln = axis4pln.Rotate(-Constants::half_pi, axis4pln.yAxis);
	ALine xL4 = PlaneIntersect(wristPln, axis4pln);

	if (!xL4.isValid())
	{
		bangs.reachScore = 2;
		bangs.errorCode += 100;
		return bangs;
	}
	double ang4a = VectorAngle(axis4pln.xAxis, xL4.Direction(), axis4pln);
	ang4a = (ang4a > Constants::pi ? ang4a - Constants::two_pi : ang4a);
	double ang4b = VectorAngle(axis4pln.xAxis, -xL4.Direction(), axis4pln);
	ang4b = (ang4b > Constants::pi ? ang4b - Constants::two_pi : ang4b);

	bangs.a3 = 0;
	//if one is out of range, Pick the other
	if (AInterval::A4_RNG.includesParameter(ang4a) && !AInterval::A4_RNG.includesParameter(ang4b))
		bangs.a3 = ang4a;
	else if (AInterval::A4_RNG.includesParameter(ang4b) && !AInterval::A4_RNG.includesParameter(ang4a))
		bangs.a3 = ang4b;
	else if (AInterval::A4_RNG.includesParameter(ang4a) && AInterval::A4_RNG.includesParameter(ang4b))
	{
		//else Pick the position that can be reached faster
		double diff0 = abs(lastRes->a3 - ang4a);
		double diff1 = abs(lastRes->a3 - ang4b);
		bangs.a3 = (diff0 < diff1 ? ang4a : ang4b);
	}
	else
	{
		//if both positions out of range, abort
		bangs.reachScore = 2;
		bangs.errorCode += 100;
		return bangs;
	}

	//----------------------------------------------
	//axis 5

	APlane axis5pln = axis4pln.Rotate(bangs.a3, axis4pln.zAxis);
	axis5pln = axis5pln.Rotate(Constants::half_pi, axis5pln.yAxis);
	double ang5 = VectorAngle(axis5pln.xAxis, targetPlane.origin - wrist, axis5pln);
	ang5 = (ang5 > Constants::pi ? ang5 - Constants::two_pi : ang5);
	bangs.a4 = (ang5 < AInterval::A5_RNG.Min() ? AInterval::A5_RNG.Min() : (ang5 > AInterval::A5_RNG.Max() ? AInterval::A5_RNG.Max() : ang5));
	if (bangs.a4 != ang5)
	{
		bangs.reachScore = 1;
		bangs.warningCode += 10;
	}

	//----------------------------------------------
	//axis 6

	APlane axis6pln = axis5pln.Rotate(bangs.a4, axis5pln.zAxis);
	axis6pln = axis6pln.Rotate(Constants::half_pi, -axis6pln.yAxis);
	double ang6 = VectorAngle(axis6pln.xAxis, targetPlane.xAxis, axis6pln);
	ang6 = (ang6 > Constants::pi ? ang6 - Constants::two_pi : ang6);
	bangs.a5 = (ang6 < AInterval::A6_RNG.Min() ? AInterval::A6_RNG.Min() : (ang6 > AInterval::A6_RNG.Max() ? AInterval::A6_RNG.Max() : ang6));
	if (bangs.a5 != ang6)
	{
		bangs.reachScore = 1;
		bangs.warningCode += 1;
	}
	return bangs;
}

MathTypes::IKres IKsolv::SolveEvaIK(double toX, double toY, double toZ, double tRoX, double tRoY, double tRoZ, MathTypes::IKres *lastRes)
{
	APlane targetPlane = APlane(toX, toY, toZ, tRoX, tRoY, tRoZ);
	return SolveEvaIK(targetPlane, APlane::worldXY, lastRes);
}

MathTypes::IKres IKsolv::SolveEvaIK(double toX, double toY, double toZ, double tRoX, double tRoY, double tRoZ, double boX, double boY, double boZ, double bRoX, double bRoY, double bRoZ, MathTypes::IKres *lastRes)
{
	APlane basePlane = APlane(boX, boY, boZ, bRoX, bRoY, bRoZ);
	APlane targetPlane = APlane(toX, toY, toZ, tRoX, tRoY, tRoZ);
	return SolveEvaIK(targetPlane, basePlane, lastRes);
}


