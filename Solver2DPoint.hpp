#ifndef SOLVER2DPOINT_HPP
# define SOLVER2DPOINT_HPP

# include <limits>
# include <cmath>


class Solver2DPoint {
public:
	// constructors
	Solver2DPoint();
	Solver2DPoint(double u, double v, double angle, PointType point_type);

	// get coords of the point
	void getCoord(double& u, double& v);

	// set angle between s1 vector of local basis and
	// vector d1 of the direction field at the point
	void setAngle(double new_value);
	// set type at the point
	void setType(PointType point_type);
	// set A at the point
	void setA(double new_a);
	// set B at the point
	void setB(double new_b);

	// get angle between s1 vector of local basis and
	// vector d1 of the direction field at this point
	double getAngle() const;
	// get A at the point
	double getA() const;
	// get B at the point
	double getB() const;
	// get type of the point
	PointType getPointType() const;

	// calculate A = cos(4*angle) and B = sin(4*angle) 
	// at the point
	void calcAB();
	// calculate angle = 0.25*atan2(B, A) at the point
	void calcAngle();

private:
	double m_u;
	double m_v;
	double m_angle;
	double m_a;
	double m_b;
	PointType m_point_type;
};

#endif
