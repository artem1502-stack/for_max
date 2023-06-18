#include "h.h"

void Solver2DPoint::getCoord(double& u, double& v) {
	u = m_u;
	v = m_v;
}

void Solver2DPoint::setAngle(double new_angle) {
	m_angle = new_angle;
}

void Solver2DPoint::setA(double new_a) {
	m_a = new_a;
}

void Solver2DPoint::setB(double new_b) {
	m_b = new_b;
}

double Solver2DPoint::getAngle() const {
	return m_angle;
}

void Solver2DPoint::setType(PointType point_type) {
	m_point_type = point_type;
}

double Solver2DPoint::getA() const {
	return m_a;
}

double Solver2DPoint::getB() const {
	return m_b;
}

PointType Solver2DPoint::getPointType() const {
	return m_point_type;
}

void Solver2DPoint::calcAngle() {
	m_angle = atan2(m_b, m_a) / 4.;
}

void Solver2DPoint::calcAB() {
	if (m_point_type == PointType::EXTERNAL_POINT)
		return;
	m_a = cos(4 * m_angle);
	m_b = sin(4 * m_angle);
}
