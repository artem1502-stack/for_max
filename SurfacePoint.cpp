#include "h.h"

SurfacePoint::SurfacePoint(const TopoDS_Vertex& vertex, int index) : m_vertex(vertex), m_index(index) {
	m_point = BRep_Tool::Pnt(m_vertex);
}

double SurfacePoint::getX() const {
	return m_point.X();
}

double SurfacePoint::getY() const {
	return m_point.Y();
}

double SurfacePoint::getZ() const
{
	return m_point.Z();
}

int SurfacePoint::getIndex() const {
	return m_index;
}

const TopoDS_Vertex* SurfacePoint::getTopoDSVertex() const {
	return &m_vertex;
}

const gp_Pnt* SurfacePoint::getGpPoint() const {
	return &m_point;
}
