#include "h.h"

SurfaceEdge::SurfaceEdge(const TopoDS_Edge edge, int index) :
	m_edge(edge), m_index(index)
{
	double start, end;
	m_curve = BRep_Tool::Curve(m_edge, start, end);
}

const TopoDS_Edge* SurfaceEdge::getTopoDSEdge() const {
	return &m_edge;
}

const Geom_Curve* SurfaceEdge::getGeomCurve() const {
	return m_curve.get();
}

int SurfaceEdge::getPointCount() const {
	return m_points_indices.size();
}

SurfacePoint* SurfaceEdge::getPoint(int index) const
{
	if (index >= 0 && index < m_points_indices.size())
		return &(m_points_pointer->at(m_points_indices[index]));

	return nullptr;
}
void SurfaceEdge::pushPointIndex(int new_index) {
	if (count(m_points_indices.begin(), m_points_indices.end(), new_index) == 0)
		m_points_indices.push_back(new_index);
}

int SurfaceEdge::getIndex() const {
	return m_index;
}

void SurfaceEdge::setPointsPointer(vector<SurfacePoint>* points) {
	m_points_pointer = points;
}

int SurfaceEdge::getPointLocalIndex(const SurfacePoint& point) const {
	for (int i = 0; i < m_points_indices.size(); ++i)
		if (m_points_indices[i] == point.getIndex())
			return i;

	return -1;
}

void SurfaceEdge::calcPointsParameters() {
	for (int i = 0; i < m_points_indices.size(); ++i) {
		const SurfacePoint* point = &(m_points_pointer->at(m_points_indices[i]));
		const TopoDS_Vertex* topo_vertex = point->getTopoDSVertex();
		double edge_parameter = BRep_Tool::Parameter(*topo_vertex, m_edge);
		m_points_parameters[i] = edge_parameter;
	}
}

double SurfaceEdge::getPointParameter(int index) const {
	if (index >= 0 && index < m_points_indices.size())
		return m_points_parameters[index];

	return DBL_MIN;
}
