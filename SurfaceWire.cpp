#include "h.h"

SurfaceWire::SurfaceWire(TopoDS_Wire& wire, int index) : m_wire(wire), m_index(index) {
}

const TopoDS_Wire* SurfaceWire::getTopoDSWire() const {
	return &m_wire;
}

void SurfaceWire::pushEdgeIndex(int new_index) {
	if (count(m_edges_indices.begin(), m_edges_indices.end(), new_index) == 0)
		m_edges_indices.push_back(new_index);
}

void SurfaceWire::pushPointIndex(int new_index) {
	if (count(m_points_indices.begin(), m_points_indices.end(), new_index) == 0)
		m_points_indices.push_back(new_index);
}

void SurfaceWire::setPointers(vector<SurfaceEdge>* edges_pointer, vector<SurfacePoint>* points_pointer) {
	m_edges_pointer = edges_pointer;
	m_points_pointer = points_pointer;
}

int SurfaceWire::getEdgeCount() const {
	return m_edges_indices.size();
}

int SurfaceWire::getPointCount() const {
	return m_points_indices.size();
}

SurfaceEdge* SurfaceWire::getEdge(int index)
{
	if (index >= 0 && index < m_edges_indices.size())
		return &(m_edges_pointer->at(m_edges_indices[index]));

	return nullptr;
}

int SurfaceWire::getIndex() const {
	return m_index;
}

int SurfaceWire::getEdgeLocalIndex(const SurfaceEdge& edge) const {
	for (int i = 0; i < m_edges_indices.size(); ++i)
		if (m_edges_indices[i] == edge.getIndex())
			return i;

	return -1;
}

int SurfaceWire::getPointLocalIndex(const SurfacePoint& point) const {
	for (int i = 0; i < m_points_indices.size(); ++i)
		if (m_points_indices[i] == point.getIndex())
			return i;

	return -1;
}

SurfacePoint* SurfaceWire::getPoint(int index) {
	if (index >= 0 && index < m_points_indices.size())
		return &(m_points_pointer->at(m_points_indices[index]));

	return nullptr;
}
