#include "h.h"

BodySurface::BodySurface(const TopoDS_Face& face, int index) :m_face(face), m_index(index) {
	m_surface = BRep_Tool::Surface(m_face);
}

const TopoDS_Face* BodySurface::getTopoDSFace() const {
	return &m_face;
}

const Geom_Surface* BodySurface::getGeomSurface() const {
	return m_surface.get();
}

void BodySurface::pushWireIndex(int new_index) {
	if (count(m_wires_indices.begin(), m_wires_indices.end(), new_index) == 0)
		m_wires_indices.push_back(new_index);
}

void BodySurface::pushEdgeIndex(int new_index) {
	if (count(m_edges_indices.begin(), m_edges_indices.end(), new_index) == 0)
		m_edges_indices.push_back(new_index);
}

void BodySurface::pushPointIndex(int new_index) {
	if (count(m_points_indices.begin(), m_points_indices.end(), new_index) == 0)
		m_points_indices.push_back(new_index);
}

int BodySurface::getWireCount() const {
	return m_wires_indices.size();
}

int BodySurface::getEdgeCount() const {
	return m_edges_indices.size();
}

int BodySurface::getPointCount() const {
	return m_points_indices.size();
}

SurfaceWire* BodySurface::getWire(int index) {
	if (index >= 0 && index < m_wires_indices.size())
		return &(m_wires_pointer->at(m_wires_indices[index]));

	return nullptr;
}

SurfaceEdge* BodySurface::getEdge(int index) {
	if (index >= 0 && index < m_edges_indices.size())
		return &(m_edges_pointer->at(m_edges_indices[index]));

	return nullptr;
}

SurfacePoint* BodySurface::getPoint(int index) {
	if (index >= 0 && index < m_points_indices.size())
		return &(m_points_pointer->at(m_points_indices[index]));

	return nullptr;
}

int BodySurface::getIndex() const {
	return m_index;
}

void BodySurface::setPointers(vector<SurfaceWire>* wires, vector<SurfaceEdge>* edges, vector<SurfacePoint>* points) {
	m_wires_pointer = wires;
	m_edges_pointer = edges;
	m_points_pointer = points;
}

void BodySurface::calcUVBoundaries() {
	BRepTools::UVBounds(m_face, m_u_min, m_u_max, m_v_min, m_v_max);
}

void BodySurface::calcUVCoordsOfPoints() {
	for (auto iter = m_points_indices.begin(); iter != m_points_indices.end(); ++iter) {
		gp_Pnt2d new_point_uv_coords = BRep_Tool::Parameters(*(m_points_pointer->at(*iter)).getTopoDSVertex(), m_face);
		m_points_uv_coords.push_back(pair<double, double>(new_point_uv_coords.Coord().X(), new_point_uv_coords.Coord().Y()));
	}
}

void BodySurface::calcEdgesInUVSpace() {
	for (int i = 0; i < m_edges_indices.size(); ++i) {
		SurfaceEdge* edge = &(m_edges_pointer->at(m_edges_indices[i]));
		double start_parameter, end_parameter;
		Handle(Geom2d_Curve) curve = BRep_Tool::CurveOnSurface(*(edge->getTopoDSEdge()), *getTopoDSFace(), start_parameter, end_parameter);
		m_edges_in_uv_space.push_back(curve);
		m_edges_in_uv_space_parameters.push_back(pair<double, double>(start_parameter, end_parameter));
	}
}

void BodySurface::getUVCoordsOfPoint(int index, double& u, double& v) const {
	if (index >= 0 && index < m_points_uv_coords.size()) {
		u = m_points_uv_coords[index].first;
		v = m_points_uv_coords[index].second;
	}

	else {
		u = INT_MIN;
		v = INT_MIN;
	}
}

void BodySurface::getUVBoundaries(double& u_min, double& u_max, double& v_min, double& v_max) const {
	u_min = m_u_min;
	u_max = m_u_max;
	v_min = m_v_min;
	v_max = m_v_max;
}

Geom2d_Curve* BodySurface::getEdgeInUVSpace(int index) {
	if (index >= 0 && index < m_edges_in_uv_space.size())
		return m_edges_in_uv_space[index].get();

	return nullptr;
}

void BodySurface::getEdgeInUVSpaceParameters(int index, double& u, double& v) const {
	if (index >= 0 && index < m_edges_in_uv_space_parameters.size()) {
		u = m_edges_in_uv_space_parameters[index].first;
		v = m_edges_in_uv_space_parameters[index].second;
	}

	else {
		u = INT_MIN;
		v = INT_MIN;
	}
}

int BodySurface::getWiretLocalIndex(const SurfaceWire& wire) const {
	for (int i = 0; i < m_wires_indices.size(); ++i)
		if (m_wires_indices[i] == wire.getIndex())
			return i;

	return -1;
}

int BodySurface::getEdgeLocalIndex(const SurfaceEdge& edge) const {
	for (int i = 0; i < m_edges_indices.size(); ++i)
		if (m_edges_indices[i] == edge.getIndex())
			return i;

	return -1;
}

int BodySurface::getPointLocalIndex(const SurfacePoint& point) const {
	for (int i = 0; i < m_points_indices.size(); ++i)
		if (m_points_indices[i] == point.getIndex())
			return i;

	return -1;
}
