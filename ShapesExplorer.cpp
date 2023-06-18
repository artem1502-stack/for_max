#include "h.h"

ShapeExplorer::ShapeExplorer()
{}

bool ShapeExplorer::exploreShape(TopoDS_Shape shape) {
	if (shape.ShapeType() != TopAbs_SOLID)
		return false;

	m_solid = TopoDS::Solid(shape);

	// get surfaces and convert them to geom_surface
	TopExp_Explorer face_explorer;
	for (face_explorer.Init(m_solid, TopAbs_FACE); face_explorer.More(); face_explorer.Next()) {
		TopoDS_Face face = TopoDS::Face(face_explorer.Current());
		Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
		m_surfaces.Append(surface);
	}

	// get verticles and convert them to gp_Pnt
	TopTools_IndexedMapOfShape topods_verticles;
	TopExp::MapShapes(m_solid, TopAbs_VERTEX, topods_verticles);
	for (auto iter = topods_verticles.cbegin(); iter != topods_verticles.cend(); ++iter) {
		TopoDS_Vertex vertex = TopoDS::Vertex(*iter);
		gp_Pnt point = BRep_Tool::Pnt(vertex);
		Handle(gp_Pnt) point_handler = &point;
		m_points.Append(point_handler);
	}

	if (m_surfaces.Size() == 0 || m_points.Size() == 0)
		return false;
}

int ShapeExplorer::getSurfaceCount() const {
	return m_surfaces.Size();
}

int ShapeExplorer::getPointsCount() const {
	return m_points.Size();
}

const Handle(Geom_Surface) ShapeExplorer::getSurface(int index) const
{
	if (index >= 0 && index < m_surfaces.Size())
		return m_surfaces[index];

	Handle(Geom_Surface) empty;
	return empty;
}

const Handle(gp_Pnt) ShapeExplorer::getPoint(int index) const
{
	if (index >= 0 && index < m_points.Size())
		return m_points[index];

	Handle(gp_Pnt) empty;
	return empty;
}
