#include "h.h"

Loader::Loader() : m_file_name(""), m_roots_count(0), m_shapes_count(0), m_log("")
{}

bool Loader::loadFile(string file_name) {
	m_log += "\nStart loading file " + m_file_name + '\n';

	m_file_name = file_name;

	int file_roots_count;
	STEPControl_Reader reader;
	IFSelect_ReturnStatus read_status = reader.ReadFile(m_file_name.c_str());

	reader.PrintCheckLoad(true, IFSelect_ItemsByEntity);
	if (read_status != IFSelect_RetDone) {
		string message = "Error while openning the file " + m_file_name + '\n';
		message += "\n\nLogs:\n";
		message += m_log;
		throw exception(message.c_str());
	}

	m_log += "File " + m_file_name + " is opened\n";

	file_roots_count = reader.NbRootsForTransfer();
	if (file_roots_count != 1) {
		string message = "Error while translating the file\n";
		message += "\n\nLogs:\n";
		message += m_log;
		throw exception(message.c_str());
	}

	m_log += "The file is translated. It contains " + to_string(file_roots_count) + " roots \n";

	m_roots_count = reader.TransferRoots();
	reader.PrintCheckTransfer(true, IFSelect_ItemsByEntity);
	if (m_roots_count != file_roots_count) {
		string message = "Error while getting roots from the file\n";
		message += "\n\nLogs:\n";
		message += m_log;
		throw exception(message.c_str());
	}

	m_log += "All " + to_string(m_roots_count) + " roots are translated\n";

	m_shapes_count = reader.NbShapes();
	if (m_shapes_count != 1) {
		string message = "File contain more then one main shape\n";
		message += "\n\nLogs:\n";
		message += m_log;
		throw exception(message.c_str());
	}

	TopoDS_Shape shape = reader.Shape(1);
	m_log += to_string(m_shapes_count) + " shapes are got from the file. File load complete\n";

	if (shape.ShapeType() != TopAbs_SOLID) {
		string message = "Error! The root shape is not solid.\n";
		message += "\n\nLogs:\n";
		message += m_log;
		throw exception(message.c_str());
	}

	m_log += "Got solid shape\n";

	m_log += "Healing the solid\n";

	Handle(ShapeFix_Shape) sfs = new ShapeFix_Shape;
	sfs->Init(shape);
	sfs->Perform();
	shape = sfs->Shape();

	Handle(ShapeFix_Wireframe) SFWF = new ShapeFix_Wireframe(shape);
	SFWF->ModeDropSmallEdges() = Standard_True;
	SFWF->FixSmallEdges();
	SFWF->FixWireGaps();
	shape = SFWF->Shape();

	m_log += "The solid was healed\n";

	m_solid = TopoDS::Solid(shape);

	m_log += "Loading done!\n";

	return true;
}

int Loader::getWiresCount() const {
	return m_suface_wires.size();
}

bool Loader::parseShape() {
	m_log += "\nStart parsing data\n";

	TopExp_Explorer face_explorer, wire_exprorer, edge_explorer, vertex_explorer;
	int face_counter;

	face_counter = 0;
	for (face_explorer.Init(m_solid, TopAbs_FACE); face_explorer.More(); face_explorer.Next()) {
		m_log += "Parsing face number " + to_string(face_counter) + '\n';

		TopoDS_Face face = TopoDS::Face(face_explorer.Current());
		m_surfaces.push_back(BodySurface(face, face_counter));

		for (wire_exprorer.Init(face, TopAbs_WIRE); wire_exprorer.More(); wire_exprorer.Next()) {
			TopoDS_Wire wire = TopoDS::Wire(wire_exprorer.Current());
			int wire_index = m_suface_wires.size();

			m_suface_wires.push_back(SurfaceWire(wire, wire_index));
			m_surfaces.back().pushWireIndex(wire_index);

			for (edge_explorer.Init(wire, TopAbs_EDGE); edge_explorer.More(); edge_explorer.Next()) {
				TopoDS_Edge edge = TopoDS::Edge(edge_explorer.Current());

				if (edge.IsNull()) {
					string message = "Error. Surface number " + to_string(face_counter) + " has a NULL edge\n";
					message += "\n\nLogs:\n";
					message += m_log;
					throw exception(message.c_str());
				}

				int edge_index = getEdgeIndex(edge);

				if (edge_index == -1) {
					edge_index = m_surface_edges.size();
					m_surface_edges.push_back(SurfaceEdge(edge, edge_index));
				}

				m_surfaces.back().pushEdgeIndex(edge_index);
				m_suface_wires.back().pushEdgeIndex(edge_index);

				for (vertex_explorer.Init(edge, TopAbs_VERTEX); vertex_explorer.More(); vertex_explorer.Next()) {
					TopoDS_Vertex vertex = TopoDS::Vertex(vertex_explorer.Current());
					int vertex_index = getVertexIndex(vertex);

					if (vertex_index == -1) {
						vertex_index = m_points.size();
						m_points.push_back(SurfacePoint(vertex, vertex_index));
					}

					m_surfaces.back().pushPointIndex(vertex_index);
					m_suface_wires.back().pushPointIndex(vertex_index);
					m_surface_edges[edge_index].pushPointIndex(vertex_index);
				}
			}
		}

		++face_counter;
	}

	for (auto iter = m_surface_edges.begin(); iter != m_surface_edges.end(); ++iter) {
		iter->setPointsPointer(&m_points);
		iter->calcPointsParameters();
	}

	for (auto iter = m_suface_wires.begin(); iter != m_suface_wires.end(); ++iter)
		iter->setPointers(&m_surface_edges, &m_points);

	for (auto iter = m_surfaces.begin(); iter != m_surfaces.end(); ++iter) {
		iter->setPointers(&m_suface_wires, &m_surface_edges, &m_points);
		iter->calcUVBoundaries();
		iter->calcUVCoordsOfPoints();
		iter->calcEdgesInUVSpace();
	}

	if (m_surfaces.size() == 0 || m_suface_wires.size() == 0 || m_surface_edges.size() == 0 || m_points.size() == 0) {
		string message = "Error. There is no surfaces/wires/edges/points is file!\n";
		message += "\n\nLogs:\n";
		message += m_log;
		throw exception(message.c_str());
	}

	m_log += "Shapes explored. We got " + to_string(m_surfaces.size()) + " surfaces, " + to_string(m_surface_edges.size()) +
		" edges, " + to_string(m_suface_wires.size()) + " wires and " + to_string(m_points.size()) + " vertexes\n";

	return true;
}

int Loader::getSurfacesCount() const {
	return m_surfaces.size();
}

int Loader::getEdgesCount() const {
	return m_surface_edges.size();
}

int Loader::getPointsCount() const {
	return m_points.size();
}

BodySurface* Loader::getSurface(int index)
{
	if (index >= 0 && index < m_surfaces.size()) {
		return &(m_surfaces[index]);
	}

	return nullptr;
}

SurfaceWire* Loader::getWire(int index)
{
	if (index >= 0 && index < m_suface_wires.size()) {
		return &(m_suface_wires[index]);
	}

	return nullptr;
}

SurfaceEdge* Loader::getEdge(int index)
{
	if (index >= 0 && index < m_surface_edges.size()) {
		return  &(m_surface_edges[index]);
	}

	return nullptr;
}

SurfacePoint* Loader::getPoint(int index)
{
	if (index >= 0 && index < m_points.size())
		return  &(m_points[index]);

	return nullptr;
}

string Loader::getLog() const {
	return m_log;
}

int Loader::getEdgeIndex(const TopoDS_Edge& edge) const
{
	bool new_edge_flag = true;
	int i = 0;
	for (; i < m_surface_edges.size(); ++i)
		if (BRepTools::Compare(*(m_surface_edges[i].getTopoDSEdge()), edge)) {
			new_edge_flag = false;
			break;
		}

	if (new_edge_flag)
		return -1;

	return i;
}

int Loader::getVertexIndex(const TopoDS_Vertex& vertex) const
{
	bool new_vertex_flag = true;
	int i = 0;
	for (; i < m_points.size(); ++i)
		if (BRepTools::Compare(*(m_points[i].getTopoDSVertex()), vertex)) {
			new_vertex_flag = false;
			break;
		}

	if (new_vertex_flag)
		return -1;

	return i;
}
