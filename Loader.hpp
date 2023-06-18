#ifndef LOADER_HPP
# define LOADER_HPP

class Loader
{
public:
	// constructor
	Loader();
	// load file and check if the data is correct
	bool loadFile(string file_name);
	// parse data that is in the file 
	// and check of the data is correct
	bool parseShape();

	// get number of wires loaded from the file
	int getWiresCount() const;
	// get number of faces loaded from the file
	int getSurfacesCount() const;
	// get number of edges loaded from the file
	int getEdgesCount() const;
	// get points of wires loaded from the file
	int getPointsCount() const;

	// get index of the edge in local numeration
	int getEdgeIndex(const TopoDS_Edge& edge) const;
	// get indexof the vertex in local numeration
	int getVertexIndex(const TopoDS_Vertex& vertex) const;

	// get surface with given local index
	BodySurface* getSurface(int index);
	// get wire with given local index
	SurfaceWire* getWire(int index);
	// get edge with given local index
	SurfaceEdge* getEdge(int index);
	// get point with given local index
	SurfacePoint* getPoint(int index);

	// get log of the module
	string getLog() const;

private:
	int m_roots_count, m_shapes_count;
	string m_file_name;
	TopoDS_Solid m_solid;
	vector<BodySurface> m_surfaces;
	vector<SurfaceWire> m_suface_wires;
	vector<SurfaceEdge> m_surface_edges;
	vector<SurfacePoint> m_points;
	string m_log;
};

#endif
