#ifndef H_H
# define H_H

# include <iostream>
# include <stdlib.b>
# include <vector>
# include <map>
# include <algorithm>

# include <BRepTools.hxx>
# include <BRep_Tool.hxx>
# include <TopoDS_Face.hxx>
# include <TopoDS_Wire.hxx>
# include <TopoDS_Edge.hxx>
# include <BRepBuilderAPI_MakeEdge.hxx>

# include "SurfacePoint.hpp"
# include "Loader.hpp"
# include "SurfaceEdge.hpp"
# include "SurfaceMeshGenerator.hpp"
# include "DelaunayTriangulation.hpp"
# include "SurfaceWire.hpp"
# include "Solver2DPoint.hpp"
# include "BodySurface.hpp"
# include "Solver2D.hpp"
# include "Rtree.hpp"

struct Vertex {
	double x; 
	double y; 
	double z; 
};

struct Triangle {
	int v1; 
	int v2; 
	int v3; 
};

struct Quadrilateral {
	int v1; 
	int v2; 
	int v3; 
	int v4; 
};

void read_obj_file(const std::string& filename, std::vector<Vertex>& vertices,
				std::vector<Triangle>& triangles);
void write_msh_file(const std::string& filename, const std::vector<Vertex>& vertices,
				const std::vector<Quadrilateral>& quadrilaterals);
void qmorph(const std::vector<Vertex>& vertices, const std::vector<Triangle>& triangles,
				std::vector<Quadrilateral>& quadrilaterals);

#endif
