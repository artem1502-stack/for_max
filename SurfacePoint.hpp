#ifndef SURFACEPOINT_HPP
# define SURFACEPOINT_HPP

# include <BRep_Tool.hxx>
# include <gp_Pnt.hxx>
# include <TopoDS_Vertex.hxx>
# include <Bnd_Box2d.hxx>

class SurfacePoint {
public:
	// constructor
	SurfacePoint(const TopoDS_Vertex& vertex, int index);

	// get TopoDS_Vertex of the point
	const TopoDS_Vertex* getTopoDSVertex() const;
	// get gp_Pnt of the point
	const gp_Pnt* getGpPoint() const;

	// get X coord of the point
	double getX() const;
	// get Y coord of the point
	double getY() const;
	// get Z coord of the point
	double getZ() const;
	// get index of the point
	int getIndex() const;

private:
	int m_index;
	TopoDS_Vertex m_vertex;
	gp_Pnt m_point;
};

#endif
