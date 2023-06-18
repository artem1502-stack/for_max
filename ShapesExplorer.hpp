#ifndef SHAPESEXPLORER_HPP
# define SHAPESEXPLORER_HPP

# include <TopoDS.hxx>
# include <TopoDS_Shape.hxx>
# include <TopoDS_Solid.hxx>
# include <TopoDS_Face.hxx>
# include <TopoDS_Vertex.hxx>
# include <TopExp.hxx>
# include <TopExp_Explorer.hxx>
# include <TopTools.hxx>
# include <TopTools_IndexedMapOfShape.hxx>
# include <BRep_Tool.hxx>
# include <Geom_Surface.hxx>
# include <Geom_Point.hxx>
# include <Geom_CartesianPoint.hxx>
# include <Standard_Handle.hxx>
# include <NCollection_Vector.hxx>

class ShapeExplorer
{
public:
	ShapeExplorer();
	bool exploreShape(const TopoDS_Shape m_shape);
	int getSurfaceCount() const;
	int getPointsCount() const;
	const Handle(Geom_Surface) getSurface(int index) const;
	const Handle(gp_Pnt) getPoint(int index) const;

private:
	TopoDS_Solid m_solid;
	NCollection_Vector<Handle(Geom_Surface)> m_surfaces;
	NCollection_Vector<Handle(gp_Pnt)> m_points;
};

#endif
