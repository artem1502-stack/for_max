#include "h.h"

int main() {
    std::vector<Vertex> vertices;
    std::vector<Triangle> triangles;
    read_obj_file("tor.obj", vertices, triangles);
    std::vector<Quadrilateral> quadrilaterals;
    qmorph(vertices, triangles, quadrilaterals);
    write_msh_file("output.msh", vertices, quadrilaterals);
    std::cout << "Program completed successfully.\n";
    return 0;
}
