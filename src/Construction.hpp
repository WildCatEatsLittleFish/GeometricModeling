//
// Created by Yulin Liu on 7/31/19.
//

#ifndef EX2_CONSTRUCTION_HPP
#define EX2_CONSTRUCTION_HPP

#include "Mesh.hpp"


class Construction
{
public:
    // Parameter: grid resolution
    static unsigned int resolution;
    // Parameter: degree of the polynomial
    static unsigned int polyDegree;
    // Parameter: neighbor layers for computing neighbor points
    static unsigned int layer;

    // Bounding Box enlarge degree
    static double bbmin_Degree;
    static double bbmax_Degree;

private:
    // Wendland weight function radius (make this relative to the size of the mesh)
    static double wendlandRadius;

    // Grid bounds: axis-aligned bounding box
    static Eigen::RowVector3d bb_min, bb_max;
    //bounding_box_diagonnal
    static Eigen::Vector3d dia_mM;

public:

    static void computeConstrainedPoints(Mesh& mesh);
    static void computeInterpolatedPoints(Mesh& mesh);
    static bool MarchingCube(Mesh& mesh);  // using marching-cubes

private:
    static void createGrid(Mesh& mesh);
    static std::list<int> computeNeighborCellIndex(int x, int y, int z, int layer, int resolution);
    static void getLines(Mesh& mesh);

};


#endif //EX2_CONSTRUCTION_HPP
