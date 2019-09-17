//
// Created by Yulin Liu on 7/30/19.
//

#ifndef EX2_MESH_HPP
#define EX2_MESH_HPP

#include <iostream>
#include <string>
#include <list>


#include <igl/readOFF.h>
#include <igl/per_face_normals.h>
#include <igl/massmatrix.h>
#include <igl/cotmatrix.h>
#include <igl/adjacency_list.h>
#include <igl/barycenter.h>
#include <igl/vertex_triangle_adjacency.h>
#include <igl/copyleft/marching_cubes.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/rotate_by_quat.h>
#include <igl/slice_into.h>



class Mesh {

public:
    // Imported points, #P x3
    Eigen::MatrixXd P;
    // Normals, #P x3
    Eigen::MatrixXd N;

    //Vertex array, #V x3
    Eigen::MatrixXd V;
    // Face array, #F x3
    Eigen::MatrixXi F;

    // Weighted Normals of the mesh
    Eigen::MatrixXd WN;
    // Face normals of the mesh, #F x3
    Eigen::MatrixXd FN;
    // Vertex normals of the mesh
    Eigen::MatrixXd VN;

    // Intermediate result: constrained points, #C x3
    Eigen::MatrixXd constrained_points;
    // Intermediate result: implicit function values at constrained points, #C x1
    Eigen::VectorXd constrained_values;

    // Intermediate result: grid points, at which the imlicit function will be evaluated, #G x3
    Eigen::MatrixXd grid_points;
    // Intermediate result: implicit function values at the grid points, #G x1
    Eigen::VectorXd grid_values;
    // Intermediate result: grid point colors, for display, #G x3
    Eigen::MatrixXd grid_colors;
    // Intermediate result: grid lines, for display, #L x6 (each row contains
    // starting and ending point of line segment)
    Eigen::MatrixXd grid_lines;

    //Smoothed Vertex array, #V x3
    Eigen::MatrixXd U;
    // Neighboring vertices
    std::vector<std::vector<int> > VV;
    // Compute neighboring faces
    std::vector<std::vector<int> > VF;
    // Laplace-Beltrami operator: #V by #V
    Eigen::SparseMatrix<double> L;
    // Color map based on per_vertex_normal
    Eigen::MatrixXd C;

    // The mass-matrix M and cot-stiffness-matrx Lw of the mesh
    Eigen::SparseMatrix<double> M, Lw;
    // A = Lw * M-1 * Lw
    Eigen::MatrixXd A;
    Eigen::MatrixXd Aff, Afc;



//    // Parameter: grid resolution
//    unsigned int resolution = 20;
//    // Parameter: degree of the polynomial
//    unsigned int polyDegree = 0;
//    // Parameter: neighbor layers for computing neighbor points
//    unsigned int layer = 2;
//
//    // Wendland weight function radius (make this relative to the size of the mesh)
//    double wendlandRadius = 0.1;
//
//    // Bounding Box enlarge degree
//    double bbmin_Degree = 1.2;
//    double bbmax_Degree = 1.2;
//
//    // Grid bounds: axis-aligned bounding box
//    Eigen::RowVector3d bb_min, bb_max;
//    //bounding_box_diagonnal
//    Eigen::Vector3d dia_mM;

    // The eigenvector of the covariance matrix of the input point cloud
    //Eigen::MatrixXd EigenVectors;
    char* input_data;

public:
    Mesh() = default;
    Mesh(const char* input_off);
    void loadPoints();
    void loadMesh();

    void reloadPointsData(const char* input_off);
    void reloadMeshData(const char* input_off);

    void computeData(); // Used after abtaining reconstructing mesh V, F

//    inline void setResolution(unsigned int r) { resolution = r; }
//    //inline void setPolyDegree(unsigned int d) { polyDegree = d; }
//    inline void setNeighborLayers(unsigned int l) { layer = l; }

};


#endif //EX2_MESH_HPP

