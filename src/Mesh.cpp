//
// Created by Yulin Liu on 7/30/19.
//

#include "Mesh.hpp"


//Mesh::Mesh(std::string& input_off)
Mesh::Mesh(const char* input_off)
{
    input_data = (char*)input_off;

//    grid_points.resize(0, 3);
//    grid_colors.resize(0, 3);
//    grid_lines. resize(0, 6);
//    grid_values.resize(0);
//    V. resize(0, 3);
//    F. resize(0, 3);
//    FN.resize(0, 3);


};

void Mesh::loadPoints()
{
    igl::readOFF(input_data,P,F,N);
}

void Mesh::loadMesh()
{
    igl::readOFF(input_data, V, F);
    computeData();
}

void Mesh::computeData()
{
    // Initialize smoothing vertex
    U = V;


    /******* Pre-computation for smoothing *********/

    // Compute Laplace-Beltrami operator: #V by #V
    igl::cotmatrix(V,F,L);

    // Compute neighboring vertices
    igl::adjacency_list(F, VV);

    // Compute neighboring faces
    std::vector<std::vector<int> > VFi;
    igl::vertex_triangle_adjacency(U, F, VF, VFi);

    // Compute Face Normals of the mesh
    igl::per_face_normals(V,F,FN);



    /******* Pre-computation for shape deformation *********/

    // Compute the mass-matrix M and cot-stiffness-matrx Lw
    igl::cotmatrix(V, F, Lw);
    igl::massmatrix(V, F, igl::MASSMATRIX_TYPE_BARYCENTRIC, M);

    Eigen::MatrixXd M_inverse;
    M_inverse.resize(V.rows(), V.rows());
    M_inverse.setZero();
    for(unsigned v = 0; v < V.rows(); ++v)
    {
        M_inverse(v, v) = 1 / M.coeff(v, v) ;
    }
    A = Lw * M_inverse * Lw;



}

void Mesh::reloadPointsData(const char* input_off)
{
    input_data = (char*)input_off;

    igl::readOFF(input_off, P, F, N);
}

void Mesh::reloadMeshData(const char* input_off)
{
    input_data = (char*)input_off;

    igl::readOFF(input_off, V, F);
    computeData();
}
