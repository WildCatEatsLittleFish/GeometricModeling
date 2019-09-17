//
// Created by Yulin Liu on 8/6/19.
//

#ifndef EX2_DEFORMATION_HPP
#define EX2_DEFORMATION_HPP

#include "Mesh.hpp"
#include "Handle.hpp"

#include <unordered_set>
#include <igl/slice.h>
#include <Eigen/SparseCholesky>

class Deformation {
public:
    static bool multiResolutionSurfaceMethod(Mesh& mesh, Handle& handle);

private:

    static Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>, Eigen::RowMajor > solver;

    // Flag to indicate whether to recompute A_ff
    static bool handles_changed;

    // 4 different deformed mesh
    static Eigen::MatrixXd S;
    static Eigen::MatrixXd B;
    static Eigen::MatrixXd B2;
    static Eigen::MatrixXd S2;



};


#endif //EX2_DEFORMATION_HPP
