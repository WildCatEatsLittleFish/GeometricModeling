//
// Created by Yulin Liu on 7/31/19.
//

#ifndef EX2_SMOOTHING_HPP
#define EX2_SMOOTHING_HPP

#include "Mesh.hpp"


namespace LaplacianSmoothing
{
    void computeSmoothedVertex(Mesh& mesh);

}

class BilateralDenoising{
private:
    // The parameters of the algorithm
    static double sigma_c, sigma_s, rho;
//    int iterations = 5;
//
//    // Denoising center vertex
//    int chosen_vertex = 0;

    //Record last Smoothed Vertex array, #V x3
    static Eigen::MatrixXd last_U;

public:
    static int chosen_vertex;

public:
    //static void iterationDenoising(Mesh& mesh, int chosen_vertex, int iterations = 5);
//    static void computeDenoisedVertex(Mesh& mesh, int chosen_vertex);
    static void iterationDenoising(Mesh& mesh, int iterations = 5);
    static void computeDenoisedVertex(Mesh& mesh);

    static void resetToLast(Mesh& mesh);
    static void resetToOriginal(Mesh& mesh);

private:
//    static bool computeParas(Mesh& mesh, int chosen_vertex);
    static bool computeParas(Mesh& mesh);
    static bool denoisePoint(Mesh& mesh);

};


#endif //EX2_SMOOTHING_HPP
