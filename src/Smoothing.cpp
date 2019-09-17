//
// Created by Yulin Liu on 7/31/19.
//

#include "Smoothing.hpp"


namespace LaplacianSmoothing
{
    void computeSmoothedVertex(Mesh& mesh)
    {
        // Recompute just mass matrix on each step
        Eigen::SparseMatrix<double> M;
        igl::massmatrix(mesh.U, mesh.F,igl::MASSMATRIX_TYPE_BARYCENTRIC,M);
        // Solve (M-delta*L) U = M*U
        const auto & S = (M - 0.001 * mesh.L);
        Eigen::SimplicialLLT<Eigen::SparseMatrix<double > > solver(S);
        assert(solver.info() == Eigen::Success);
        mesh.U = solver.solve(M * mesh.U).eval();
        // Compute centroid and subtract (also important for numerics)
        Eigen::VectorXd dblA;
        igl::doublearea(mesh.U, mesh.F,dblA);
        double area = 0.5*dblA.sum();
        Eigen::MatrixXd BC;
        igl::barycenter(mesh.U, mesh.F,BC);
        Eigen::RowVector3d centroid(0,0,0);
        for(int i = 0;i<BC.rows();i++)
        {
            centroid += 0.5*dblA(i)/area*BC.row(i);
        }
        mesh.U.rowwise() -= centroid;
        // Normalize to unit surface area (important for numerics)
        mesh.U.array() /= sqrt(area);

    }

}

double BilateralDenoising::sigma_c;
double BilateralDenoising::sigma_s;
double BilateralDenoising::rho;

int BilateralDenoising::chosen_vertex;
Eigen::MatrixXd BilateralDenoising::last_U;

void BilateralDenoising::iterationDenoising(Mesh& mesh, int iterations)
{
    last_U = mesh.U;
    for(int i = 0 ; i < iterations; ++i)
    {
        computeDenoisedVertex(mesh);
    }
}

void BilateralDenoising::computeDenoisedVertex(Mesh& mesh)
{
    last_U = mesh.U;
    computeParas(mesh);
    denoisePoint(mesh);
}

void BilateralDenoising::resetToLast(Mesh& mesh)
{
    mesh.U = last_U;
}

void BilateralDenoising::resetToOriginal(Mesh& mesh)
{
    mesh.U = mesh.V;
}


bool BilateralDenoising::computeParas(Mesh& mesh)
{

    /* Based on last denoised points and areas to recompute triangle area */
    // Compute each triangle area
    Eigen::MatrixXd A_area;
    igl::doublearea(mesh.U, mesh.F, A_area);

    // Compute weighed normals
    mesh.WN.resize(mesh.U.rows(), 3);
    for(unsigned v = 0 ; v < mesh.U.rows(); ++v)
    {
        std::vector<int> neighborFaces = mesh.VF[v];
        double A_area_all = 0;
        Eigen::RowVector3d Nv;
        Nv << 0, 0, 0;
        for(auto &each_face : neighborFaces)
        {
            A_area_all += A_area(each_face);
            Nv = Nv +  mesh.FN.row(each_face) * A_area(each_face);
        }
        Nv = Nv / A_area_all;

        mesh.WN.row(v) = Nv.normalized();
    }

    std::vector<int> neighbors = mesh.VV[chosen_vertex];
    std::vector<double> offsets;
    double all_offsets = 0;

    // Compute the radius of the neighborhood
    double radius = 0;
    for(auto &each : neighbors)
    {
        double dis = (mesh.U.row(each) - mesh.U.row(chosen_vertex)).norm();
        if(dis > radius)
        {
            radius = dis;
        }

        // offset
        Eigen::Vector3d vq;
        vq = mesh.U.row(chosen_vertex) - mesh.U.row(each);
        double offset = mesh.WN.row(chosen_vertex).dot(vq);
        offsets.push_back(offset);
        all_offsets += offset;
    }
    sigma_c = radius;
    rho = sigma_c * 2;

    // Compute standard deviation of the offsets in the selected neighborhood
    double mean_offset = all_offsets / neighbors.size();
    double variance = 0;
    for(auto &each_offset : offsets)
    {
        variance += pow((each_offset - mean_offset), 2) / (neighbors.size() - 1);
    }
    sigma_s = pow(variance, 0.5);


    return true;
}

bool BilateralDenoising::denoisePoint(Mesh& mesh)
{
    for(unsigned v = 0; v < last_U.rows(); ++v)
    {
        std::vector<int> neighbors = mesh.VV[v];
        double sum = 0;
        double normalizer = 0;
        for(auto &each : neighbors)
        {
            // Decide if each of the neighbors is in the rho radius of the vertex
            double dis = (last_U.row(v) - last_U.row(each)).norm();
            if(dis <= rho)
            {
                double t = (last_U.row(v) - last_U.row(each)).norm();
                double h = mesh.WN.row(v).dot(last_U.row(v) - last_U.row(each));
                double wc = exp(- pow(t, 2) / (2 * pow(sigma_c, 2)));
                double ws = exp(- pow(h, 2) / (2 * pow(sigma_s, 2)));
                sum += (wc * ws) * h;
                normalizer += wc * ws;
            }
        }

        mesh.U.row(v) = last_U.row(v) - mesh.WN.row(v) * (sum / normalizer);
    }

    return true;
}

