//
// Created by Yulin Liu on 8/6/19.
//

#include "Deformation.hpp"


Eigen::SimplicialCholesky<Eigen::SparseMatrix<double>, Eigen::RowMajor >  Deformation::solver;

// Flag to indicate whether to recompute A_ff
bool Deformation::handles_changed = false;

// 4 different deformed mesh
Eigen::MatrixXd Deformation::S;
Eigen::MatrixXd Deformation::B;
Eigen::MatrixXd Deformation::B2;
Eigen::MatrixXd Deformation::S2;

bool Deformation::multiResolutionSurfaceMethod(Mesh& mesh, Handle& handle)
{
    /**** Computing the deformation from handle_vertex_positions and handle_vertices ****/
    //igl::slice_into(handle_vertex_positions, handle_vertices, 1, V);
    S = mesh.V;

    /*********** Step 1: Smooth Mesh *************/

    // Remove high-frequency details and Solve the bilaplacian system to smooth the mesh
    Eigen::VectorXi fRows;
    Eigen::VectorXi cRows;
    fRows.resize(mesh.V.rows() - handle.handle_vertices.rows());
    cRows.resize(handle.handle_vertices.rows());
    std::unordered_set<int> c_set;
    Eigen::MatrixXd vc;
    vc.resize(handle.handle_vertices.rows(), 3);
    for(unsigned v = 0 ; v < handle.handle_vertices.rows(); ++v)
    {
        c_set.insert(handle.handle_vertices(v));
        cRows(v) = handle.handle_vertices(v);
        vc.row(v) = mesh.V.row(handle.handle_vertices(v));
    }
    int count = 0;
    for(unsigned v = 0 ; v < mesh.V.rows(); ++v)
    {
        if(c_set.find(v) == c_set.end())
        {
            // remaining free vertices
            fRows(count++) = v;
        }
    }
    if(handles_changed)
    {
        std::cout << "Handles changed" << std::endl;
        igl::slice(mesh.A, fRows, fRows, mesh.Aff);
        igl::slice(mesh.A, fRows, cRows,  mesh.Afc);

        Eigen::SparseMatrix<double> A_ff(mesh.Aff.rows(), mesh.Aff.cols());
        std::vector< Eigen::Triplet<double> > t;
        for(unsigned r = 0; r < mesh.Aff.rows();++r)
        {
            for(unsigned c = 0; c < mesh.Aff.cols(); ++c)
            {
                if( mesh.Aff(r, c) != 0)
                {
                    t.push_back(Eigen::Triplet<double>(r,c, mesh.Aff(r, c)));
                }
            }
        }
        A_ff.setFromTriplets(t.begin(), t.end());
        solver.compute(A_ff); // Factorize the free part of the ststen
        assert(solver.info()==Eigen::Success);
        handles_changed = false;
    }

    Eigen::MatrixXd vf = solver.solve(- mesh.Afc * vc);
    assert(solver.info()==Eigen::Success);

    Eigen::Vector3i cols;
    cols(0) = 0;
    cols(1) = 1;
    cols(2) = 2;
    B = mesh.V;
    igl::slice_into(vf, fRows, cols, B);
    std::cout << "finish smoothing from S to B" << std::endl;

    /*********** Step 2: Encode Displacements *************/

    // Compute per-vertex normals for surface B
    Eigen::MatrixXd N_vertices;
    igl::per_vertex_normals(B, mesh.F, N_vertices);


    // Construct Frame Basis
    // The edge chosen for B to construct frame basis xi
    Eigen::VectorXi basis_X;
    basis_X.resize(mesh.V.rows());
    // Coefficients of the frame basis
    Eigen::MatrixXd basis_cof;
    basis_cof.resize(mesh.V.rows(), 3);

    for(unsigned v = 0; v < mesh.V.rows(); v++)
    {
        std::vector<int> neighbors = mesh.VV[v];
        double max_length = 0;
        int longest_neighbor = neighbors[0];
        for(auto &each : neighbors)
        {
            double a = N_vertices.row(v).dot(mesh.V.row(each) - mesh.V.row(v));
            double b = (mesh.V.row(each) - mesh.V.row(v)).norm();
            double c = pow(pow(b, 2) - pow(a, 2), 0.5);
            if(max_length < c)
            {
                max_length = c;
                longest_neighbor = each;
            }
        }
        // Local Frame's Basis <xi, yi, ni>
        basis_X(v) = longest_neighbor;
        // Normalize the longest edge neighbor vector and call it xi
        Eigen::Vector3d ni = N_vertices.row(v);
        Eigen::Vector3d xi = (mesh.V.row(longest_neighbor) - mesh.V.row(v)).normalized();
        Eigen::Vector3d yi = ni.cross(xi);


        // Displacement vector
        Eigen::Vector3d di = mesh.V.row(v) - B.row(v);
        // Decompose the displacement vector in the frame's basis
        // dix, diy, din
        basis_cof(v, 0) = xi.dot(di);
        basis_cof(v, 1) = yi.dot(di);
        basis_cof(v, 2) = ni.dot(di);
    }

    /*********** Step 3: Deform B *************/

    // Deformed smooth mesh B'
    Eigen::MatrixXd vf_new = solver.solve(- mesh.Afc * handle.handle_vertex_positions);
    assert(solver.info()==Eigen::Success);
    B2.resize(mesh.V.rows(), 3);
    igl::slice_into(vf_new, fRows, cols, B2);
    igl::slice_into(handle.handle_vertex_positions, cRows, cols, B2);

    std::cout << "Finish deformed smooth mesh B to B2" << std::endl;


    /*********** Step 4: Add Transformed Detail *************/

    // Recompute per-vertex normals for surface B2
    Eigen::MatrixXd N_vertices2;
    igl::per_vertex_normals(B2, mesh.F, N_vertices2);

    // Compute new frame basis (xi', yi', ni')
    for(unsigned v = 0; v < mesh.V.rows(); v++)
    {
        Eigen::Vector3d ni = N_vertices2.row(v);
        Eigen::Vector3d xi = (mesh.V.row(basis_X(v)) - mesh.V.row(v)).normalized();
        Eigen::Vector3d yi = ni.cross(xi);


        // Construct the transformed displacement vectors
        Eigen::RowVector3d di = basis_cof(v, 0)*xi + basis_cof(v, 1)*yi + basis_cof(v, 2)*ni;

        // Add them to B2 to form S2
        S2.resize(mesh.V.rows(), 3);
        Eigen::RowVector3d vi_S2 = di + B2.row(v);
        S2.row(v) = vi_S2;
    }
    std::cout << "Finish adding detail from B2 to S2" << std::endl;

    // Show the final deformed mesh
    mesh.V = S2;
    return true;
};
