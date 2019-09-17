//
// Created by Yulin Liu on 7/31/19.
//

#include "Construction.hpp"



unsigned int Construction::resolution = 20;
unsigned int Construction::polyDegree = 0;
unsigned int Construction::layer = 2;
double Construction::bbmin_Degree = 1.2;
double Construction::bbmax_Degree = 1.2;
double Construction::wendlandRadius = 0.1;
Eigen::RowVector3d Construction::bb_min;
Eigen::RowVector3d Construction::bb_max;
Eigen::Vector3d Construction::dia_mM;


void Construction::computeConstrainedPoints(Mesh& mesh)
{
    // Grid bounds: axis-aligned bounding box
    bb_min = mesh.P.colwise().minCoeff();
    bb_max = mesh.P.colwise().maxCoeff();

    // Adjusting bounding box size based on given degree
    for(int i = 0; i < 3; i++)
    {
        if(bb_min(i) < 0)
        {
            bb_min(i) += bb_min(i) * (bbmin_Degree - 1);
        } else{
            bb_min(i) -= bb_min(i) * (bbmin_Degree - 1);
        }
        if(bb_max(i) < 0)
        {
            bb_max(i) -= bb_max(i) * (bbmax_Degree - 1);
        } else{
            bb_max(i) += bb_max(i) * (bbmax_Degree - 1);
        }

    }

    // Compute the bounding_box_diagonnal
    dia_mM = bb_max - bb_min;
    double epsilon = dia_mM.norm() * 0.01;

    // Grid spacing
    const double dx = dia_mM[0] / (double)(resolution - 1);
    const double dy = dia_mM[1] / (double)(resolution - 1);
    const double dz = dia_mM[2] / (double)(resolution - 1);

    // Initialize wendlanradius value to a reasonable one, the diagonal distance of each grid cell.
    wendlandRadius = sqrt( dx*dx + dy*dy + dz*dz);

    // For each point, compute the contrained points
    mesh.constrained_points.resize(mesh.P.rows() * 2, 3);
    mesh.constrained_values.resize(mesh.P.rows() * 2, 1);

    // Compute outside constrained points P_plus
    for (auto i = 0; i < mesh.P.rows(); ++i)
    {
        Eigen::RowVector3d P_plus;
        int flag = 1;
        while(flag)
        {
            flag = 0;
            P_plus = mesh.P.row(i) + mesh.N.row(i) * epsilon;

            // Check if P is the closest to P_plus
            double min_dis = epsilon;
            double dis = 0;
            for(auto j = 0; j < mesh.P.rows(); ++j)
            {
                if( j == i)
                {
                    continue;
                }
                dis = (mesh.P.row(j) - P_plus).norm();
                if(dis < min_dis)
                {
                    min_dis = dis;
                    flag = 1;
                    break;
                }
            }
            if(flag)
            {
                // flag = 1 means that P is not the closest to P_plus
                // halve epsilon to recompute new P_plus until flag=0
                epsilon /= 2;
            }
        }
        mesh.constrained_points.row(i) = P_plus;
        mesh.constrained_values(i) = epsilon;
    }

    // Compute inside constrianed points P_minus
    for(auto i = 0; i < mesh.P.rows(); ++i)
    {
        Eigen::RowVector3d P_minus;
        int flag = 1;
        while(flag)
        {
            flag = 0;
            P_minus = mesh.P.row(i) - mesh.N.row(i) * epsilon;

            // check if Pi is the closest to P_plus
            double min_dis = epsilon;
            double dis = 0;
            for(auto j = 0; j < mesh.P.rows(); ++j)
            {
                if(j == i)
                {
                    continue;
                }
                dis = (mesh.P.row(j) - P_minus).norm();
                if( dis < min_dis )
                {
                    min_dis = dis;
                    flag = 1;
                    break;
                }
            }
            if(flag == 1)
            {
                epsilon /= 2; // halve epsilon
            }
        }
        mesh.constrained_points.row(mesh.P.rows() + i) = P_minus;
        mesh.constrained_values(mesh.P.rows() + i) = -epsilon ;
    }
}


void Construction::computeInterpolatedPoints(Mesh& mesh) {
    // Make grid, evaluate interpolant at each grid point
    createGrid(mesh);

    // get grid lines
    getLines(mesh);

    // Code for coloring and displaying the grid points and lines
    // Assumes that grid_values and grid_points have been correctly assigned.
    mesh.grid_colors.setZero(mesh.grid_points.rows(), 3);

    // Build color map
    for (int i = 0; i < mesh.grid_points.rows(); ++i) {
        double value = mesh.grid_values(i);
        if (value < 0) {
            mesh.grid_colors(i, 1) = 1;
        }
        else {
            if (value > 0)
                mesh.grid_colors(i, 0) = 1;
        }
    }
}

bool Construction::MarchingCube(Mesh& mesh)
{
    // Code for computing the mesh (V,F) from grid_points and grid_values
    if ((mesh.grid_points.rows() == 0) || (mesh.grid_values.rows() == 0)) {
        std::cerr << "Not enough data for Marching Cubes !" << std::endl;
        return false;
    }
    // Run marching cubes
    igl::copyleft::marching_cubes(mesh.grid_values, mesh.grid_points,resolution, resolution, resolution, mesh.V, mesh.F);
    if (mesh.V.rows() == 0) {
        std::cerr << "Marching Cubes failed!" << std::endl;
        return false;
    }

    return true;
}

// Creates a grid_points array for the simple sphere example. The points are
// stacked into a single matrix, ordered first in the x, then in the y and
// then in the z direction.
void Construction::createGrid(Mesh& mesh) {

    // Grid spacing
    const double dx = dia_mM[0] / (double)(resolution - 1);
    const double dy = dia_mM[1] / (double)(resolution - 1);
    const double dz = dia_mM[2] / (double)(resolution - 1);


    // Binning each point into the corresponding cubic cell
    std::list<int> p_dic[(resolution - 1)*(resolution - 1)*(resolution - 1)]; //dictionary for on surface points, storing the row-index
    std::list<int> cp_dic[(resolution - 1)*(resolution - 1)*(resolution - 1)]; //dictionary for off-surface points
    for(auto i = 0; i < mesh.P.rows(); i++)
    {
        // On-surface points
        Eigen::RowVector3d p = mesh.P.row(i) - bb_min;  // relative location in grids
        int cell_index;
        auto x = int(p[0] / dx);
        auto y = int(p[1] / dy);
        auto z = int(p[2] / dz);
        // omit points that are out of bounding box
        if( x < 19 && y < 19 && z < 19)
        {
            // bottom-left grid point of the cell as the cell index
            cell_index = x + (resolution - 1) * (y + (resolution - 1) * z);
            p_dic[cell_index].push_back(i);
        }

        // Off-surface points
        // P_plus
        p = mesh.constrained_points.row(i) - bb_min;
        x = int(p[0] / dx);
        y = int(p[1] / dy);
        z = int(p[2] / dz);
        if( x < 19 && y < 19 && z < 19)
        {
            cell_index = x + (resolution - 1) * (y + (resolution - 1) * z);
            cp_dic[cell_index].push_back(i);
        }
        //P_minus
        p = mesh.constrained_points.row(mesh.P.rows() + i) - bb_min;
        x = int(p[0] / dx);  // x: 0 ~ 18
        y = int(p[1] / dy);
        z = int(p[2] / dz);
        if( x < 19 && y < 19 && z < 19)
        {
            cell_index = x + (resolution - 1) * (y + (resolution - 1) * z);
            cp_dic[cell_index].push_back(mesh.P.rows() + i);
        }
    }


    // 3D positions of the grid points
    mesh.grid_points.resize(resolution * resolution * resolution, 3);
    // Create each gridpoint
    for (unsigned int x = 0; x < resolution; ++x) {
        for (unsigned int y = 0; y < resolution; ++y) {
            for (unsigned int z = 0; z < resolution; ++z) {
                // Linear index of the point at (x,y,z)
                int index = x + resolution * (y + resolution * z);
                // 3D point at (x,y,z)
                mesh.grid_points.row(index) = bb_min + Eigen::RowVector3d(x * dx, y * dy, z * dz);
            }
        }
    }

    // Evaluate local MLS interpolant at each grid point x
    // Scalar values of the grid points (the implicit function values)
    mesh.grid_values.resize(resolution * resolution * resolution);
    for (unsigned int x = 0; x < resolution; ++x) {
        for (unsigned int y = 0; y < resolution; ++y) {
            for (unsigned int z = 0; z < resolution; ++z) {
                int index = x + resolution * (y + resolution * z);
                std::list<int> neighbors = computeNeighborCellIndex(x, y, z, layer, resolution);

                // Compute coefficients a(x, y, z);
                // Construct matrix B, matrix W and vector d
                Eigen::MatrixXd B;
                Eigen::MatrixXd W;
                Eigen::VectorXd d;
                Eigen::RowVectorXd fx;
                // Compute matrix size
                int length = 0;
                std::list<int> cp_neighbors;
                for(auto each : neighbors)
                {
                    for(auto &cp_row : cp_dic[each]) {
                        if( (mesh.constrained_points.row(cp_row) - mesh.grid_points.row(index)).norm() < wendlandRadius)
                        {
                            length++;
                            cp_neighbors.push_back(cp_row);
                        }
                    }
                }
                if(length == 0)
                {
                    // no constraint points are with wendlandRadius of the evaluation grid point
                    mesh.grid_values[index] = 1;
                    continue;
                }
                else
                {
                    W.resize(length, length);
                    W.setZero();
                    d.resize(length, 1);
                    if(polyDegree == 0)
                    {
                        B.resize(length, 1);
                        fx.resize(1);
                        fx << 1;
                    }
                    else if(polyDegree == 1)
                    {
                        B.resize(length, 4);
                        fx.resize(4);
                        double _x = mesh.grid_points.row(index)(0);
                        double _y = mesh.grid_points.row(index)(1);
                        double _z = mesh.grid_points.row(index)(2);
                        fx << 1, _x, _y, _z;
                    }
                    else if(polyDegree == 2)
                    {
                        B.resize(length, 10);
                        fx.resize(10);
                        double _x = mesh.grid_points.row(index)(0);
                        double _y = mesh.grid_points.row(index)(1);
                        double _z = mesh.grid_points.row(index)(2);
                        fx << 1, _x, _y, _z,
                                _x * _x, _y * _y, _z * _z,
                                _x * _y, _x * _z, _y * _z;
                    }
                    int row = 0;
                    for(auto &cp_row : cp_neighbors)
                    {
                        Eigen::RowVector3d cp = mesh.constrained_points.row(cp_row);
                        double dis = (cp - mesh.grid_points.row(index)).norm();
                        double w = pow((1 - dis / (double) wendlandRadius), 4) * (4 * dis / (double) wendlandRadius + 1);
                        W(row, row) = w;
                        d(row) = mesh.constrained_values(cp_row);
                        if(polyDegree == 0)
                        {
                            B.row(row++) << 1;
                        }
                        else if(polyDegree == 1)
                        {
                            // 1, x, y, z
                            B.row(row++) << 1, cp(0), cp(1), cp(2);
                        }
                        else if(polyDegree == 2)
                        {
                            // 1, x, y, z, xx, yy, zz, xy, xz, yz
                            B.row(row++) << 1, cp(0), cp(1), cp(2),
                                    cp(0)*cp(0), cp(1)*cp(1), cp(2)*cp(2),
                                    cp(0)*cp(1), cp(0)*cp(2), cp(1)*cp(2);
                        } else{
                            std::cout << "polyDegree is too large!" << std::endl;
                            exit(1);
                        }

                    }
                    // Compute Interpolant
                    mesh.grid_values[index] =  fx * ((B.transpose() * W * B).inverse() * (B.transpose() * W * d));
                }

            }
        }
    }
}


std::list<int> Construction::computeNeighborCellIndex(int x, int y, int z, int layer, int resolution)
{
    // layer = 1, find up to 8 neighbor cells, 2 * 2 * 2 around the grid point
    // layer = 2, find up to 64 neighbor cells, 4 * 4 * 4
    std::list<int> neighbors;
    std::list<int> valid_x;
    std::list<int> valid_y;
    std::list<int> valid_z;
    for(auto i = 0; i < layer; i++)
    {
        // valid_x
        if(x + i < resolution - 1)
        {
            valid_x.push_back(x + i);
        }
        if( (x - 1 - i) > 0)
        {
            valid_x.push_back(x - 1 - i);
        }
        // valid_y
        if(y + i < resolution - 1)
        {
            valid_y.push_back(y + i);
        }
        if( (y - 1 - i) >  0)
        {
            valid_y.push_back(y - 1 - i);
        }
        // valid_z
        if(z + i < resolution - 1)
        {
            valid_z.push_back(z + i);
        }
        if((z - 1 - i) > 0)
        {
            valid_z.push_back(z - 1 - i);
        }
    }
    for(auto &x_ : valid_x)
    {
        for(auto &y_ : valid_y)
        {
            for(auto &z_ : valid_z)
            {
                int cell_index = x_ + (resolution - 1) * (y_ + (resolution - 1) * z_);
                neighbors.push_back(cell_index);
            }
        }
    }
    return neighbors;
}


// Code to display the grid lines given a grid structure of the given form.
// Assumes grid_points have been correctly assigned
void Construction::getLines(Mesh& mesh) {
    mesh.grid_lines.resize(3 * mesh.grid_points.rows(), 6);
    int numLines = 0;

    for (unsigned int x = 0; x<resolution; ++x) {
        for (unsigned int y = 0; y < resolution; ++y) {
            for (unsigned int z = 0; z < resolution; ++z) {
                int index = x + resolution * (y + resolution * z);
                if (x < resolution - 1) {
                    int index1 = (x + 1) + y * resolution + z * resolution * resolution;
                    mesh.grid_lines.row(numLines++) << mesh.grid_points.row(index), mesh.grid_points.row(index1);
                }
                if (y < resolution - 1) {
                    int index1 = x + (y + 1) * resolution + z * resolution * resolution;
                    mesh.grid_lines.row(numLines++) << mesh.grid_points.row(index), mesh.grid_points.row(index1);
                }
                if (z < resolution - 1) {
                    int index1 = x + y * resolution + (z + 1) * resolution * resolution;
                    mesh.grid_lines.row(numLines++) << mesh.grid_points.row(index), mesh.grid_points.row(index1);
                }
            }
        }
    }

    mesh.grid_lines.conservativeResize(numLines, Eigen::NoChange);
}


