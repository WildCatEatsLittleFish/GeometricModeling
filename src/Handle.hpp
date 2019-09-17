//
// Created by Yulin Liu on 8/6/19.
//

#ifndef EX2_HANDLE_HPP
#define EX2_HANDLE_HPP

#include <Eigen/Core>
#include "Lasso.hpp"
#include "Mesh.hpp"


//mouse interaction
enum MouseMode { SELECT, TRANSLATE, ROTATE, NONE };

class Handle {

public:

    Mesh& mesh;
    MouseMode mouse_mode = NONE;
    // Flag to indicate whether to recompute A_ff
    bool handles_changed = false;

    //vertex-to-handle index, #V x1 (-1 if vertex is free)
    Eigen::VectorXi handle_id;
    //list of all vertices belonging to handles, #HV x1
    Eigen::VectorXi handle_vertices;
    //centroids of handle regions, #H x1
    Eigen::MatrixXd handle_centroids;
    //updated positions of handle vertices, #HV x3
    Eigen::MatrixXd handle_vertex_positions;
    //index of handle being moved
    int moving_handle = -1;
    //rotation and translation for the handle being moved
    Eigen::Vector3f translation;
    Eigen::Vector4f rotation;


    //for selecting vertices
    std::unique_ptr<Lasso> lasso;
    //list of currently selected vertices
    Eigen::VectorXi selected_v;


public:
    Handle() = default;
    Handle(Mesh& m);
    void get_new_handle_locations();
    void compute_handle_centroids();
    Eigen::Vector3f computeTranslation (igl::opengl::glfw::Viewer& viewer, int mouse_x, int from_x, int mouse_y, int from_y, Eigen::RowVector3d pt3D);
    Eigen::Vector4f computeRotation(igl::opengl::glfw::Viewer& viewer, int mouse_x, int from_x, int mouse_y, int from_y, Eigen::RowVector3d pt3D);


    void applySelection();
    void onNewHandleID();


};


#endif //EX2_HANDLE_HPP
