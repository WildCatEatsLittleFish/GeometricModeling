//
// Created by Yulin Liu on 8/6/19.
//

#include "Handle.hpp"

Handle::Handle(Mesh& m) : mesh(m)
{
    handle_id.setConstant(mesh.V.rows(), 1, -1);
}

void Handle::get_new_handle_locations()
{
    int count = 0;
    for (long vi = 0; vi < mesh.V.rows(); ++vi)
        if (handle_id[vi] >= 0)
        {
            Eigen::RowVector3f goalPosition = mesh.V.row(vi).cast<float>();
            if (handle_id[vi] == moving_handle) {
                if (mouse_mode == TRANSLATE)
                    goalPosition += translation;
                else if (mouse_mode == ROTATE) {
                    goalPosition -= handle_centroids.row(moving_handle).cast<float>();
                    igl::rotate_by_quat(goalPosition.data(), rotation.data(), goalPosition.data());
                    goalPosition += handle_centroids.row(moving_handle).cast<float>();
                }
            }
            handle_vertex_positions.row(count++) = goalPosition.cast<double>();
        }
}


void Handle::compute_handle_centroids()
{
    //compute centroids of handles
    int num_handles = handle_id.maxCoeff()+1;
    handle_centroids.setZero(num_handles,3);

    Eigen::VectorXi num; num.setZero(num_handles,1);
    for (long vi = 0; vi<mesh.V.rows(); ++vi)
    {
        int r = handle_id[vi];
        if ( r!= -1)
        {
            handle_centroids.row(r) += mesh.V.row(vi);
            num[r]++;
        }
    }

    for (long i = 0; i<num_handles; ++i)
        handle_centroids.row(i) = handle_centroids.row(i).array()/num[i];

}

//computes translation for the vertices of the moving handle based on the mouse motion
Eigen::Vector3f Handle::computeTranslation (igl::opengl::glfw::Viewer& viewer,
                                    int mouse_x,
                                    int from_x,
                                    int mouse_y,
                                    int from_y,
                                    Eigen::RowVector3d pt3D)
{
    Eigen::Matrix4f modelview = viewer.core.view;
    //project the given point (typically the handle centroid) to get a screen space depth
    Eigen::Vector3f proj = igl::project(pt3D.transpose().cast<float>().eval(),
                                        modelview,
                                        viewer.core.proj,
                                        viewer.core.viewport);
    float depth = proj[2];

    double x, y;
    Eigen::Vector3f pos1, pos0;

    //unproject from- and to- points
    x = mouse_x;
    y = viewer.core.viewport[3] - mouse_y;
    pos1 = igl::unproject(Eigen::Vector3f(x,y,depth),
                          modelview,
                          viewer.core.proj,
                          viewer.core.viewport);


    x = from_x;
    y = viewer.core.viewport[3] - from_y;
    pos0 = igl::unproject(Eigen::Vector3f(x,y,depth),
                          modelview,
                          viewer.core.proj,
                          viewer.core.viewport);

    //translation is the vector connecting the two
    Eigen::Vector3f translation = pos1 - pos0;
    return translation;

}

//computes translation for the vertices of the moving handle based on the mouse motion
Eigen::Vector4f Handle::computeRotation(igl::opengl::glfw::Viewer& viewer,
                                int mouse_x,
                                int from_x,
                                int mouse_y,
                                int from_y,
                                Eigen::RowVector3d pt3D)
{

    Eigen::Vector4f rotation;
    rotation.setZero();
    rotation[3] = 1.;

    Eigen::Matrix4f modelview = viewer.core.view;

    //initialize a trackball around the handle that is being rotated
    //the trackball has (approximately) width w and height h
    double w = viewer.core.viewport[2]/8;
    double h = viewer.core.viewport[3]/8;

    //the mouse motion has to be expressed with respect to its center of mass
    //(i.e. it should approximately fall inside the region of the trackball)

    //project the given point on the handle(centroid)
    Eigen::Vector3f proj = igl::project(pt3D.transpose().cast<float>().eval(),
                                        modelview,
                                        viewer.core.proj,
                                        viewer.core.viewport);
    proj[1] = viewer.core.viewport[3] - proj[1];

    //express the mouse points w.r.t the centroid
    from_x -= proj[0]; mouse_x -= proj[0];
    from_y -= proj[1]; mouse_y -= proj[1];

    //shift so that the range is from 0-w and 0-h respectively (similarly to a standard viewport)
    from_x += w/2; mouse_x += w/2;
    from_y += h/2; mouse_y += h/2;

    //get rotation from trackball
    Eigen::Vector4f drot = viewer.core.trackball_angle.coeffs();
    Eigen::Vector4f drot_conj;
    igl::quat_conjugate(drot.data(), drot_conj.data());
    igl::trackball(w, h, float(1.), rotation.data(), from_x, from_y, mouse_x, mouse_y, rotation.data());

    //account for the modelview rotation: prerotate by modelview (place model back to the original
    //unrotated frame), postrotate by inverse modelview
    Eigen::Vector4f out;
    igl::quat_mult(rotation.data(), drot.data(), out.data());
    igl::quat_mult(drot_conj.data(), out.data(), rotation.data());
    return rotation;
}


void Handle::applySelection()
{
    int index = handle_id.maxCoeff()+1;
    for (int i =0; i<selected_v.rows(); ++i)
    {
        const int selected_vertex = selected_v[i];
        if (handle_id[selected_vertex] == -1)
            handle_id[selected_vertex] = index;
    }
    selected_v.resize(0,1);

    onNewHandleID();
}

void Handle::onNewHandleID()
{
    //store handle vertices too
    int numFree = (handle_id.array() == -1).cast<int>().sum();
    int num_handle_vertices = mesh.V.rows() - numFree;
    handle_vertices.setZero(num_handle_vertices);
    handle_vertex_positions.setZero(num_handle_vertices,3);

    int count = 0;
    for (long vi = 0; vi<mesh.V.rows(); ++vi)
        if(handle_id[vi] >=0)
            handle_vertices[count++] = vi;

    compute_handle_centroids();
}

