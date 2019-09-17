//
// Created by Yulin Liu on 8/6/19.
//

#include "Event.hpp"


//activate this for alternate UI (easier to debug)
#define UPDATE_ONLY_ON_UP


// Color Configuration
#define MAXNUMREGIONS 7
double regionColors[MAXNUMREGIONS][3]= {
        {0, 0.4470, 0.7410},
        {0.8500, 0.3250, 0.0980},
        {0.9290, 0.6940, 0.1250},
        {0.4940, 0.1840, 0.5560},
        {0.4660, 0.6740, 0.1880},
        {0.3010, 0.7450, 0.9330},
        {0.6350, 0.0780, 0.1840},
};


//EventHandler::EventHandler(Mesh& m, Handle& h) : mesh(m), handle(h)
//{
////    construction_e = ConstructionEvent::None;
////    smoothing_e = SmoothingEvent::None;
//
//    doit = false;
//    down_mouse_x = -1;
//    down_mouse_y = -1;
//}




//Mesh& EventHandler::mesh = m;
//Handle& EventHandler::handle = Mesh();

Mesh* EventHandler::mesh = nullptr;
Handle* EventHandler::handle = nullptr;
bool EventHandler::doit = false;
int EventHandler::down_mouse_x = -1;
int EventHandler::down_mouse_y = -1;



bool EventHandler::constructingMesh(igl::opengl::glfw::Viewer& viewer, ConstructionEvent e)
{
    switch(e)
    {
        case ShowPoints:
        {
            // Show imported points
            viewer.data().clear();
            viewer.core.align_camera_center(mesh->P);
            viewer.data().point_size = 11;
            viewer.data().add_points(mesh->P, Eigen::RowVector3d(0,0,0));
            break;
        }

        case ShowConstrainedPoints:
        {
            // Show all constraints
            viewer.data().clear();
            viewer.core.align_camera_center(mesh->P);

            /****** Computing auxiliary constraint points here *****/
            Construction::computeConstrainedPoints(*mesh);

            /******* Displaying all points ********/

            // Color the constrained points P_plus and P_minus
            viewer.data().add_points(mesh->P, Eigen::RowVector3d(0, 0, 1)); // On-surface: blue
            viewer.data().add_points(mesh->constrained_points.block(0, 0, mesh->P.rows(), 3), Eigen::RowVector3d(1, 0, 0)); // Outside_surface: red
            viewer.data().add_points(mesh->constrained_points.block(mesh->P.rows(), 0, mesh->P.rows(), 3), Eigen::RowVector3d(0, 1, 0)); // Inside_surface: green
            break;

        }

        case CreateGrids:
        {
            // Show grid points with colored nodes and connected with lines
            viewer.data().clear();
            viewer.core.align_camera_center(mesh->P);

            Construction::computeInterpolatedPoints(*mesh);

            // Draw lines and points
            viewer.data().point_size = 8;
            viewer.data().add_points(mesh->grid_points, mesh->grid_colors);
            viewer.data().add_edges(mesh->grid_lines.block(0, 0, mesh->grid_lines.rows(), 3),
                                    mesh->grid_lines.block(0, 3, mesh->grid_lines.rows(), 3),
                                    Eigen::RowVector3d(0.8, 0.8, 0.8));
            break;
        }

        case ConstructMesh:
        {
            // Show reconstructed mesh
            viewer.data().clear();

            Construction::MarchingCube(*mesh);
            mesh->computeData();

            viewer.data().set_mesh(mesh->V, mesh->F);
            viewer.data().show_lines = true;
            viewer.data().show_faces = true;
            viewer.data().set_normals(mesh->FN);

            mesh->C.resize(mesh->V.rows(), 3);
            Eigen::VectorXd Z = mesh->V.col(2);
            igl::jet(Z, true, mesh->C);
            viewer.data().set_colors(mesh->C);

            break;
        }
        case None:
        {
            viewer.data().clear();
            break;
        }

    }
    return true;
}


bool EventHandler::mouse_down(igl::opengl::glfw::Viewer& viewer, int, int)
{
    int fid_ray;
    Eigen::Vector3f bary;
    // Cast a ray in the view direction starting from the mouse position
    double x = viewer.current_mouse_x;
    double y = viewer.core.viewport(3) - viewer.current_mouse_y;
    if(igl::unproject_onto_mesh(Eigen::Vector2f(x,y), viewer.core.view,
                                viewer.core.proj, viewer.core.viewport, mesh->V, mesh->F, fid_ray, bary))
    {

        float a, b, c;
        a = bary[0];
        b = bary[1];
        c = bary[2];


        if(a >= b and a >= c)
        {
            BilateralDenoising::chosen_vertex = mesh->F(fid_ray, 0);
        }
        if(b >= a and b >= c)
        {
            BilateralDenoising::chosen_vertex = mesh->F(fid_ray, 1);
        }
        if(c >= a and c >= b)
        {
            BilateralDenoising::chosen_vertex = mesh->F(fid_ray, 2);
        }

        // Clear the previous chosen point
        viewer.data().clear();
        viewer.data().set_mesh(mesh->U, mesh->F);
        viewer.data().set_colors(mesh->C);

        mesh->P = mesh->V.row(BilateralDenoising::chosen_vertex);
        viewer.data().add_points(mesh->P,Eigen::RowVector3d(1,0,0));

        return true;
    }
    return false;
};

bool EventHandler::mouse_move(igl::opengl::glfw::Viewer& viewer, int mouse_x, int mouse_y) { return true; }
bool EventHandler::mouse_up(igl::opengl::glfw::Viewer& viewer, int button, int modifier) { return true; }
bool EventHandler::pre_draw(igl::opengl::glfw::Viewer& viewer) { return true; }
bool EventHandler::key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifiers) { return true; }

bool EventHandler::smoothingMesh(igl::opengl::glfw::Viewer& viewer, SmoothingEvent e)
{
    switch(e)
    {
        case Laplacian:
        {
            LaplacianSmoothing::computeSmoothedVertex(*mesh);
            // Send new positions, update normals, recenter
            viewer.data().set_vertices(mesh->U);
            viewer.data().compute_normals();
            viewer.core.align_camera_center(mesh->U, mesh->F);
            break;
        }

        case BilateralDenoising:
        {
            BilateralDenoising::computeDenoisedVertex(*mesh);
            // Send new positions, update normals, recenter
            viewer.data().set_vertices(mesh->U);
            viewer.data().compute_normals();
            viewer.core.align_camera_center(mesh->U, mesh->F);
            break;
        }

        case IterationDenoising:
        {
            BilateralDenoising::iterationDenoising(*mesh);
            // Send new positions, update normals, recenter
            viewer.data().set_vertices(mesh->U);
            viewer.data().compute_normals();
            viewer.core.align_camera_center(mesh->U, mesh->F);
            break;
        }

        case BackToLast:
        {
            BilateralDenoising::resetToLast(*mesh);
            // Send new positions, update normals, recenter
            viewer.data().set_vertices(mesh->U);
            viewer.data().compute_normals();
            viewer.core.align_camera_center(mesh->U, mesh->F);
            break;
        }

        case ResetMesh:
        {
            BilateralDenoising::resetToOriginal(*mesh);
            // Send new positions, update normals, recenter
            viewer.data().set_vertices(mesh->U);
            viewer.data().compute_normals();
            viewer.core.align_camera_center(mesh->U, mesh->F);
            break;
        }



    }
    return true;
}


bool EventHandler::callback_mouse_down(igl::opengl::glfw::Viewer& viewer, int button, int modifier)
{
    if (button == (int) igl::opengl::glfw::Viewer::MouseButton::Right)
        return false;

    down_mouse_x = viewer.current_mouse_x;
    down_mouse_y = viewer.current_mouse_y;

    if (handle->mouse_mode == SELECT)
    {
        if (handle->lasso->strokeAdd(viewer.current_mouse_x, viewer.current_mouse_y) >=0)
            doit = true;
        else
            handle->lasso->strokeReset();
    }
    else if ((handle->mouse_mode == TRANSLATE) || (handle->mouse_mode == ROTATE))
    {
        int vi = handle->lasso->pickVertex(viewer.current_mouse_x, viewer.current_mouse_y);
        if(vi>=0 && handle->handle_id[vi]>=0)  //if a region was found, mark it for translation/rotation
        {
            handle->moving_handle = handle->handle_id[vi];
            handle->get_new_handle_locations();
            doit = true;
        }
    }
    return doit;

}

bool EventHandler::callback_mouse_move(igl::opengl::glfw::Viewer& viewer, int mouse_x, int mouse_y)
{
    if (!doit)
        return false;
    if (handle->mouse_mode == SELECT)
    {
        handle->handles_changed = true;
        handle->lasso->strokeAdd(mouse_x, mouse_y);
        return true;
    }
    if ((handle->mouse_mode == TRANSLATE) || (handle->mouse_mode == ROTATE))
    {
        if (handle->mouse_mode == TRANSLATE) {
            handle->translation = handle->computeTranslation(viewer,
                                             mouse_x,
                                             down_mouse_x,
                                             mouse_y,
                                             down_mouse_y,
                                                           handle->handle_centroids.row(handle->moving_handle));
        }
        else {
            handle->rotation = handle->computeRotation(viewer,
                                       mouse_x,
                                       down_mouse_x,
                                       mouse_y,
                                       down_mouse_y,
                                                     handle->handle_centroids.row(handle->moving_handle));
        }
        handle->get_new_handle_locations();
#ifndef UPDATE_ONLY_ON_UP
        Deformation::multiResolutionSurfaceMethod(mesh, handle);
        down_mouse_x = mouse_x;
        down_mouse_y = mouse_y;
#endif
        return true;

    }
    return false;
}

bool EventHandler::callback_mouse_up(igl::opengl::glfw::Viewer& viewer, int button, int modifier)
{
    if (!doit)
        return false;
    doit = false;
    if (handle->mouse_mode == SELECT)
    {
        handle->selected_v.resize(0,1);
        handle->lasso->strokeFinish(handle->selected_v);
        return true;
    }

    if ((handle->mouse_mode == TRANSLATE) || (handle->mouse_mode == ROTATE))
    {
#ifdef UPDATE_ONLY_ON_UP
        if(handle->moving_handle>=0)
            Deformation::multiResolutionSurfaceMethod(*mesh, *handle);
#endif
        handle->translation.setZero();
        handle->rotation.setZero();
        handle->rotation[3] = 1.0f;
        handle->moving_handle = -1;

        handle->compute_handle_centroids();

        return true;
    }

    return false;
};

bool EventHandler::callback_pre_draw(igl::opengl::glfw::Viewer& viewer)
{
    // Initialize vertex colors
//    vertex_colors = Eigen::MatrixXd::Constant(mesh->V.rows(),3,.9);
   mesh->C = Eigen::MatrixXd::Constant(mesh->V.rows(),3,.9);

    //first, color constraints
    int num = handle->handle_id.maxCoeff();
    if (num ==0)
        num = 1;
    for (int i = 0; i<mesh->V.rows(); ++i)
        if (handle->handle_id[i]!=-1)
        {
            int r = handle->handle_id[i] % MAXNUMREGIONS;
//            vertex_colors.row(i) << regionColors[r][0], regionColors[r][1], regionColors[r][2];
            mesh->C.row(i) << regionColors[r][0], regionColors[r][1], regionColors[r][2];
        }
    //then, color selection
    for (int i = 0; i<handle->selected_v.size(); ++i)
//        vertex_colors.row(handle->selected_v[i]) << 131./255, 131./255, 131./255.;
        mesh->C.row(handle->selected_v[i]) << 131./255, 131./255, 131./255.;
//    viewer.data().set_colors(vertex_colors);
    viewer.data().set_colors(mesh->C);


    //clear points and lines
    viewer.data().set_points(Eigen::MatrixXd::Zero(0,3), Eigen::MatrixXd::Zero(0,3));
    viewer.data().set_edges(Eigen::MatrixXd::Zero(0,3), Eigen::MatrixXi::Zero(0,3), Eigen::MatrixXd::Zero(0,3));

    //draw the stroke of the selection
    for (unsigned int i = 0; i<handle->lasso->strokePoints.size(); ++i)
    {
        viewer.data().add_points(handle->lasso->strokePoints[i],Eigen::RowVector3d(0.4,0.4,0.4));
        if (i>1)
            viewer.data().add_edges(handle->lasso->strokePoints[i-1], handle->lasso->strokePoints[i], Eigen::RowVector3d(0.7,0.7,.7));
    }

    // update the vertex position all the time
    viewer.data().V.resize(mesh->V.rows(),3);
    viewer.data().V << mesh->V;

    viewer.data().dirty |= igl::opengl::MeshGL::DIRTY_POSITION;

#ifdef UPDATE_ONLY_ON_UP
    //draw only the moving parts with a white line
  if (handle->moving_handle>=0)
  {
    Eigen::MatrixXd edges(3*mesh->F.rows(),6);
    int num_edges = 0;
    for (int fi = 0; fi<mesh->F.rows(); ++fi)
    {
      int firstPickedVertex = -1;
      for(int vi = 0; vi<3 ; ++vi)
        if (handle->handle_id[mesh->F(fi,vi)] == handle->moving_handle)
        {
          firstPickedVertex = vi;
          break;
        }
      if(firstPickedVertex==-1)
        continue;


      Eigen::Matrix3d points;
      for(int vi = 0; vi<3; ++vi)
      {
        int vertex_id = mesh->F(fi,vi);
        if (handle->handle_id[vertex_id] == handle->moving_handle)
        {
          int index = -1;
          // if face is already constrained, find index in the constraints
          (handle->handle_vertices.array()-vertex_id).cwiseAbs().minCoeff(&index);
          points.row(vi) = handle->handle_vertex_positions.row(index);
        }
        else
          points.row(vi) =  mesh->V.row(vertex_id);

      }
      edges.row(num_edges++) << points.row(0), points.row(1);
      edges.row(num_edges++) << points.row(1), points.row(2);
      edges.row(num_edges++) << points.row(2), points.row(0);
    }
    edges.conservativeResize(num_edges, Eigen::NoChange);
    viewer.data().add_edges(edges.leftCols(3), edges.rightCols(3), Eigen::RowVector3d(0.9,0.9,0.9));

  }
#endif
    return false;
}

bool EventHandler::callback_key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifiers)
{
    bool handled = false;
    if (key == 'S')
    {
        handle->mouse_mode = SELECT;
        handled = true;
    }

    if ((key == 'T') && (modifiers == IGL_MOD_ALT))
    {
        handle->mouse_mode = TRANSLATE;
        handled = true;
    }

    if ((key == 'R') && (modifiers == IGL_MOD_ALT))
    {
        handle->mouse_mode = ROTATE;
        handled = true;
    }
    if (key == 'A')
    {
        handle->applySelection();
        callback_key_down(viewer, '1', 0);
        handled = true;
    }

    return handled;
}

