#include <igl/opengl/glfw/Viewer.h>
/*** insert any necessary libigl headers here ***/
#include <igl/opengl/glfw/imgui/ImGuiMenu.h>
#include <igl/opengl/glfw/imgui/ImGuiHelpers.h>
#include <imgui/imgui.h>

#include "Mesh.hpp"
#include "Construction.hpp"
#include "Smoothing.hpp"
#include "Event.hpp"



//Mesh mesh("../data/cat.off");
//int chosen_vertex = 0;

//bool callback_key_down(Viewer &viewer, unsigned char key, int modifiers) {
//    if (key == '1') {
//        // Show imported points
//        viewer.data().clear();
//        viewer.core.align_camera_center(mesh.P);
//        viewer.data().point_size = 11;
//        viewer.data().add_points(mesh.P, Eigen::RowVector3d(0,0,0));
//    }
//
//    if (key == '2') {
//        // Show all constraints
//        viewer.data().clear();
//        viewer.core.align_camera_center(mesh.P);
//
//        /****** Computing auxiliary constraint points here *****/
//        Construction::computeConstrainedPoints(mesh);
//
//        /******* Displaying all points ********/
//
//        // Color the constrained points P_plus and P_minus
//        viewer.data().add_points(mesh.P, Eigen::RowVector3d(0, 0, 1)); // On-surface: blue
//        viewer.data().add_points(mesh.constrained_points.block(0, 0, mesh.P.rows(), 3), Eigen::RowVector3d(1, 0, 0)); // Outside_surface: red
//        viewer.data().add_points(mesh.constrained_points.block(mesh.P.rows(), 0, mesh.P.rows(), 3), Eigen::RowVector3d(0, 1, 0)); // Inside_surface: green
//    }
//
//    if (key == '3') {
//        // Show grid points with colored nodes and connected with lines
//        viewer.data().clear();
//        viewer.core.align_camera_center(mesh.P);
//
//        Construction::computeInterpolatedPoints(mesh);
//
//        // Draw lines and points
//        viewer.data().point_size = 8;
//        viewer.data().add_points(mesh.grid_points, mesh.grid_colors);
//        viewer.data().add_edges(mesh.grid_lines.block(0, 0, mesh.grid_lines.rows(), 3),
//                                mesh.grid_lines.block(0, 3, mesh.grid_lines.rows(), 3),
//                                Eigen::RowVector3d(0.8, 0.8, 0.8));
//    }
//
//    if (key == '4') {
//        // Show reconstructed mesh
//        viewer.data().clear();
//
//        Construction::MarchingCube(mesh);
//        mesh.computeData();
//
//        viewer.data().set_mesh(mesh.V, mesh.F);
//        viewer.data().show_lines = true;
//        viewer.data().show_faces = true;
//        viewer.data().set_normals(mesh.FN);
//
//        mesh.C.resize(mesh.V.rows(), 3);
//        Eigen::VectorXd Z = mesh.V.col(2);
//        igl::jet(Z, true, mesh.C);
//        viewer.data().set_colors(mesh.C);
//
//        // export mesh in off format
//        //igl::writeOFF(input_file + ".out", mesh.V, mesh.F, C);
//    }
////    if(key == '5') {
////
////
////        // Using a non-axis aligned grid
////        mesh.alignDataGrid();
////
////        // Show grid points with colored nodes and connected with lines
////        viewer.data().clear();
////        viewer.core.align_camera_center(mesh.P);
////
////        // Draw lines and points
////        viewer.data().point_size = 8;
////        viewer.data().add_points(mesh.grid_points, mesh.grid_colors);
////        viewer.data().add_edges(mesh.grid_lines.block(0, 0, mesh.grid_lines.rows(), 3),
////                                mesh.grid_lines.block(0, 3, mesh.grid_lines.rows(), 3),
////                                Eigen::RowVector3d(0.8, 0.8, 0.8));
////    }
//    if(key == '5')
//    {
//        LaplacianSmoothing::computeSmoothedVertex(mesh);
//        // Send new positions, update normals, recenter
//        viewer.data().set_vertices(mesh.U);
//        viewer.data().compute_normals();
//        viewer.core.align_camera_center(mesh.U, mesh.F);
//    }
//
//    if(key == '6')
//    {
//        BilateralDenoising::computeDenoisedVertex(mesh);
//        // Send new positions, update normals, recenter
//        viewer.data().set_vertices(mesh.U);
//        viewer.data().compute_normals();
//        viewer.core.align_camera_center(mesh.U, mesh.F);
//    }
//
//    if(key == '7')
//    {
//
//        BilateralDenoising::iterationDenoising(mesh);
//        // Send new positions, update normals, recenter
//        viewer.data().set_vertices(mesh.U);
//        viewer.data().compute_normals();
//        viewer.core.align_camera_center(mesh.U, mesh.F);
//    }
//    if(key == '8')
//    {
//        BilateralDenoising::resetToLast(mesh);
//        // Send new positions, update normals, recenter
//        viewer.data().set_vertices(mesh.U);
//        viewer.data().compute_normals();
//        viewer.core.align_camera_center(mesh.U, mesh.F);
//    }
//    if(key == '9' )
//    {
//        BilateralDenoising::resetToOriginal(mesh);
//        // Send new positions, update normals, recenter
//        viewer.data().set_vertices(mesh.U);
//        viewer.data().compute_normals();
//        viewer.core.align_camera_center(mesh.U, mesh.F);
//    }
//
//    return true;
//}
//
//bool mouse_down(igl::opengl::glfw::Viewer& viewer, int, int)
//{
//    int fid_ray;
//    Eigen::Vector3f bary;
//    // Cast a ray in the view direction starting from the mouse position
//    double x = viewer.current_mouse_x;
//    double y = viewer.core.viewport(3) - viewer.current_mouse_y;
//    if(igl::unproject_onto_mesh(Eigen::Vector2f(x,y), viewer.core.view,
//                                viewer.core.proj, viewer.core.viewport, mesh.V, mesh.F, fid_ray, bary))
//    {
//
//        float a, b, c;
//        a = bary[0];
//        b = bary[1];
//        c = bary[2];
//
//
//        if(a >= b and a >= c)
//        {
//            BilateralDenoising::chosen_vertex = mesh.F(fid_ray, 0);
//        }
//        if(b >= a and b >= c)
//        {
//            BilateralDenoising::chosen_vertex = mesh.F(fid_ray, 1);
//        }
//        if(c >= a and c >= b)
//        {
//            BilateralDenoising::chosen_vertex = mesh.F(fid_ray, 2);
//        }
//
//        // Clear the previous chosen point
//        viewer.data().clear();
//        viewer.data().set_mesh(mesh.U, mesh.F);
//        viewer.data().set_colors(mesh.C);
//
//        mesh.P = mesh.V.row(BilateralDenoising::chosen_vertex);
//        viewer.data().add_points(mesh.P,Eigen::RowVector3d(1,0,0));
//
//        return true;
//    }
//    return false;
//};

//extern Mesh mesh("../data/cat.off");
//extern Handle handle(mesh);

int main(int argc, char *argv[]) {
//    if (argc != 2) {
//        cout << "Usage ex2_bin mesh.off" << endl;
//        exit(0);
//    }
//
//    // Read points and normals
//    igl::readOFF(argv[1],P,F,N);

    //igl::readOFF("../data/cat.off",P,F,N);

//    Mesh mesh("../data/cat.off");

    igl::opengl::glfw::Viewer viewer;

    Mesh mesh("../data/cat.off");
    Handle handle(mesh);

    //EventHandler::EventHandler(mesh, handle);
    EventHandler::mesh = &mesh;
    EventHandler::handle = &handle;

    // Register the callbacks
    viewer.callback_mouse_down = &EventHandler::mouse_down;

    // Attach a menu plugin
    igl::opengl::glfw::imgui::ImGuiMenu menu;
    viewer.plugins.push_back(&menu);

    menu.callback_draw_viewer_menu = [&]() {
        menu.draw_viewer_menu();

        // Draw additional windows
        menu.callback_draw_custom_window = [&]()
        {
            // Define next window position + size
            ImGui::SetNextWindowPos(ImVec2(180.f * menu.menu_scaling(), 10), ImGuiSetCond_FirstUseEver);
            ImGui::SetNextWindowSize(ImVec2(250, 500), ImGuiSetCond_FirstUseEver);
            ImGui::Begin(
                    "Reconstruction Options", nullptr,
                    ImGuiWindowFlags_NoSavedSettings
            );

            enum DataOption { Cat = 0, Bunny, Luigi,};
            static DataOption inputData = Cat;
            ImGui::Combo("InputData", (int *)(&inputData), "Cat\0Bunny\0Luigi\0\0");

//            enum resolutionOption { small = 20, medium = 25, large =30};
//            static resolutionOption resolutionData = small;
//            ImGui::Combo("resolution", (int *)(&resolutionData), "small\0 medium\0 large\0\0");

//            enum polyDegreeOption { Zero = 0, One, Two};
//            static polyDegreeOption polyDegreeData = Zero;
//            ImGui::Combo("polyDegree", (int *)(&polyDegreeData), "Zero\0One\0Two\0\0");

//            enum neighborLayerOption { two = 2, three, four};
//            static neighborLayerOption neighborLayerData = two;
//            ImGui::Combo("neighborLayer", (int *)(&neighborLayerData), "two\0three\0four\0\0");


            if (ImGui::Button("Set PointsData", ImVec2(-1,0))) {
                // Reload data
                switch (inputData)
                {
                    case Cat:
                        mesh.reloadPointsData("../data/cat.off");
                        break;
                    case Bunny:
                        mesh.reloadPointsData("../data/bunny-500.off");
                        break;
                    case Luigi:
                        mesh.reloadPointsData("../data/luigi.off");
                        break;
                    default:
                        mesh.reloadPointsData("../data/cat.off");
                        break;
                }

//                switch (resolutionData)
//                {
//                    case small:
//                        Construction::resolution = 20;
//                        break;
//                    case medium:
//                        Construction::resolution = 25;
//                        break;
//                    case large:
//                        Construction::resolution = 30;
//                        break;
//                    default:
//                        Construction::resolution = 20;
//                        break;
//                }
//
//                switch (polyDegreeData)
//                {
//                    case Zero:
//                        mesh.setPolyDegree(0);
//                        break;
//                    case One:
//                        mesh.setPolyDegree(1);
//                        break;
//                    case Two:
//                        mesh.setPolyDegree(2);
//                        break;
//                    default:
//                        mesh.setPolyDegree(0);
//                        break;
//                }
//
//                switch (neighborLayerData)
//                {
//                    case two:
//                        Construction::layer = 2;
//                        break;
//                    case three:
//                        Construction::layer = 3;
//                        break;
//                    case four:
//                        Construction::layer = 4;
//                        break;
//                    default:
//                        Construction::layer = 2;
//                        break;
//                }

                EventHandler::constructingMesh(viewer, ShowPoints);
//
//                callback_key_down(viewer, '1', 0);

            }

            if (ImGui::Button("Compute Constrained_points", ImVec2(-1,0))) {
                // Switch view to show the grid
                EventHandler::constructingMesh(viewer, ShowConstrainedPoints);
//                callback_key_down(viewer, '2', 0);
            }
            if (ImGui::Button("Create Grids", ImVec2(-1,0))) {
                // Switch view to show the grid
                EventHandler::constructingMesh(viewer, CreateGrids);
//                callback_key_down(viewer, '3', 0);
            }
//            if (ImGui::Button("Show AlignDataGrids", ImVec2(-1,0))) {
//                // Switch view to show the grid
//                callback_key_down(viewer, '5', 0);
//            }

            if (ImGui::Button("Construct Mesh", ImVec2(-1,0))) {
                // Switch view to show the grid
//                callback_key_down(viewer, '4', 0);
                EventHandler::constructingMesh(viewer, ConstructMesh);
            }

            if (ImGui::Button("Laplacian Smoothing", ImVec2(-1,0))) {
                // Switch view to show the grid
//                callback_key_down(viewer, '5', 0);
                EventHandler::smoothingMesh(viewer, Laplacian);
            }

            if (ImGui::Button("Bilateral Denoising", ImVec2(-1,0))) {
                // Switch view to show the grid
                //callback_key_down(viewer, '6', 0);
                EventHandler::smoothingMesh(viewer, BilateralDenoising);
            }

            if (ImGui::Button("Iteration Denoising", ImVec2(-1,0))) {
                // Switch view to show the grid
                //callback_key_down(viewer, '7', 0);
                EventHandler::smoothingMesh(viewer, IterationDenoising);
            }

            if (ImGui::Button("Back to Last", ImVec2(-1,0))) {
                // Switch view to show the grid
                //callback_key_down(viewer, '8', 0);
                EventHandler::smoothingMesh(viewer, BackToLast);
            }

            if (ImGui::Button("Reset Mesh", ImVec2(-1,0))) {
                // Switch view to show the grid
                //callback_key_down(viewer, '9', 0);
                EventHandler::smoothingMesh(viewer, ResetMesh);
            }


//            static bool boolVariable = false;
//            if (ImGui::Checkbox("Deformation", &boolVariable))
//            {
//                // Register the callbacks
//                viewer.callback_mouse_down = &EventHandler::callback_mouse_down;
//                viewer.callback_mouse_move = &EventHandler::callback_mouse_move;
//                viewer.callback_mouse_up = &EventHandler::callback_mouse_up;
//                viewer.callback_pre_draw = &EventHandler::callback_pre_draw;
//                viewer.callback_key_down = &EventHandler::callback_key_down;
//            }
//            else
//            {
//                viewer.callback_mouse_down = &EventHandler::mouse_down;
//                viewer.callback_mouse_move = &EventHandler::mouse_move;
//                viewer.callback_mouse_up = &EventHandler::mouse_up;
//                viewer.callback_pre_draw = &EventHandler::pre_draw;
//                viewer.callback_key_down = &EventHandler::key_down;
//            }

            ImGui::End();
        };



    };

    viewer.launch();
}
