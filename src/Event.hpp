//
// Created by Yulin Liu on 8/6/19.
//

#ifndef EX2_EVENT_HPP
#define EX2_EVENT_HPP

#include <igl/opengl/glfw/Viewer.h>
#include "Mesh.hpp"
#include "Handle.hpp"
#include "Construction.hpp"
#include "Smoothing.hpp"
#include "Deformation.hpp"


enum ConstructionEvent {
    None, ShowPoints, ShowConstrainedPoints, CreateGrids, ConstructMesh
};

enum SmoothingEvent {
    Laplacian = 5, BilateralDenoising, IterationDenoising, BackToLast, ResetMesh
};


class EventHandler
{
public:
//    static Mesh& mesh;
//    static Handle& handle;

    static Mesh* mesh;
    static Handle* handle;

    static bool doit;
    static int down_mouse_x, down_mouse_y;

public:
    //EventHandler(Mesh& mesh, Handle& handle);
    //EventHandler(Mesh& mesh, Handle& handle);

    /* For mesh reconstruction */
    static bool constructingMesh(igl::opengl::glfw::Viewer& viewer, ConstructionEvent e);

    /* For mesh smoothing */
    static bool smoothingMesh(igl::opengl::glfw::Viewer& viewer, SmoothingEvent e);
    static bool mouse_down(igl::opengl::glfw::Viewer& viewer, int, int);
    static bool mouse_move(igl::opengl::glfw::Viewer& viewer, int mouse_x, int mouse_y);
    static bool mouse_up(igl::opengl::glfw::Viewer& viewer, int button, int modifier);
    static bool pre_draw(igl::opengl::glfw::Viewer& viewer);
    static bool key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifiers);

    /* For shape deformation */
    static bool callback_mouse_down(igl::opengl::glfw::Viewer& viewer, int button, int modifier);
    static bool callback_mouse_move(igl::opengl::glfw::Viewer& viewer, int mouse_x, int mouse_y);
    static bool callback_mouse_up(igl::opengl::glfw::Viewer& viewer, int button, int modifier);
    static bool callback_pre_draw(igl::opengl::glfw::Viewer& viewer);
    static bool callback_key_down(igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifiers);

};







#endif //EX2_EVENT_HPP
