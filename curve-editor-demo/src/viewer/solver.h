#pragma once
#define IMGUI_IMPL_OPENGL_LOADER_GLAD

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <memory>

#include "camera.h"
#include "shader.h"
#include "drawable.h"
#include "aVector.h"
#include "mosek.h"
#include "Eigen/Dense"

# define NUMCON 1 /* Number of constraints. */
//# define NUMVAR 3 /* Number of variables. */
# define NUMANZ 3 /* Number of non-zeros in A. */
# define NUMQNZ 4 /* Number of non-zeros in Q. */

static void MSKAPI printstr(void* handle,
	const char str[])
{
	printf("%s", str);
} /* printstr */

class Solver
{
	// tango variables
	//std::vector<int> deltaTangents;
	//void setDeltaTangents(std::vector<int> deltaTangents);

	int numVar;
	Eigen::MatrixXd Q;

	MSKint32t i, j;
	MSKenv_t env = NULL;
	MSKtask_t task = NULL;
	MSKrescodee r;

	double* coeffMatB;
	MSKint32t* qsubi;
	MSKint32t* qsubj;
	double* qval;
	double* xx;

	MSKboundkeye bkb[1] = { MSK_BK_LO };
	double blb[1] = { 1.0 };
	double bub[1] = { +MSK_INFINITY };


public:
	Solver();
	int getNumVar();
	void setNumVar(int numVar);
	MSKrescodee runSolver();
};