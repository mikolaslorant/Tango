#include "curveViewer.h"
//#include "mosek.h"

int main()
{
	CurveViewer curveViewer("Curve Editor");
	curveViewer.mainLoop();

	//MSKrescodee r, trmcode;
	//MSKenv_t env = NULL;
	//MSKtask_t task = NULL;
	//double deltaTheta = 0.0;
	//MSK_makeenv(&env, NULL); // Create environment
	//MSK_maketask(env, 0, 1, &task); // Create task
	//MSK_appendvars(task, 1); // 1 variable x
	//MSK_putcj(task, 0, 1.0); // c_0 = 1.0
	//MSK_putvarbound(task, 0, MSK_BK_RA, 2.0, 3.0); // 2.0 <= x <= 3.0
	//MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MAXIMIZE); // Minimize
	//MSK_optimizetrm(task, &trmcode); // Optimize
	//MSK_getxx(task, MSK_SOL_ITR, &deltaTheta); // Get solution
	//printf("Solution x = %f\n", deltaTheta); // Print solution
	//MSK_deletetask(&task); // Clean up task
	//MSK_deleteenv(&env); // Clean up environment
	return 0;

}