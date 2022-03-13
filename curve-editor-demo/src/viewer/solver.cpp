#include "solver.h"

int Solver::getNumVar() {
	return numVar;
}

void Solver::setSolverVariables(Eigen::MatrixXd Q, std::vector<int> deltaTangents) {
	this->numVar = deltaTangents.size();
	this->numQNZ = 0;

	//qsubi[0] = 0; qsubj[0] = 0; qval[0] = 2.0;
	//qsubi[1] = 1; qsubj[1] = 1; qval[1] = 0.2;
	//qsubi[2] = 2; qsubj[2] = 0; qval[2] = -1.0;
	//qsubi[3] = 2; qsubj[3] = 2; qval[3] = 2.0;

	for (int itr_i = 0; itr_i < numVar; itr_i++) {
		for (int itr_j = 0; itr_j < itr_i; itr_j++) {
			if (Q(itr_i, itr_j) != 0.f) {
				qsubi[numQNZ] = itr_i;	qsubj[numQNZ] = itr_j;	qval[numQNZ] = Q(itr_i, itr_j);
				this->numQNZ++;
			}
		}
	}
}

//void setDeltaTangents(std::vector<int> &deltaTangentsIp) {
//	this->deltaTangents = &deltaTangentsIp;
//}

// solver variables
Solver::Solver() {

	// coefMatB
	coeffMatB = new double[numVar];

	// deltaTheta
	xx = new double[numVar];

	// Q
	qsubi = new MSKint32t[numQNZ];
	qsubj = new MSKint32t[numQNZ];
	qval = new double[numQNZ];

	i = 0;
	j = 0;
	env = NULL;
	task = NULL;
}

MSKrescodee Solver::runSolver() {
	r = MSK_makeenv(&env, NULL);	// Create Mosek environment

	if (r == MSK_RES_OK)
	{
		/* Create the optimization task. */
		r = MSK_maketask(env, NUMCON, numVar, &task);
		if (r == MSK_RES_OK)
		{
			r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);
			/* Append 'NUMCON' empty constraints.
			The constraints will initially have no bounds. */
			if (r == MSK_RES_OK)
				r = MSK_appendcons(task, NUMCON);
			/* Append 'NUMVAR' variables.
			The variables will initially be fixed at zero (x=0). */
			if (r == MSK_RES_OK)
				r = MSK_appendvars(task, numVar);
			/* Optionally add a constant term to the objective. */
			if (r == MSK_RES_OK)
				r = MSK_putcfix(task, 0.0);
			for (j = 0; j < numVar && r == MSK_RES_OK; ++j)
			{
				/* Set the linear term c_j in the objective.*/
				if (r == MSK_RES_OK)
					r = MSK_putcj(task, j, coeffMatB[j]);
				/* Set the bounds on variable j.
				blx[j] <= x_j <= bux[j] */
				if (r == MSK_RES_OK)
					r = MSK_putvarbound(task,
						j, /* Index of variable.*/
						bkx[j], /* Bound key.*/
						blx[j], /* Numerical value of lower bound.*/
						bux[j]); /* Numerical value of upper bound.*/
						/* Input column j of A */
				if (r == MSK_RES_OK)
					r = MSK_putacol(task,
						j, /* Variable (column) index.*/
						aptre[j] - aptrb[j], /* Number of non-zeros in column j.*/
						asub + aptrb[j], /* Pointer to row indexes of column j.*/
						aval + aptrb[j]); /* Pointer to Values of column j.*/
			}
			/* Set the bounds on constraints.
			for i=1, ...,NUMCON : blc[i] <= constraint i <= buc[i] */
			for (i = 0; i < NUMCON && r == MSK_RES_OK; ++i)
				r = MSK_putconbound(task,
					i, /* Index of constraint.*/
					bkc[i], /* Bound key.*/
					blc[i], /* Numerical value of lower bound.*/
					buc[i]); /* Numerical value of upper bound.*/
			if (r == MSK_RES_OK)
			{
				/*
				* The lower triangular part of the Q
				* matrix in the objective is specified.
				* Currently implemented in setSolverVariables() by external call
				* qsubi[0] = 0; qsubj[0] = 0; qval[0] = 2.0;
				* qsubi[1] = 1; qsubj[1] = 1; qval[1] = 0.2;
				* qsubi[2] = 2; qsubj[2] = 0; qval[2] = -1.0;
				* qsubi[3] = 2; qsubj[3] = 2; qval[3] = 2.0;
				* /

				/* Input the Q for the objective. */
				r = MSK_putqobj(task, numQNZ, qsubi, qsubj, qval);
			}
			if (r == MSK_RES_OK)
			{
				MSKrescodee trmcode;
				/* Run optimizer */
				r = MSK_optimizetrm(task, &trmcode);
				/* Print a summary containing information
				about the solution for debugging purposes*/
				MSK_solutionsummary(task, MSK_STREAM_MSG);
				if (r == MSK_RES_OK)
				{
					MSKsolstae solsta;
					int j;
					MSK_getsolsta(task, MSK_SOL_ITR, &solsta);
					switch (solsta)
					{
					case MSK_SOL_STA_OPTIMAL:
						MSK_getxx(task,
							MSK_SOL_ITR, /* Request the interior solution. */
							xx);
						printf("Optimal primal solution\n");
						for (j = 0; j < numVar; ++j)
							printf("x[%d]: %e\n", j, xx[j]);
						break;
					case MSK_SOL_STA_DUAL_INFEAS_CER:
					case MSK_SOL_STA_PRIM_INFEAS_CER:
						printf("Primal or dual infeasibility certificate found.\n");
						break;
					case MSK_SOL_STA_UNKNOWN:
						printf("The status of the solution could not be determined. Termination code: % d.\n", trmcode);
							break;
					default:
						printf("Other solution status.");
						break;
					}
				}
				else
				{
					printf("Error while optimizing.\n");
				}
			}
			if (r != MSK_RES_OK)
			{
				/* In case of an error print error code and description. */
				char symname[MSK_MAX_STR_LEN];
				char desc[MSK_MAX_STR_LEN];
				printf("An error occurred while optimizing.\n");
				MSK_getcodedesc(r,
					symname,
					desc);
				printf("Error %s - '%s'\n", symname, desc);
			}
			MSK_deletetask(&task);
		}
		MSK_deleteenv(&env);
		return r;
}