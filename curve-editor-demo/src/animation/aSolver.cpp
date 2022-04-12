#include "aSolver.h"
#include <string>
#include <cmath>
#include <unordered_set>
#include <Eigen\Dense>
#include <Eigen\Core>


ASolver::ASolver() : r(MSK_makeenv(&env, NULL)), curveSegments(), keyFrames(), pins(), contacs()
{

}

ASolver::~ASolver()
{
	MSK_deleteenv(&env);
}

double ASolver::phi(double ui, const KeyFrame& currentKeyFrame, const KeyFrame& otherKeyFrame) inline const
{
	return (4.0 / 3.0) * (ui - std::min(currentKeyFrame.value, otherKeyFrame.value));
}

double ASolver::psi(double vi, const KeyFrame& currentKeyFrame, const KeyFrame& otherKeyFrame) inline const
{
	return (4.0 / 3.0) * (vi - std::max(currentKeyFrame.value, otherKeyFrame.value));
}

void ASolver::solve(State& newState, int totalNumberOfKeys)
{
	// Calculate ro vector and C vectors
	std::vector<CurveSegment*> C;
	std::unordered_set<State*> ro;
	for (const auto& pin : pins)
	{
		ro.insert(pin.second.get());
	}
	calculateCurveSegmentsThatNeedOptimizing(newState, ro, C);
	int iters = 0;
	// Total number of variables to optimize for (for each curve segment there is two tangents with x and y components)
	int numberOfVariables = C.size() * 4;
	// All of the delta thetas summed together between iterations
	Eigen::VectorXd solutionDeltaTangentsAccumulated = Eigen::VectorXd::Zero(numberOfVariables);
	while (iters < MAX_ITERS_TO_SOLVE)
	{
		
		Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(numberOfVariables, numberOfVariables);
		Eigen::VectorXd b = Eigen::VectorXd::Zero(numberOfVariables);
		Eigen::VectorXd lowerBounds = Eigen::VectorXd::Zero(numberOfVariables);
		Eigen::VectorXd upperBounds = Eigen::VectorXd::Zero(numberOfVariables);
		Eigen::VectorXd constraintsBounds = Eigen::VectorXd::Zero(ro.size() * 3);
		std::vector<Eigen::Matrix3Xd> A;
		calculateSolverInputs(newState, ro, C, totalNumberOfKeys, Q, b, lowerBounds, upperBounds, A, constraintsBounds);
		Eigen::VectorXd solutionDeltaTangents = Eigen::VectorXd::Zero(numberOfVariables);
		mosekSolve(Q, b, lowerBounds, upperBounds, A, constraintsBounds, solutionDeltaTangents);
		solutionDeltaTangentsAccumulated += solutionDeltaTangents;
		updateTangents(C, solutionDeltaTangents);
		// Check if desired state has been reached
		vec3 currrr = newState.getCurrentValue();
		double dif = (currrr - newState.point).Length();
		if (dif < MAX_ACCEPTED_DIFFERENCE_K)
		{
			break;
		}
		iters++;
	}
	// Reset tangent to original value if point is not reachable.
	if (iters == MAX_ITERS_TO_SOLVE)
	{
		Eigen::VectorXd solutionDeltaTangents = Eigen::VectorXd::Zero(numberOfVariables);
		solutionDeltaTangents -= solutionDeltaTangentsAccumulated;
		updateTangents(C, solutionDeltaTangents);
	}
}

void ASolver::calculateCurveSegmentsThatNeedOptimizing(const State& newState, 
														std::unordered_set<State*>& ro,
														std::vector<CurveSegment*>& C) const
{
	for (CurveSegment* curveSEgment : newState.orderedAffectedCurveSegments)
	{
		C.push_back(curveSEgment);
	}
	for (const auto& pin : pins)
	{
		// Check for case 1
		std::vector<CurveSegment*> intersectingSegmentsResult;
		std::set_intersection(newState.orderedAffectedCurveSegments.begin(), newState.orderedAffectedCurveSegments.end(),
			pin.second->orderedAffectedCurveSegments.begin(), pin.second->orderedAffectedCurveSegments.end(),
			std::back_inserter(intersectingSegmentsResult));
		if (intersectingSegmentsResult.size() == 0)
		{
			ro.erase(pin.second.get());
		}
		// Check for case 2
		else if (std::includes(C.begin(), C.end(),
			pin.second->orderedAffectedCurveSegments.begin(), pin.second->orderedAffectedCurveSegments.end()))
		{
			std::vector<CurveSegment*> result;
			ro.erase(pin.second.get());
			std::set_difference(C.begin(), C.end(),
				pin.second->orderedAffectedCurveSegments.begin(), pin.second->orderedAffectedCurveSegments.end(),
				std::back_inserter(result));
			C = result;
		}
		// Check case 3
		else
		{
			std::vector<CurveSegment*> result;
			std::set_union(C.begin(), C.end(),
				pin.second->orderedAffectedCurveSegments.begin(), pin.second->orderedAffectedCurveSegments.end(),
				std::back_inserter(result));
			C = result;
		}
	}
}

void ASolver::updateTangents(std::vector<CurveSegment*>& C, const Eigen::VectorXd& solutionDeltaTangents)
{
	for (int i = 0; i < C.size(); i++)
	{
		C[i]->keyLeft->tangentPlus.x += solutionDeltaTangents(i * 4);
		C[i]->keyLeft->tangentPlus.y += solutionDeltaTangents(i * 4 + 1);
		C[i]->keyRight->tangentMinus.x += solutionDeltaTangents(i * 4 + 2);
		C[i]->keyRight->tangentMinus.y += solutionDeltaTangents(i * 4 + 3);
	}
}

void ASolver::calculateSolverInputs(const State& newState, 
									const std::unordered_set<State*>& ro,
									const std::vector<CurveSegment*> &C, 
									int totalNumberOfKeys, 
									Eigen::MatrixXd& Q, 
									Eigen::VectorXd& b, 
									Eigen::VectorXd& lowerBounds, 
									Eigen::VectorXd& upperBounds,
									std::vector<Eigen::Matrix3Xd>& A,
									Eigen::VectorXd& constraintsBounds)
{
	// default hard limits for rotation and translation ui (lower bound), vi (upper bound).
	double ui[] = { -1000, -180 };
	double vi[] = { 1000, 180 };
	// Energy weights
	double wm = 100, wd = 1.0, wb = 0.0;
	int numberOfVariables = C.size() * 4;
	// One Matrix 3 by 4 because of x and y coordinate for left and right tangent of curve segments that represent a rotation R or translation T 
	// Each curve segment has three dimensions i.e. (Rx, Ry, Rz)
	// We have one matrix Js for each of the affected curve segments of the state.
	// If there are no other pints or constraints this would be all of the parents in the chain.
	// In particular in this case we only have one curve segment to work with at time, and we are considering the curve to be Cx
	Eigen::Matrix<double, 3, Eigen::Dynamic>  Js = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, numberOfVariables);
	// Calculate theta current minus the corresponding tangent on the other side of the keyframe.
	Eigen::VectorXd m = Eigen::VectorXd::Zero(numberOfVariables);
	Eigen::VectorXd mDelta = Eigen::VectorXd::Zero(numberOfVariables);
	Eigen::MatrixXd mDeltaMat = Eigen::MatrixXd::Identity(numberOfVariables, numberOfVariables);
	
	for (int i = 0; i < C.size(); i++)
	{
		// Compute derivative of state s with respect to curve c(theta, t)
		Eigen::Vector3d dSdC = Eigen::Vector3d::Zero();
		if (C[i]->type == TRANSLATION)
		{
			// For the curve viewer we only consider one joint
			dSdC(C[i]->component) = 1;
		}
		else if (C[i]->type == ROTATION)
		{
			// This case is not obtainable with curve viewer.
		}
		// Compute derivate of curve with respect to tangent using finite diferences
		Js.col(i * 4) = dSdC * C[i]->dCdT(newState.frameNumber, 0, 0);
		Js.col(i * 4 + 1) = dSdC * C[i]->dCdT(newState.frameNumber, 1, 0);
		Js.col(i * 4 + 2) = dSdC * C[i]->dCdT(newState.frameNumber, 0, 1);
		Js.col(i * 4 + 3) = dSdC * C[i]->dCdT(newState.frameNumber, 1, 1);
		// Compute m as the vector the current tangent + the tangent on the other side.
		// Temporary solution will make tangents at the edges of the curve stiffer.
		// To improve later on
		if (C[i]->keyLeft->keyFrameNumber == 0)
		{
			m(i * 4) = 0;
			m(i * 4 + 1) = 0;
			mDelta(i * 4) = 0;
			mDelta(i * 4 + 1) = 0;

		}
		else
		{
			m(i * 4) = C[i]->keyLeft->tangentMinus.x - C[i]->keyLeft->tangentPlus.x;
			m(i * 4 + 1) = C[i]->keyLeft->tangentMinus.y - C[i]->keyLeft->tangentPlus.y;
			mDelta(i * 4) = 1;
			mDelta(i * 4 + 1) = 1;
		}
		if (C[i]->keyRight->keyFrameNumber == totalNumberOfKeys - 1)
		{
			m(i * 4 + 2) = 0;
			m(i * 4 + 3) = 0;
			mDelta(i * 4 + 2) = 0;
			mDelta(i * 4 + 3) = 0;
		}
		else
		{
			m(i * 4 + 2) = - C[i]->keyRight->tangentMinus.x + C[i]->keyRight->tangentPlus.x;
			m(i * 4 + 3) = - C[i]->keyRight->tangentMinus.y + C[i]->keyRight->tangentPlus.y;
			mDelta(i * 4 + 2) = 1;
			mDelta(i * 4 + 3) = 1;
		}
		// Calculate bounds
		lowerBounds(i * 4) = 0;
		upperBounds(i * 4) = C[i]->keyRight->t - C[i]->keyLeft->t;
		lowerBounds(i * 4 + 1) = phi(ui[C[i]->type], *C[i]->keyLeft, *C[i]->keyRight);
		upperBounds(i * 4 + 1) = psi(vi[C[i]->type], *C[i]->keyLeft, *C[i]->keyRight);
		lowerBounds(i * 4 + 2) = 0;
		upperBounds(i * 4 + 2) = C[i]->keyRight->t - C[i]->keyLeft->t;
		lowerBounds(i * 4 + 3) = -psi(vi[C[i]->type], *C[i]->keyRight, *C[i]->keyLeft);
		upperBounds(i * 4 + 3) = -phi(ui[C[i]->type], *C[i]->keyRight, *C[i]->keyLeft);
	}
#if defined(DEBUG)
	std::cout << "JS MATRIX" << std::endl;
	std::cout << Js << std::endl;
#endif
	Q += Js.transpose() * Js * wm;
	// We add stiffness matrix with stiffnes of one
	Q += Eigen::MatrixXd::Identity(numberOfVariables, numberOfVariables) * wd;
	// We add the identity matrix for the energy term that tries to keep consecutive tangents the same.
	for (int i = 0; i < mDelta.size(); i++)
	{
		mDeltaMat(i,i) *= mDelta(i);
	}
	Q += mDeltaMat.transpose() * mDeltaMat * wb;
	// We multiply by two since we divided by two before
	// Add regularizing term in case matrix Q is not full rank
	Q += Eigen::MatrixXd::Identity(numberOfVariables, numberOfVariables) * REGULARIZATION_LAMBDA;
	vec3 currentValue = newState.getCurrentValue();
	vec3 ret = newState.point - currentValue;
	Eigen::Vector3d deltaS = Eigen::Vector3d::Zero(3);
	for (int i = 0; i < 3; i++)
	{
		deltaS(i) = ret[i];
	}
	b += (-2 * wm * deltaS.transpose() * Js).transpose();
	// Add m
	b += (-2 * wb * m);
#if defined(DEBUG)
	std::cout << "Q MATRIX" << std::endl;
	std::cout << Q << std::endl;
	std::cout << "b vector" << std::endl;
	std::cout << b << std::endl;
#endif
	// Calculate constraints
	int constraintBoundIdx = 0;
	for (const auto& ro : ro)
	{
		vec3 deltaRoPrime = ro->point - ro->getCurrentValue();
		constraintsBounds(constraintBoundIdx++) = deltaRoPrime[0];
		constraintsBounds(constraintBoundIdx++) = deltaRoPrime[1];
		constraintsBounds(constraintBoundIdx++) = deltaRoPrime[2];
		Eigen::MatrixXd Jro = Eigen::MatrixXd::Zero(3, numberOfVariables);
		for (int i = 0, j = 0; i < C.size() && j < ro->orderedAffectedCurveSegments.size() ; i++)
		{
			if (ro->orderedAffectedCurveSegments[j++] == C[i])
			{
				// Compute derivative of state s with respect to curve c(theta, t)
				Eigen::Vector3d dSdC = Eigen::Vector3d::Zero();
				if (C[i]->type == TRANSLATION)
				{
					// For the curve viewer we only consider one joint
					dSdC(C[i]->component) = 1;
				}
				else if (C[i]->type == ROTATION)
				{
					// This case is not obtainable with curve viewer.
				}
				// Compute derivate of curve with respect to tangent using finite diferences
				Jro.col(i * 4) = dSdC * C[i]->dCdT(newState.frameNumber, 0, 0);
				Jro.col(i * 4 + 1) = dSdC * C[i]->dCdT(newState.frameNumber, 1, 0);
				Jro.col(i * 4 + 2) = dSdC * C[i]->dCdT(newState.frameNumber, 0, 1);
				Jro.col(i * 4 + 3) = dSdC * C[i]->dCdT(newState.frameNumber, 1, 1);
			}
		}
		A.push_back(Jro);
	}

}

void ASolver::mosekSolve(const Eigen::MatrixXd& Q, const Eigen::VectorXd& b,
							const Eigen::VectorXd& lowerBounds, const Eigen::VectorXd& upperBounds, 
							const std::vector<Eigen::Matrix3Xd>& A,
							const Eigen::VectorXd& constraintsBounds,
							Eigen::VectorXd& solutionDeltaTangents)
{
	int numberOfVariables = Q.rows();
	MSKtask_t     task = NULL;
	/* Create the mosek environment. */

	MSKint32t j;
	// Q matrix vars
	std::vector<MSKint32t> qsubi;
	std::vector<MSKint32t> qsubj;
	std::vector<double> qval;
	// Matrix A information parsed
	std::vector<double> aval;
	std::vector<MSKint32t> aptrb, aptre;
	std::vector<MSKint32t> asub;
	parseConstraintMatrixA(A, aval, aptrb, aptre, asub);
	for (int row = 0; row < numberOfVariables; row++)
	{
		for (int col = 0; col <= row; col++)
		{
			if (Q(row, col) != 0)
			{
				qsubi.push_back(row);
				qsubj.push_back(col);
				if (row == col)
				{
					qval.push_back(2 * Q(row, col));
				}
				else
				{
					qval.push_back(Q(row, col));
				}
				
			}
			
		}
		
	}
	if (r == MSK_RES_OK)
	{
		/* Create the optimization task. */
		r = MSK_maketask(env, 0, numberOfVariables, &task);
		//r = MSK_maketask(env, constraintsBounds.size(), numberOfVariables, &task);
		if (r == MSK_RES_OK)
		{
#if defined(DEBUG)
			r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);
#endif
			/* Append 'NUMCON' empty constraints.
			 The constraints will initially have no bounds. */
			/*if (r == MSK_RES_OK)
				r = MSK_appendcons(task, constraintsBounds.size());*/
			/* Append 'NUMVAR' variables.
			The variables will initially be fixed at zero (x=0). */
			if (r == MSK_RES_OK)
				r = MSK_appendvars(task, numberOfVariables);
			/* Optionally add a constant term to the objective. */
			if (r == MSK_RES_OK)
				r = MSK_putcfix(task, 0.0);
			for (j = 0; j < numberOfVariables && r == MSK_RES_OK; ++j)
			{
				/* Set the linear term c_j in the objective.*/
				if (r == MSK_RES_OK)
					r = MSK_putcj(task, j, b(j));
				/* Set the bounds on variable j.
				 lowerBounds[j] <= x_j <= upperBounds[j] */
				if (r == MSK_RES_OK)
					r = MSK_putvarbound(task,
						j,           /* Index of variable.*/
						MSK_BK_RA,      /* Bound key. This value represents that the variable has a range. */
						lowerBounds(j),      /* Numerical value of lower bound.*/
						upperBounds(j));     /* Numerical value of upper bound.*/
				/* Input column j of A */
				// TODO
				//if (r == MSK_RES_OK)
				//	r = MSK_putacol(task,
				//		j,                 /* Variable (column) index.*/
				//		aptre[j] - aptrb[j], /* Number of non-zeros in column j.*/
				//		asub.data() + aptrb[j],   /* Pointer to row indexes of column j.*/
				//		aval.data() + aptrb[j]);  /* Pointer to Values of column j.*/
			}
			/* Set the bounds on constraints.
			for i=1, ...,NUMCON : blc[i] <= constraint i <= buc[i] */
			//for (int i = 0; i < constraintsBounds.size() && r == MSK_RES_OK; ++i)
			//	r = MSK_putconbound(task,
			//		i,           /* Index of constraint.*/
			//		MSK_BK_FX,      /* Bound key. In this case it's a fixed value*/
			//		constraintsBounds(i),      /* Numerical value of lower bound.*/
			//		constraintsBounds(i));     /* Numerical value of upper bound.*/
			if (r == MSK_RES_OK)
			{
				/*
				 * The lower triangular part of the Q^o
				 * matrix in the objective is specified.
				 * Input the Q^o for the objective. */

				r = MSK_putqobj(task, qval.size(), qsubi.data(), qsubj.data(), qval.data());
			}
			/*
			if (r == MSK_RES_OK)
				r = MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MINIMIZE);*/
			if (r == MSK_RES_OK)
			{
				MSKrescodee trmcode;

				/* Run optimizer */
				r = MSK_optimizetrm(task, &trmcode);

				/* Print a summary containing information
				   about the solution for debugging purposes*/
#if defined(DEBUG)
				MSK_solutionsummary(task, MSK_STREAM_LOG);
#endif
				if (r == MSK_RES_OK)
				{
					MSKsolstae solsta;
					int j;

					MSK_getsolsta(task, MSK_SOL_ITR, &solsta);

					switch (solsta)
					{
					case MSK_SOL_STA_OPTIMAL:
						MSK_getxx(task,
							MSK_SOL_ITR,    /* Request the interior solution. */
							solutionDeltaTangents.data());
#if defined(DEBUG)
						printf("Optimal primal solution\n");
						for (j = 0; j < numberOfVariables; ++j)
							printf("x[%d]: %e\n", j, solutionDeltaTangents[j]);
#endif
						break;

					case MSK_SOL_STA_DUAL_INFEAS_CER:
					case MSK_SOL_STA_PRIM_INFEAS_CER:
						printf("Primal or dual infeasibility certificate found.\n");
						break;

					case MSK_SOL_STA_UNKNOWN:
						printf("The status of the solution could not be determined. Termination code: %d.\n", trmcode);
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
		}
		MSK_deletetask(&task);
	}
}

void ASolver::parseConstraintMatrixA(const std::vector<Eigen::Matrix3Xd>&A,
							std::vector<double>& aval,
							std::vector<MSKint32t>& aptrb, std::vector<MSKint32t>& aptre, std::vector<MSKint32t>& asub) const
{
	int numCols = aptrb.size();
	for (int j = 0; j < numCols; j++)
	{
		bool firstFound = false;
		for (int i = 0; i < A.size(); i++)
		{
			for (int k = 0; k < 3; k++)
			{
				if (A[i](k, j) != 0)
				{
					aval.push_back(A[i](k, j));
					asub.push_back(i * 3 + k);
					if (!firstFound)
					{
						if (aptrb.size() > j)
						{

						}
						aptrb[j] = asub.size() - 1;
						firstFound = true;
					}
					aptre[j] = asub.size() - 1;
				}
			}
			
		}
	}
	
}


vec3 State::getCurrentValue() inline const
{
	vec3 ret;
	for (int i = 0; i < 2; i++)
	{	
		vec3 tangentPlus = vec3(orderedAffectedCurveSegments[i]->keyLeft->tangentPlus.x, orderedAffectedCurveSegments[i]->keyLeft->tangentPlus.y, 0);
		vec3 tangentMinus = vec3(orderedAffectedCurveSegments[i]->keyRight->tangentMinus.x, orderedAffectedCurveSegments[i]->keyRight->tangentMinus.y, 0);
		ret[i] = orderedAffectedCurveSegments[i]->evaluateBezierGivenTangents(frameNumber, tangentPlus, tangentMinus);
	}
	return ret;
}

double CurveSegment::evaluateBezierGivenTangents(int frameNumber, const vec3& tangentPlus, const vec3& tangentMinus) inline const
{
	// u = (t - tk) / (tk+1 - tk)
	double x = ((double)frameNumber) / FPS;
	vec3 b0(keyLeft->t, keyLeft->value, 0);
	vec3 b3(keyRight->t, keyRight->value, 0);
	vec3 b1 = (1 / 3.0) * tangentPlus + b0; // Tangent Plus(thetax + Dt, thetay, 0) => b1 = (x,y,z) => b1 = (x',y,z)
	vec3 b2 = -(1 / 3.0) * tangentMinus + b3; 
	double u = getTGivenX(x, b0[0], b1[0], b2[0], b3[0]);
	vec3 ret = b0 * std::pow(1 - u, 3) + b1 * 3 * u * std::pow(1 - u, 2) + b2 * 3 * std::pow(u, 2) * (1 - u) + b3 * std::pow(u, 3);
	return ret[1];
}


double CurveSegment::dCdT(int frameNumber, int component, int index) const
{
	// delta tangents
	double dT = 0.000001; //h
	// This will be evaluated using Maya functions.
	vec3 tangentPlus = vec3(keyLeft->tangentPlus.x, keyLeft->tangentPlus.y, 0);
	vec3 tangentMinus = vec3(keyRight->tangentMinus.x, keyRight->tangentMinus.y, 0);
	if (index == 0)
	{
		// deltaT = deltaT + h = t 
		tangentPlus[component] += dT;
	}
	else
	{
		tangentMinus[component] += dT;
	}
 	double evalSecond = evaluateBezierGivenTangents(frameNumber, tangentPlus, tangentMinus);
	if (index == 0)
	{
		tangentPlus[component] -= 2 * dT;
	}
	else
	{
		tangentMinus[component] -= 2 * dT;
	}
	double evalFirst = evaluateBezierGivenTangents(frameNumber, tangentPlus, tangentMinus);
	return (evalSecond - evalFirst) / (2 * dT);
}

std::string ASolver::getKey(int type, int component, int frameNumber)
{
	return std::to_string(type) + "-" + std::to_string(component) + "-" + std::to_string(frameNumber);
}

// Method taken from stackoverflow post
// https://stackoverflow.com/questions/51879836/cubic-bezier-curves-get-y-for-given-x-special-case-where-x-of-control-points

bool approximately(double a, double b) 
{
	return std::abs(a - b) < 0.000001;
}

double crt(double x)
{
	return x < 0 ? -std::pow(-x, 1.0 / 3.0) : std::pow(x, 1.0 / 3.0);
}

void findRoots(std::vector<double>& roots, double x, double pa, double pb, double pc, double pd)
{
	double   a = (3 * pa - 6 * pb + 3 * pc),
		b = (-3 * pa + 3 * pb),
		c = pa,
		d = (-pa + 3 * pb - 3 * pc + pd);

	// do a check to see whether we even need cubic solving:
	if (approximately(d, 0)) {
		// this is not a cubic curve.
		if (approximately(a, 0)) {
			// in fact, this is not a quadratic curve either.
			if (approximately(b, 0)) {
				// in fact in fact, there are no solutions.
				return;
			}
			// linear solution
			roots.push_back(-c / b);
			return;
		}
		// quadratic solution
		double q = sqrt(b * b - 4 * a * c), dsa = 2 * a;
		roots = { (q - b) / dsa, (-b - q) / dsa };
		return;
	}

	// at this point, we know we need a cubic solution.

	a /= d;
	b /= d;
	c /= d;

	double p = (3 * b - a * a) / 3,
		p3 = p / 3,
		q = (2 * a * a * a - 9 * a * b + 27 * c) / 27,
		q2 = q / 2,
		discriminant = q2 * q2 + p3 * p3 * p3;

	// and some variables we're going to use later on:
	double u1, v1, root1, root2, root3;

	// three possible real roots:
	if (discriminant < 0) {
		double mp3 = -p / 3,
			mp33 = mp3 * mp3 * mp3,
			r = sqrt(mp33),
			t = -q / (2 * r),
			cosphi = t < -1 ? -1 : t>1 ? 1 : t,
			phi = acos(cosphi),
			crtr = crt(r),
			t1 = 2 * crtr;
		root1 = t1 * cos(phi / 3) - a / 3;
		root2 = t1 * cos((phi + 2 * M_PI) / 3) - a / 3;
		root3 = t1 * cos((phi + 4 * M_PI) / 3) - a / 3;
		roots = { root1, root2, root3 };
		return;
	}

	// three real roots, but two of them are equal:
	if (discriminant == 0) {
		u1 = q2 < 0 ? crt(-q2) : -crt(q2);
		root1 = 2 * u1 - a / 3;
		root2 = -u1 - a / 3;
		roots = { root1, root2 };
		return;
	}

	// one real root, two complex roots
	double sd = sqrt(discriminant);
	u1 = crt(sd - q2);
	v1 = crt(sd + q2);
	root1 = u1 - v1 - a / 3;
	roots = {root1};
	return;
}

double CurveSegment::getTGivenX(double x, double pa, double pb, double pc, double pd) const 
{
	int size = 0;
	std::vector<double> roots;
	findRoots(roots, x, pa - x, pb - x, pc - x, pd - x);
	double t;
	if (roots.size() > 0) {
		for (double _t : roots) {
			if (_t < 0 || _t > 1) continue;
			t = _t;
			break;
		}
	}
	return t;
}