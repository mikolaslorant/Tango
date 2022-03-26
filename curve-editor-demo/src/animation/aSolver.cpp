#include "aSolver.h"

#include <cmath>
#include <Eigen\Dense>
#include <Eigen\Core>


void ASolver::solve(const State& newState)
{

	// compute Q matrix
	// One Matrix 3 by 4 because of x and y coordinate for left and right tangent of curve segments that represent a rotation R or translation T 
	// Each curve segment has three dimensions i.e. (Rx, Ry, Rz)
	// We have one matrix Js for each of the affected curve segments of the state.
	// If there are no other pints or constraints this would be all of the parents in the chain.
	// In particular in this case we only have one curve segment to work with at time.
	Eigen::Matrix<double, 3, Eigen::Dynamic>  Js = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, 4 * newState.orderedAffectedCurveSegments.size());
	for (const CurveSegment* curveSegment : newState.orderedAffectedCurveSegments)
	{
		// Compute derivative of state s with respect to curve c(theta, t)
		Eigen::Vector3d dSdC = Eigen::Vector3d::Zero();
		if (curveSegment->type == TRANSLATION)
		{

		}
		else if (curveSegment->type)

	}
	Js.col()


}

