#include "aSolver.h"

#include <cmath>
#include <Eigen\Dense>
#include <Eigen\Core>


void ASolver::solve(State& newState)
{
	// Energy weights
	double wm = 100, wd = 1.0, wb = 1.0;
	
	// Calculate segments to optimize for
	std::vector<CurveSegment> C;
	// In this case since we are working with curve
	for (CurveSegment* curveSEgment : newState.orderedAffectedCurveSegments)
	{
		C.push_back(*curveSEgment);
	}
	// Total number of variables to optimize for (for each curve segment there is two tangents with x and y components)
	int numberOfVariables = C.size() * 4;
	// Matrix Q
	Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>  Q = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Zero(numberOfVariables, numberOfVariables);
	// One Matrix 3 by 4 because of x and y coordinate for left and right tangent of curve segments that represent a rotation R or translation T 
	// Each curve segment has three dimensions i.e. (Rx, Ry, Rz)
	// We have one matrix Js for each of the affected curve segments of the state.
	// If there are no other pints or constraints this would be all of the parents in the chain.
	// In particular in this case we only have one curve segment to work with at time, and we are considering the curve to be Cx
	Eigen::Matrix<double, 3, Eigen::Dynamic>  Js = Eigen::Matrix<double, 3, Eigen::Dynamic>::Zero(3, numberOfVariables);
	for (int i = 0; i < C.size(); i++)
	{
		// Compute derivative of state s with respect to curve c(theta, t)
		Eigen::Vector3d dSdC = Eigen::Vector3d::Zero();
		if (C[i].type == TRANSLATION)
		{
			dSdC(C[i].component) = 1;
		}
		else if (C[i].type == ROTATION)
		{
			// This case is not obtainable with curve viewer.
		}
		// Compute derivate of curve with respect to tangent using finite diferences
		Js.col(i * 4) = dSdC * C[i].dCdT(newState.frameNumber, 0, 0);
		Js.col(i * 4 + 1) = dSdC * C[i].dCdT(newState.frameNumber, 1, 0);
		Js.col(i * 4 + 2) = dSdC * C[i].dCdT(newState.frameNumber, 0, 1);
		Js.col(i * 4 + 3) = dSdC * C[i].dCdT(newState.frameNumber, 1, 1);
	}
	Q += Js.transpose() * Js * wm;
	// We add stiffness matrix with stiffnes of one
	Q += Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>::Identity(numberOfVariables, numberOfVariables) * wd;
	// We multiply by two since we divided by two before
	Q *= 2;
	// Linear term of the optimizaiton
	Eigen::VectorXd b = Eigen::VectorXd::Zero(numberOfVariables);



}

double CurveSegment::dCdT(int frameNumber, int component, int index)
{
	// delta tangents
	double dT = 0.0000001;
	double u = (frameNumber * FPS - keyLeft->t) / (keyRight->t - keyLeft->t);
	// This will be evaluated using Maya functions.
	vec3 tangentPlus = vec3(keyLeft->tangentPlus.x, keyLeft->tangentPlus.y, 0);
	vec3 tangentMinus = vec3(keyRight->tangentMinus.x, keyLeft->tangentMinus.y, 0);
	vec3 b0(keyLeft->value, keyLeft->t, 0);
	vec3 b3(keyRight->value, keyRight->t, 0);
	if (index == 0)
	{
		tangentPlus[component] += dT;
	}
	else
	{
		tangentMinus[component] += dT;
	}
	// Compute middle control points with modified tangents
	vec3 b1 = (1 / 3.0) * tangentPlus + b0;
	vec3 b2 = -(1 / 3.0) * tangentMinus + b3;
	// Evaluate curve value one
	vec3 evalSecond = b0 * std::pow(1 - u, 3) + b1 * 3 * u * std::pow(1 - u, 2) + b2 * 3 * std::pow(u, 2) * (1 - u) + b3 * std::pow(u, 3);
	if (index == 0)
	{
		tangentPlus[component] -= 2 * dT;
	}
	else
	{
		tangentMinus[component] -= 2 * dT;
	}
	// ReCompute middle control points with modified tangents
	b1 = (1 / 3.0) * tangentPlus + b0;
	b2 = -(1 / 3.0) * tangentMinus + b3;
	// Evalue curve value two
	vec3 evalFirst = b0 * std::pow(1 - u, 3) + b1 * 3 * u * std::pow(1 - u, 2) + b2 * 3 * std::pow(u, 2) * (1 - u) + b3 * std::pow(u, 3);
	return (evalSecond[1] - evalFirst[1]) / (2 * dT);
}

std::string ASolver::getKey(int type, int component, int frameNumber)
{
	return std::to_string(type) + "-" + std::to_string(component) + "-" + std::to_string(frameNumber);
}

