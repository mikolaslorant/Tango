#include "aSolver.h"

void ASolver::solve(const State& newState)
{
	// Check curve segment
	std::unique_ptr<CurveSegment> curveSegment = std::make_unique<CurveSegment>();
	// Assume translation
	curveSegment->type = TRANSLATION;
	curveSegment->id;

}