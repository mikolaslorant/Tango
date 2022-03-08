#include "aSplineVec3.h"
#include <algorithm>
#include <cmath>
#include <Eigen\Dense>

#pragma warning(disable:4018)
#pragma warning(disable:4244)


ASplineVec3::ASplineVec3() : mInterpolator(new ABernsteinInterpolatorVec3())
{
}

ASplineVec3::~ASplineVec3()
{
	if (mInterpolator) delete mInterpolator;
}

void ASplineVec3::setFramerate(double fps)
{
	mInterpolator->setFramerate(fps);
}

double ASplineVec3::getFramerate() const
{
	return mInterpolator->getFramerate();
}

void ASplineVec3::setLooping(bool loop)
{
	mLooping = loop;
}

bool ASplineVec3::getLooping() const
{
	return mLooping;
}

void ASplineVec3::setInterpolationType(ASplineVec3::InterpolationType type)
{
	double fps = getFramerate();

	if (mInterpolator) { delete mInterpolator; }
	switch (type)
	{
	case LINEAR: mInterpolator = new ALinearInterpolatorVec3(); break;
	case CUBIC_BERNSTEIN: mInterpolator = new ABernsteinInterpolatorVec3(); break;
	case CUBIC_CASTELJAU: mInterpolator = new ACasteljauInterpolatorVec3(); break;
	case CUBIC_MATRIX: mInterpolator = new AMatrixInterpolatorVec3(); break;
	case CUBIC_HERMITE: mInterpolator = new AHermiteInterpolatorVec3(); break;
	case CUBIC_BSPLINE: mInterpolator = new ABSplineInterpolatorVec3(); break;
	case LINEAR_EULER: mInterpolator = new AEulerLinearInterpolatorVec3(); break;
	case CUBIC_EULER: mInterpolator = new AEulerCubicInterpolatorVec3(); break;
	};

	mInterpolator->setFramerate(fps);
	computeControlPoints();
	cacheCurve();
}

ASplineVec3::InterpolationType ASplineVec3::getInterpolationType() const
{
	return mInterpolator->getType();
}

void ASplineVec3::editKey(int keyID, const vec3& value)
{
	assert(keyID >= 0 && keyID < mKeys.size());
	mKeys[keyID].second = value;
	computeControlPoints();
	cacheCurve();
}

void ASplineVec3::editControlPoint(int ID, const vec3& value)
{
	assert(ID >= 0 && ID < mCtrlPoints.size() + 2);
	if (ID == 0)
	{
		mStartPoint = value;
		computeControlPoints(false);
	}
	else if (ID == mCtrlPoints.size() + 1)
	{
		mEndPoint = value;
		computeControlPoints(false);
	}
	else mCtrlPoints[ID - 1] = value;
	cacheCurve();
}

void ASplineVec3::appendKey(double time, const vec3& value, bool updateCurve)
{
	mKeys.push_back(Key(time, value));

	if (updateCurve)
	{
		computeControlPoints();
		cacheCurve();
	}
}

int ASplineVec3::insertKey(double time, const vec3& value, bool updateCurve)
{
	if (mKeys.size() == 0)
	{
		appendKey(time, value, updateCurve);
		return 0;
	}

	for (int i = 0; i < mKeys.size(); ++i)
	{
		assert(time != mKeys[i].first);
		if (time < mKeys[i].first)
		{
			mKeys.insert(mKeys.begin() + i, Key(time, value));
			if (updateCurve)
			{
				computeControlPoints();
				cacheCurve();
			}
			return i;
		}
	}

	// Append at the end of the curve
	appendKey(time, value, updateCurve);
	return mKeys.size() - 1;
}

void ASplineVec3::appendKey(const vec3& value, bool updateCurve)
{
	if (mKeys.size() == 0)
	{
		appendKey(0, value, updateCurve);
	}
	else
	{
		double lastT = mKeys[mKeys.size() - 1].first;
		appendKey(lastT + 1, value, updateCurve);
	}
}

void ASplineVec3::deleteKey(int keyID)
{
	assert(keyID >= 0 && keyID < mKeys.size());
	mKeys.erase(mKeys.begin() + keyID);
	computeControlPoints();
	cacheCurve();
}

vec3 ASplineVec3::getKey(int keyID) const
{
	assert(keyID >= 0 && keyID < mKeys.size());
	return mKeys[keyID].second;
}

int ASplineVec3::getNumKeys() const
{
	return mKeys.size();
}

vec3 ASplineVec3::getControlPoint(int ID) const
{
	assert(ID >= 0 && ID < mCtrlPoints.size() + 2);
	if (ID == 0) return mStartPoint;
	else if (ID == mCtrlPoints.size() + 1) return mEndPoint;
	else return mCtrlPoints[ID - 1];
}

int ASplineVec3::getNumControlPoints() const
{
	return mCtrlPoints.size() + 2; // include endpoints
}

void ASplineVec3::clear()
{
	mKeys.clear();
}

double ASplineVec3::getDuration() const
{
	return mKeys.size() == 0 ? 0 : mKeys[mKeys.size() - 1].first;
}

double ASplineVec3::getNormalizedTime(double t) const
{
	return (t / getDuration());
}

double ASplineVec3::getKeyTime(int keyID) const
{
	assert(keyID >= 0 && keyID < mKeys.size());
	return mKeys[keyID].first;
}

vec3 ASplineVec3::getValue(double t) const
{
	if (mCachedCurve.size() == 0 || mKeys.size() == 0) return vec3();
	if (t < mKeys[0].first)
		return mCachedCurve[0];
	else
		t -= mKeys[0].first;

	double dt = mInterpolator->getDeltaTime();
	int rawi = (int)(t / dt); // assumes uniform spacing
	double frac = (t - rawi * dt) / dt;

	int i = mLooping ? rawi % mCachedCurve.size() : std::min<int>(rawi, mCachedCurve.size() - 1);
	int inext = mLooping ? (i + 1) % mCachedCurve.size() : std::min<int>(i + 1, mCachedCurve.size() - 1);

	vec3 v1 = mCachedCurve[i];
	vec3 v2 = mCachedCurve[inext];
	vec3 v = v1 * (1 - frac) + v2 * frac;
	return v;
}

void ASplineVec3::cacheCurve()
{
	mInterpolator->interpolate(mKeys, mCtrlPoints, mCachedCurve);
}

void ASplineVec3::computeControlPoints(bool updateEndPoints)
{
	if (mKeys.size() >= 2 && updateEndPoints)
	{
		int totalPoints = mKeys.size();

		//If there are more than 1 interpolation point, set up the 2 end points to help determine the curve.
		//They lie on the tangent of the first and last interpolation points.
		vec3 tmp = mKeys[0].second - mKeys[1].second;
		double n = tmp.Length();
		mStartPoint = mKeys[0].second + (tmp / n) * n * 0.25; // distance to endpoint is 25% of distance between first 2 points

		tmp = mKeys[totalPoints - 1].second - mKeys[totalPoints - 2].second;
		n = tmp.Length();
		mEndPoint = mKeys[totalPoints - 1].second + (tmp / n) * n * 0.25;
	}
	mInterpolator->computeControlPoints(mKeys, mCtrlPoints, mStartPoint, mEndPoint);
}

vec3* ASplineVec3::getCachedCurveData()
{
	return mCachedCurve.data();
}

vec3* ASplineVec3::getControlPointsData()
{
	return mCtrlPoints.data();
}

int ASplineVec3::getNumCurveSegments() const
{
	return mCachedCurve.size();
}

vec3 ASplineVec3::getCurvePoint(int i) const
{
	return mCachedCurve[i];
}

//---------------------------------------------------------------------
AInterpolatorVec3::AInterpolatorVec3(ASplineVec3::InterpolationType t) : mDt(1.0 / 120.0), mType(t)
{
}

void AInterpolatorVec3::setFramerate(double fps)
{
	mDt = 1.0 / fps;
}

double AInterpolatorVec3::getFramerate() const
{
	return 1.0 / mDt;
}

double AInterpolatorVec3::getDeltaTime() const
{
	return mDt;
}

void AInterpolatorVec3::interpolate(const std::vector<ASplineVec3::Key>& keys,
	const std::vector<vec3>& ctrlPoints, std::vector<vec3>& curve)
{
	vec3 val = 0.0;
	double u = 0.0;

	curve.clear();

	int numSegments = keys.size() - 1;
	for (int segment = 0; segment < numSegments; segment++)
	{
		for (double t = keys[segment].first; t < keys[segment + 1].first - FLT_EPSILON; t += mDt)
		{
			// TODO: Compute u, fraction of duration between segment and segmentnext, for example,
			// u = 0.0 when t = keys[segment-1].first  
			// u = 1.0 when t = keys[segment].first
			u = (t - keys[segment].first) / (keys[segment + 1].first - keys[segment].first);
			val = interpolateSegment(keys, ctrlPoints, segment, u);
			curve.push_back(val);
		}
	}
	// add last point
	if (keys.size() > 1)
	{
		u = 1.0;
		val = interpolateSegment(keys, ctrlPoints, numSegments - 1, u);
		curve.push_back(val);
	}
}


// Interpolate p0 and p1 so that t = 0 returns p0 and t = 1 returns p1
vec3 ALinearInterpolatorVec3::interpolateSegment(
	const std::vector<ASplineVec3::Key>& keys,
	const std::vector<vec3>& ctrlPoints,
	int segment, double u)
{
	vec3 key0 = keys[segment].second;
	vec3 key1 = keys[segment + 1].second;
	// TODO: Linear interpolate between key0 and key1 so that u = 0 returns key0 and u = 1 returns key1
	return key0 * (1 - u) + key1 * u;
}

vec3 ABernsteinInterpolatorVec3::interpolateSegment(
	const std::vector<ASplineVec3::Key>& keys,
	const std::vector<vec3>& ctrlPoints,
	int segment, double u)
{
	vec3 b0 = ctrlPoints[segment * 4];
	vec3 b1 = ctrlPoints[segment * 4 + 1];
	vec3 b2 = ctrlPoints[segment * 4 + 2];
	vec3 b3 = ctrlPoints[segment * 4 + 3];
	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	// Step2: Compute the interpolated value f(u) point using  Bernstein polynomials
	return b0 * std::pow(1 - u, 3) + b1 * 3 * u * std::pow(1 - u, 2) + b2 * 3 * std::pow(u, 2) * (1 - u) + b3 * std::pow(u, 3);
}

vec3 ACasteljauInterpolatorVec3::interpolateSegment(
	const std::vector<ASplineVec3::Key>& keys,
	const std::vector<vec3>& ctrlPoints,
	int segment, double u)
{
	vec3 b0 = ctrlPoints[segment * 4];
	vec3 b1 = ctrlPoints[segment * 4 + 1];
	vec3 b2 = ctrlPoints[segment * 4 + 2];
	vec3 b3 = ctrlPoints[segment * 4 + 3];

	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	// Step2: Compute the interpolated value f(u) point using  deCsteljau alogithm
	return vec3::Lerp(vec3::Lerp(vec3::Lerp(b0, b1, u), vec3::Lerp(b1, b2, u), u),
		vec3::Lerp(vec3::Lerp(b1, b2, u), vec3::Lerp(b2, b3, u), u),
		u);
}

vec3 AMatrixInterpolatorVec3::interpolateSegment(
	const std::vector<ASplineVec3::Key>& keys,
	const std::vector<vec3>& ctrlPoints,
	int segment, double u)
{
	vec3 b0 = ctrlPoints[segment * 4];
	vec3 b1 = ctrlPoints[segment * 4 + 1];
	vec3 b2 = ctrlPoints[segment * 4 + 2];
	vec3 b3 = ctrlPoints[segment * 4 + 3];

	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	// Step2: Compute the interpolated value f(u) point using  matrix method f(u) = GMU
	// Hint: Using Eigen::MatrixXd data representations for a matrix operations
	Eigen::Matrix<double, 3, 4> GBezier;
	GBezier << b0[0], b1[0], b2[0], b3[0],
		b0[1], b1[1], b2[1], b3[1],
		b0[2], b1[2], b2[2], b3[2];
	Eigen::Matrix4d MBezier;
	MBezier << 1, -3, 3, -1,
		0, 3, -6, 3,
		0, 0, 3, -3,
		0, 0, 0, 1;
	Eigen::Matrix<double, 4, 1> UMatrix = { 1, u, std::pow(u, 2), std::pow(u, 3) };
	auto fU = GBezier * MBezier * UMatrix;
	return vec3(fU[0], fU[1], fU[2]);
}

vec3 AHermiteInterpolatorVec3::interpolateSegment(
	const std::vector<ASplineVec3::Key>& keys,
	const std::vector<vec3>& ctrlPoints,
	int segment, double u)
{
	vec3 p0 = keys[segment].second;
	vec3 p1 = keys[segment + 1].second;
	vec3 q0 = ctrlPoints[segment]; // slope at p0
	vec3 q1 = ctrlPoints[segment + 1]; // slope at p1

	return p0 * (2 * std::pow(u, 3) - 3 * std::pow(u, 2) + 1)
		+ p1 * (-2 * std::pow(u, 3) + 3 * std::pow(u, 2))
		+ q0 * (std::pow(u, 3) - 2 * std::pow(u, 2) + u)
		+ q1 * (std::pow(u, 3) - std::pow(u, 2));
}

vec3 ABSplineInterpolatorVec3::interpolateSegment(
	const std::vector<ASplineVec3::Key>& keys,
	const std::vector<vec3>& ctrlPoints,
	int segment, double u)
{
	double t = keys[segment].first;
	int j = 0;
	for (int i = 1; i < mKnots.size(); i++)
	{
		if (mKnots[i] > t)
		{
			j = i - 1;
			break;
		}
	}
	double B03 = (std::pow(1 - u, 3) / 6.);
	double B13 = (3 * std::pow(u, 3) - 6 * u * u + 4) / 6.;
	double B23 = (-3 * std::pow(u, 3) + 3 * u * u + 3 * u + 1) / 6.;
	double B33 = std::pow(u, 3) / 6.;
	return B03 * ctrlPoints[j - 3] + B13 * ctrlPoints[j - 2] + B23 * ctrlPoints[j - 1] + B33 * ctrlPoints[j];
}

void ACubicInterpolatorVec3::computeControlPoints(
	const std::vector<ASplineVec3::Key>& keys,
	std::vector<vec3>& ctrlPoints,
	vec3& startPoint, vec3& endPoint)
{
	ctrlPoints.clear();
	if (keys.size() <= 1) return;
	vec3 b0, b1, b2, b3;
	b0 = keys[0].second;
	b1 = keys[0].second + (1 / 3.) * 0.5 * (keys[1].second - startPoint);
	if (keys.size() == 2)
	{
		b2 = keys[1].second - (1 / 3.) * 0.5 * (endPoint - keys[0].second);
	}
	else
	{
		b2 = keys[1].second - (1 / 3.) * 0.5 * (keys[2].second - keys[0].second);
	}
	b3 = keys[1].second;
	ctrlPoints.push_back(b0);
	ctrlPoints.push_back(b1);
	ctrlPoints.push_back(b2);
	ctrlPoints.push_back(b3);
	for (int i = 1; i < keys.size() - 2; i++)
	{

		// TODO: compute b0, b1, b2, b3
		b0 = keys[i].second;
		b1 = b0 + (1 / 3.) * 0.5 * (keys[i + 1].second - keys[i - 1].second);
		b2 = keys[i + 1].second - (1 / 3.) * 0.5 * (keys[i + 2].second - keys[i].second);
		b3 = keys[i + 1].second;
		ctrlPoints.push_back(b0);
		ctrlPoints.push_back(b1);
		ctrlPoints.push_back(b2);
		ctrlPoints.push_back(b3);
	}
	if (keys.size() > 2)
	{
		b0 = keys[keys.size() - 2].second;
		b1 = keys[keys.size() - 2].second + (1 / 3.) * 0.5 * (keys[keys.size() - 1].second - keys[keys.size() - 3].second);
		b2 = keys[keys.size() - 1].second - (1 / 3.) * 0.5 * (endPoint - keys[keys.size() - 2].second);
		b3 = keys[keys.size() - 1].second;
		ctrlPoints.push_back(b0);
		ctrlPoints.push_back(b1);
		ctrlPoints.push_back(b2);
		ctrlPoints.push_back(b3);
	}
}

void AHermiteInterpolatorVec3::computeControlPoints(
	const std::vector<ASplineVec3::Key>& keys,
	std::vector<vec3>& ctrlPoints,
	vec3& startPoint, vec3& endPoint)
{
	// TODO: 
	// For each key point pi, compute the corresonding value of the slope pi_prime.
	// Hints: Using Eigen::MatrixXd for a matrix data structures, 
	// this can be accomplished by solving the system of equations AC=D for C.
	// Don't forget to save the values computed for C in ctrlPoints
	// For clamped endpoint conditions, set 1st derivative at first and last points (p0 and pm) to s0 and s1, respectively
	// For natural endpoints, set 2nd derivative at first and last points (p0 and pm) equal to 0

	// Step 1: Initialize A
	// Step 2: Initialize D
	// Step 3: Solve AC=D for C
	// Step 4: Save control points in ctrlPoints

	// Control Points: [p0_prime, p1_prime, p2_prime, ..., pm_prime]
	// The size of control points should be the same as the size of the keys
	// Use operator[] to set elements in ctrlPoints by indices

	// Hint: Do not use push_back() to insert control points here because the vector has been resized
	ctrlPoints.clear();
	ctrlPoints.resize(keys.size(), vec3(0, 0, 0));
	if (keys.size() <= 1) return;
	Eigen::MatrixXd A(keys.size(), keys.size());
	Eigen::MatrixXd D(keys.size(), 3);
	// Initialize A
	A(0, 0) = 2;
	A(0, 1) = 1;
	A(keys.size() - 1, keys.size() - 1) = 2;
	A(keys.size() - 1, keys.size() - 2) = 1;
	for (int i = 2; i < keys.size(); i++)
	{
		A(0, i) = 0;
	}
	for (int i = 0; i < keys.size() - 2; i++)
	{
		A(keys.size() - 1, i) = 0;
	}
	for (int i = 1; i < keys.size() - 1; i++)
	{
		for (int j = 0; j < keys.size(); j++)
		{
			if (i == j)
			{
				A(i, j) = 4;
			}
			else if (i - 1 == j || i + 1 == j)
			{
				A(i, j) = 1;
			}
			else
			{
				A(i, j) = 0;
			}
		}
	}
	// Initialize D
	vec3 resultFirst = 3 * (keys[1].second - keys[0].second);
	vec3 resultFinal = 3 * (keys.back().second - keys[keys.size() - 2].second);
	D(0, 0) = resultFirst[0];
	D(0, 1) = resultFirst[1];
	D(0, 2) = resultFirst[2];
	D(keys.size() - 1, 0) = resultFinal[0];
	D(keys.size() - 1, 1) = resultFinal[1];
	D(keys.size() - 1, 2) = resultFinal[2];
	for (int i = 1; i < keys.size() - 1; i++)
	{
		vec3 Drow = 3 * (keys[i + 1].second - keys[i - 1].second);
		for (int j = 0; j < 3; j++)
		{
			D(i, j) = Drow[j];
		}
	}
	Eigen::MatrixXd C(keys.size(), 3);
	C = A.inverse() * D;
	for (int i = 0; i < ctrlPoints.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			ctrlPoints[i][j] = C(i, j);
		}
	}
}

std::vector<double> ABSplineInterpolatorVec3::N(std::vector<double>& knots, int n, int j, double t) const
{
	Eigen::MatrixXd BB(n + 1, n + 2);
	std::vector<double> result;
	for (int i = 0; i < n + 1; i++)
	{
		for (int k = 0; k < n + 2; k++)
		{
			BB(i, k) = 0.;
		}
	}
	BB(0, n) = 1.;
	for (int i = 1; i < n + 1; i++)
	{
		for (int k = 0; k < n + 1; k++)
		{
			double realN = i;
			double realJ = j - 3 + k;
			BB(i, k) = ((t - knots[realJ]) / (knots[realJ + realN] - knots[realJ])) * BB(i - 1, k)
				+ ((knots[realJ + realN + 1] - t) / (knots[realJ + realN + 1] - knots[realJ + 1])) * BB(i - 1, k + 1);
		}
	}

	for (int k = 0; k < n + 1; k++)
	{
		result.push_back(BB(n, k));
	}
	return result;
}

std::vector<double> ABSplineInterpolatorVec3::dN(std::vector<double>& knots, int n, int j, double t, double l) const
{
	Eigen::MatrixXd BB(n, n + 2);
	std::vector<double> result;
	for (int i = 0; i < n; i++)
	{
		for (int k = 0; k < n + 2; k++)
		{
			BB(i, k) = 0.;
		}
	}
	std::vector<double> values = N(knots, 1, j, t);
	BB(0, 3) = values[1];
	BB(0, 2) = values[0];
	for (int i = 1; i < n; i++)
	{
		for (int k = 0; k < n + 1; k++)
		{
			double realJ = j - 3 + k;
			double realN = i + 1;
			BB(i, k) = realN * ((1 / (knots[realJ + realN] - knots[realJ])) * BB(i - 1, k)
				- (1 / (knots[realJ + realN + 1] - knots[realJ + 1])) * BB(i - 1, k + 1));
		}
	}
	for (int k = 0; k < n + 1; k++)
	{
		result.push_back(BB(n - 1, k));
	}
	return result;
}

void ABSplineInterpolatorVec3::computeControlPoints(
	const std::vector<ASplineVec3::Key>& keys,
	std::vector<vec3>& ctrlPoints,
	vec3& startPt, vec3& endPt)
{
	ctrlPoints.clear();
	ctrlPoints.resize(keys.size() + 2, vec3(0, 0, 0));
	if (keys.size() <= 1) return;
	double t0 = 0;
	double t1 = 1;
	mKnots.clear();
	mKnots.push_back(t0 - 3 * (t1 - t0));
	mKnots.push_back(t0 - 2 * (t1 - t0));
	mKnots.push_back(t0 - (t1 - t0));
	int mn = keys.size() + 2;
	int m = mn - 3;
	for (int i = 0; i < m + 1; i++)
	{
		mKnots.push_back(i);
	}
	double tn = mKnots[mKnots.size() - 1];
	double tn_1 = mKnots[mKnots.size() - 2];
	mKnots.push_back(tn + (tn - tn_1));
	mKnots.push_back(tn + 2 * (tn - tn_1));
	mKnots.push_back(tn + 3 * (tn - tn_1));

	Eigen::MatrixXd A(mn, mn);
	Eigen::MatrixXd D(mn, 3);

	// Middle rows
	int count = 0;
	std::vector<double> values;
	for (int i = 1; i < mn - 3; i++)
	{
		for (int j = 0; j < i - 1; j++)
		{
			A(i, j) = 0;
		}
		count = 0;
		values = N(mKnots, 3, i + 2, i - 1);
		for (int j = i - 1; j < i + 3; j++)
		{
			A(i, j) = values[count++];
		}
		for (int j = i + 3; j < mn; j++)
		{
			A(i, j) = 0;
		}
	}
	// Row m+n-3
	values = N(mKnots, 3, m + 2, m - 1);
	count = 0;
	for (int j = 0; j < mn; j++)
	{

		if (j >= mn - 4)
		{

			A(keys.size() - 1, j) = values[count++];

		}
		else
		{
			A(keys.size() - 1, j) = 0.;
		}
	}
	// // Row m+n-2
	values = N(mKnots, 3, m + 2, m);
	count = 0;
	for (int j = 0; j < keys.size() + 2; j++)
	{
		if (j >= mn - 4)
		{
			A(keys.size(), j) = values[count++];
		}
		else
		{
			A(keys.size(), j) = 0.;
		}
	}
	// First row
	values = dN(mKnots, 3, 3, 0, 2);
	for (int i = 0; i < mn; i++)
	{
		if (i < 4)
		{
			A(0, i) = values[i];
		}
		else
		{
			A(0, i) = 0.;
		}
	}
	// Last row
	values = dN(mKnots, 3, m + 2, m, 2);
	count = 0;
	for (int j = 0; j < mn; j++)
	{
		if (j >= mn - 4)
		{
			A(mn - 1, j) = values[count++];
		}
		else
		{
			A(mn - 1, j) = 0.;
		}
	}
	// Set d
	for (int j = 0; j < 3; j++)
	{
		D(0, j) = 0.;
		D(keys.size() + 1, j) = 0.;
	}
	for (int i = 1; i < keys.size() + 1; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			D(i, j) = keys[i - 1].second[j];
		}
	}
	Eigen::MatrixXd C(keys.size() + 2, 3);
	C = A.inverse() * D;
	for (int i = 0; i < ctrlPoints.size(); i++)
	{
		for (int j = 0; j < 3; j++)
		{
			ctrlPoints[i][j] = C(i, j);
		}
	}
}


vec3 AEulerLinearInterpolatorVec3::interpolateSegment(
	const std::vector<ASplineVec3::Key>& keys,
	const std::vector<vec3>& ctrlPoints,
	int segment, double u)
{
	// TODO:
	// Linear interpolate between key0 and key1
	// You should convert the angles to find the shortest path for interpolation
	vec3 key0 = getAngleIn180(keys[segment].second);
	vec3 key1 = getAngleIn180(keys[segment + 1].second);
	for (int i = 0; i < 3; i++)
	{
		if (std::abs(key0[i] - key1[i]) > 180)
		{
			if (key1[i] < 0)
			{
				key1[i] += 360;
			}
			else if (key1[i] > 0)
			{
				key1[i] -= 360;
			}
		}
	}
	return key0 * (1 - u) + key1 * u;
}

vec3 AEulerCubicInterpolatorVec3::interpolateSegment(
	const std::vector<ASplineVec3::Key>& keys,
	const std::vector<vec3>& ctrlPoints, int segment, double u)
{
	// TODO: 
	// Step1: Get the 4 control points, b0, b1, b2 and b3 from the ctrlPoints vector
	// Step2: Compute the interpolated value f(u) point using  Bernstein polynomials
	// You should convert the angles to find the shortest path for interpolation
	vec3 b0 = ctrlPoints[segment * 4];
	vec3 b1 = ctrlPoints[segment * 4 + 1];
	vec3 b2 = ctrlPoints[segment * 4 + 2];
	vec3 b3 = ctrlPoints[segment * 4 + 3];
	b1 = getInterpolatedKey(b0, b1);
	b2 = getInterpolatedKey(b1, b2);
	b3 = getInterpolatedKey(b2, b3);
	return b0 * std::pow(1 - u, 3) + b1 * 3 * u * std::pow(1 - u, 2) + b2 * 3 * std::pow(u, 2) * (1 - u) + b3 * std::pow(u, 3);
}

void AEulerCubicInterpolatorVec3::computeControlPoints(
	const std::vector<ASplineVec3::Key>& keys,
	std::vector<vec3>& ctrlPoints, vec3& startPoint, vec3& endPoint)
{

	// Hint: One naive way is to first convert the keys such that the differences of the x, y, z Euler angles 
	//		 between every two adjacent keys are less than 180 degrees respectively 

	ctrlPoints.clear();
	if (keys.size() <= 1) return;
	vec3 b0, b1, b2, b3;
	std::vector<vec3> shortestAngles;
	shortestAngles.push_back(getAngleIn180(startPoint));
	for (int i = 0; i < keys.size(); i++)
	{
		shortestAngles.push_back(getInterpolatedKey(shortestAngles[i], getAngleIn180(keys[i].second)));
	}
	shortestAngles.push_back(getInterpolatedKey(shortestAngles.back(), getAngleIn180(endPoint)));
	for (int i = 1; i < shortestAngles.size() - 2; i++)
	{

		// TODO: compute b0, b1, b2, b3
		b0 = shortestAngles[i];
		b1 = b0 + (1 / 3.) * 0.5 * (shortestAngles[i + 1] - shortestAngles[i - 1]);
		b2 = shortestAngles[i + 1] - (1 / 3.) * 0.5 * (shortestAngles[i + 2] - shortestAngles[i]);
		b3 = shortestAngles[i + 1];
		ctrlPoints.push_back(b0);
		ctrlPoints.push_back(b1);
		ctrlPoints.push_back(b2);
		ctrlPoints.push_back(b3);
	}
}

vec3 AEulerCubicInterpolatorVec3::getInterpolatedKey(const vec3& key0, const vec3& key1)
{
	vec3 result = key1;
	for (int i = 0; i < 3; i++)
	{
		if (std::abs(key0[i] - key1[i]) > 180)
		{
			if (key1[i] < 0)
			{
				result[i] += 360;
			}
			else if (key1[i] > 0)
			{
				result[i] -= 360;
			}
		}
	}
	return result;
}

vec3 AEulerCubicInterpolatorVec3::getAngleIn180(const vec3& key)
{
	vec3 result = key;
	for (int i = 0; i < 3; i++)
	{
		result[i] = std::fmod(result[i], 360);
		if (result[i] > 180)
		{
			result[i] -= 360;
		}
		else if (result[i] < -180)
		{
			result[i] += 360;
		}
	}
	return result;
}

vec3 AEulerLinearInterpolatorVec3::getAngleIn180(const vec3& key)
{
	vec3 result = key;
	for (int i = 0; i < 3; i++)
	{
		result[i] = std::fmod(result[i], 360);
		if (result[i] > 180)
		{
			result[i] -= 360;
		}
		else if (result[i] < -180)
		{
			result[i] += 360;
		}
	}
	return result;
}