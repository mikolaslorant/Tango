// Rocket.cpp: implementation of the ARocket class.
//
//////////////////////////////////////////////////////////////////////

#include "aRocket.h"
#include <stdlib.h>
#include <math.h>
#include <time.h>

#ifndef RAD
#define PI 3.14159265358979f
#define RAD (PI / 180.0f)
#endif
#ifndef GRAVITY
#define GRAVITY 9.8f
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

ARocket::ARocket(vec3 color): ASpark()
{
	m_explosionCount = -1;
	for (int i = 0; i < 3; i++)
		m_color[i] = color[i];
	m_mode = FLYING;
	m_Vexplode = 50.0;
	
	m_mass = 50.0;
}

ARocket::~ARocket()
{

}

void ARocket::update(float deltaT, int extForceMode)
{

	if (m_mode == EXPLOSION && m_explosionCount > 0)
		m_explosionCount--;

	
	if ( !(extForceMode & EXT_ROCKETFORCES_ACTIVE))
		extForceMode = 0;
	computeForces(extForceMode);
	updateState(deltaT, EULER);


	// resolve collisions with ground
	if (m_state[1] <= 0 && m_state[4] < 0)
	{
		m_state[4] = -m_state[4];
	}

	float Vmin = m_Vexplode;
	if (m_state[4] < Vmin && m_mode == FLYING)
	{
		m_mode = EXPLOSION;
		m_explosionCount = TOTALEXPLOSIONS;
	}

	if (m_mode == EXPLOSION && m_explosionCount == 0)
	{
		m_mode = DEAD;
		m_alive = false;
	}
}

 /*	computeForces() computes the forces applied to this rocket
 *  In this fucntion, you need to set the correct value for state[6] and state[7], 
 *  which are forces along X and Y axis.
 */
void ARocket::computeForces(int extForceMode)
{
	ASpark::computeForces(extForceMode);
}


