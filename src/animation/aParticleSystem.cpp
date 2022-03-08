#include "aParticleSystem.h"
#include <Windows.h>
#include <cstring>
#include <math.h>
#include "aVector.h"


#define GRAVITY 9.8f

AParticleSystem::AParticleSystem()
{

	mMaxParticles = 20;
	mParticles.clear();

	mDeltaT = 0.033;
	mLifeSpan = 3.0; 
	mJitterTime = AJitter(-0.5 * mLifeSpan, 0);
	mSpawnTime = (int)(mLifeSpan / (mMaxParticles*mDeltaT));
	mSpawnDelay = 0;

	mGravity = vec3(0, -9.8, 0);

	mStartColor = vec3(1, 0, 0);
	mEndColor = vec3(0, 0, 0);
	mColorJitter = AJitter(-0.25, 0.25);

	mStartPos = vec3(0, 0, 0);
	mPositionJitter = AJitter(-0.5, 0.5);

	mStartVel = vec3(0, 20.0, 0);  
	mVelocityJitter = AJitter(-2.0, 2.0);

	mStartAlpha = 1.0;
	mEndAlpha = 0.0;

	mStartScale = 0.1;
	mEndScale = 1.0;
	mScaleJitter = AJitter(0, 0.25);
	
	mRoot = NULL;
}

AParticleSystem::~AParticleSystem()
{
	for (unsigned int i = 0; i < mParticles.size(); i++)
	{
		AParticle* pParticle = mParticles[i];
		delete pParticle;
	}

	mParticles.clear();
}

bool AParticleSystem::isAlive()
{
	if (mInfinite) { return true; } // if mInfinite = true particles never die!
	if (mParticles.size() < mMaxParticles) { return true; }
    for (unsigned int i = 0; i < mParticles.size(); i++)
    {
        if (mParticles[i]->isAlive()) 
			return true;
    }
    return false;
}

void AParticleSystem::update(double deltaT)
{
	if (mParticles.size() == 0 && mMaxParticles == 0) { return; }
	int forceMode = 0;
	mDeltaT = deltaT;
	if (mParticles.size() > mMaxParticles)
	{
		for (unsigned int i = mParticles.size() - 1; i >= mMaxParticles; --i)
		{
			AParticle* pParticle = mParticles[i];
			delete pParticle;
			mParticles.pop_back();
		}
	}

	for (unsigned int i = 0; i < mParticles.size(); i++)
	{
		mParticles[i]->update(deltaT, forceMode);
		if (mInfinite && !mParticles[i]->isAlive())
		{
			mParticles[i]->initialize(*this);	// Respawn
		}
	}
	if (mParticles.size() < mMaxParticles)  // add new particle
	{
		mSpawnDelay++;
		if (mSpawnDelay == mSpawnTime)
		{
			AParticle *particle = new AParticle();
			particle->initialize(*this);

			if (mInfinite) // jitter the LifeSpan in infinite mode
			{
				double newLifeSpan = mLifeSpan; //+ AJitterVal(mJitterTime);
				particle->setLifeSpan(newLifeSpan);
			}
			mParticles.push_back(particle);
			mSpawnDelay = 0;
		}
	}
}

void AParticleSystem::reset()
{
	for (unsigned int i = 0; i < mParticles.size(); i++)
	{
		AParticle* pParticle = mParticles[i];
		delete pParticle;
	}
    mParticles.clear();
	mSpawnTime = (int)(mLifeSpan / (mMaxParticles*mDeltaT));

}

