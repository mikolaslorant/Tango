#include "camera.h"
#include <iostream>

Camera::Camera(unsigned int w, unsigned int h, const glm::vec3 &e, const glm::vec3 &r, const glm::vec3 &worldUp) :
	fovy(45),
	width(w),
	height(h),
	near_clip(1.0f),
	far_clip(4000),
	eye(e),
	ref(r),
	world_up(worldUp),
	xaxis(glm::vec3(1, 0, 0)),
	yaxis(glm::vec3(0, 1, 0)),
	zaxis(glm::vec3(0, 0, 1)),
	theta(0.f),
	phi(0.f),
	zoom(0.f)

{
	look = glm::normalize(ref - eye);
	right = glm::normalize(glm::cross(look, world_up));
	up = glm::cross(right, look);

	float tan_fovy = tan(glm::radians(fovy / 2));
	float len = glm::length(ref - eye);
	aspect = (float)width / (float)height;

	eyeOrigin = eye;
	upOrigin = up;
	rightOrigin = right;
	refOrigin = ref;
}


glm::mat4 Camera::getProjView()
{
	return getProj() * getView();
}

glm::mat4 Camera::getProj()
{
	return glm::perspective(glm::radians(fovy), width / (float)height, near_clip, far_clip);
}

glm::mat4 Camera::getView()
{
	return glm::lookAt(eye, ref, up);
}

glm::vec3 Camera::getEye()
{
	return eye;
}


void Camera::PolarRotateAboutX(float deg)
{
	phi += deg;
	phi = std::fmax(phi, -85);
	phi = std::fmin(0, phi);

	PolarRecomputeAttributes();
}

void Camera::PolarRotateAboutY(float deg)
{
	theta += deg;
	PolarRecomputeAttributes();
}

void Camera::PolarZoom(float amt)
{
	if ((zoom + amt) > (glm::length(eye - ref) - 200.0f))
	{
		return;
	}
	zoom += amt;
	PolarRecomputeAttributes();
}

void Camera::PolarPanX(float amt)
{
	refOrigin += right * amt;
	eyeOrigin += right * amt;
	ref = refOrigin;
	PolarRecomputeAttributes();
}

void Camera::PolarPanY(float amt)
{
	refOrigin += up * amt;
	eyeOrigin += up * amt;
	ref = refOrigin;
	PolarRecomputeAttributes();
}

void Camera::PolarRecomputeAttributes()
{
	look = glm::normalize(refOrigin - eyeOrigin);
	glm::mat4 sphericalTransform =
		glm::rotate(glm::mat4(1.0f), glm::radians(theta), yaxis) *
		glm::rotate(glm::mat4(1.0f), glm::radians(phi), xaxis) *
		glm::translate(glm::mat4(1.0f), zoom * look);

	glm::vec4 sphericalEye = sphericalTransform * glm::vec4(eyeOrigin - refOrigin, 1.0f);
	eye = glm::vec3(sphericalEye) + refOrigin;
	up = glm::normalize(glm::vec3(sphericalTransform * glm::vec4(upOrigin, 0.0f)));
	right = glm::normalize(glm::cross(glm::normalize(ref - eye), up));
}
