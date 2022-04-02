#pragma once
#include <glm.hpp>
#include <gtc/matrix_transform.hpp>

//A perspective projection camera
class Camera
{
public:

	Camera(unsigned int w, unsigned int h, const glm::vec3 &e, const glm::vec3 &r, const glm::vec3 &worldUp);

	float fovy;
	unsigned int width, height;  // Screen dimensions
	float near_clip;  // Near clip plane distance
	float far_clip;  // Far clip plane distance

	//Computed attributes
	float aspect;

	glm::vec3 eye,      //The position of the camera in world space
		ref,      //The point in world space towards which the camera is pointing
		look,     //The normalized vector from eye to ref. Is also known as the camera's "forward" vector.
		up,       //The normalized vector pointing upwards IN CAMERA SPACE. This vector is perpendicular to LOOK and RIGHT.
		right,    //The normalized vector pointing rightwards IN CAMERA SPACE. It is perpendicular to UP and LOOK.
		world_up; //The normalized vector pointing upwards IN WORLD SPACE. This is primarily used for computing the camera's initial UP vector.

	glm::vec3 eyeOrigin, upOrigin, rightOrigin, refOrigin;
	glm::vec3 xaxis, yaxis, zaxis;    // The unit vectors of x-axis, y-axis, z-axis in world space
	float theta;  // Angle for y-aixs
	float phi;    // Angle for x-axis
	float zoom;   // Radius

	glm::mat4 getProjView();
	glm::mat4 getProj();
	glm::mat4 getView();
	glm::vec3 getEye();


	void PolarRotateAboutX(float deg);
	void PolarRotateAboutY(float deg);
	void PolarZoom(float amt);
	void PolarPanX(float amt);
	void PolarPanY(float amt);

	void PolarRecomputeAttributes();
};