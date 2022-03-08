#include "utils.h"
#include <gtc/matrix_transform.hpp>
#include <gtc/type_ptr.hpp>

glm::mat4 toGLMmat4(const mat3 & rot, const vec3 & tran)
{
	float m[16];
	rot.WriteToGLMatrix(m);
	m[12] = static_cast<float>(tran[0]);
	m[13] = static_cast<float>(tran[1]);
	m[14] = static_cast<float>(tran[2]);
	return glm::make_mat4(m);
}

glm::vec3 toGLMvec3(const vec3 & tran)
{
	return glm::vec3(tran[0], tran[1], tran[2]);
}
