#pragma once

// STL

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <string>
#include <sstream>
#include <functional>
#include <typeinfo>
#include <algorithm>
#include <utility>

// GLEW

#define GLEW_STATIC
#include <GL/glew.h>

// GLM

#define GLM_FORCE_RADIANS

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/norm.hpp>


// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


// Bullet
#include <btBulletDynamicsCommon.h>


#include <backend/maths.h>
#include <utils.h>
// common include files that may be needed by almost every source.
