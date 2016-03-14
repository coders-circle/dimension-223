#pragma once

/**
 * Convert vec4 color i.e. RGBA value to particular type.
 * @param color Vec4 color to convert from.
 * @return Color of type T.
 */
template <class T=glm::vec4>
inline T VecToColor(glm::vec4 color)
{
    return color;
}

template <>
inline float VecToColor<float>(glm::vec4 color)
{
    return color[0];
}


/**
 * Convert color of particular type to vec4 i.e. RGBA format.
 * @param color Color of type T.
 * @return color of type vec4.
 */
template <class T=glm::vec4>
inline glm::vec4 ColorToVec(T color)
{
    return color;
}

template <>
inline glm::vec4 ColorToVec<float>(float color)
{
    return glm::vec4(color, color, color, 1.0f);
}

/**
 * Check whether the given type of color is grayscale or not.
 * @return true if T is float (grayscale) and false otherwise.
 */
template <class T=glm::vec4>
inline bool IsGrayscale() { return false; }

template<>
inline bool IsGrayscale<float>() { return true; }
