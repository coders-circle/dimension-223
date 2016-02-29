#pragma once

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

template <class T=glm::vec4>
inline bool IsGrayscale() { return false; }

template<>
inline bool IsGrayscale<float>() { return true; }
