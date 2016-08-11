#pragma once


// Mark the points in image as a plane surface.
struct Surface {
    std::vector<bool> marked;
    float y;

    Surface() 
        : y(-9999999)
    {}
};
