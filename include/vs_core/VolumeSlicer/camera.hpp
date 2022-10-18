//
// Created by wyz on 2021/6/15.
//

#ifndef VOLUMESLICER_CAMERA_HPP
#define VOLUMESLICER_CAMERA_HPP

#include <VolumeSlicer/export.hpp>
#include <VolumeSlicer/status.hpp>
#include <VolumeSlicer/define.hpp>

#include<array>

VS_START
class Camera {
public:
    std::array<float, 16> unProjection;
    std::array<float, 9> eyeToWorldRotate;
    float zoom;                 // FOV on y-axis
    std::array<float, 3> pos;
    std::array<float, 3> look_at;
    std::array<float, 3> up;    // normalized
    std::array<float, 3> right; // normalized
    float n, f;
};

VS_END


#endif //VOLUMESLICER_CAMERA_HPP
