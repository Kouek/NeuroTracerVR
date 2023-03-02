#ifndef KOUEK_COMP_VOL_ALGORITHM_H
#define KOUEK_COMP_VOL_ALGORITHM_H

#include <limits>

#include <tuple>

#include <glm/glm.hpp>

namespace kouek {

static constexpr auto NONE_POS_VAL = std::numeric_limits<float>::quiet_NaN();

// <<
// In this scope, VRSp refers to Volume Render Space
void prepareMaxVoxPos(const glm::uvec3 &sampleBoxSzVRSp, float minScalar);
void execMaxVoxPos(const glm::vec3 &minVRSp, const glm::vec3 &maxVRSp);
std::tuple<bool, glm::vec3> fetchMaxVoxPos();
// >>

void waitForAllVoxAlgorithms();

} // namespace kouek

#endif // !KOUEK_COMP_VOL_ALGORITHM_H
