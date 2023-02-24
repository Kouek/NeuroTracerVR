#ifndef KOUEK_COMP_VOL_ALGORITHM_H
#define KOUEK_COMP_VOL_ALGORITHM_H

#include <limits>

#include <tuple>

#include <glm/glm.hpp>

namespace kouek {

static constexpr auto NONE_POS_VAL = std::numeric_limits<float>::infinity();

void prepareMaxVoxPos(const glm::uvec3 &sampleBoxSzVSp, float minScalar);
void execMaxVoxPos(const glm::vec3 &minVSp, const glm::vec3 &maxVSp);
std::tuple<bool, glm::vec3> fetchMaxVoxPos();

void waitForAllVoxAlgorithms();

} // namespace kouek

#endif // !KOUEK_COMP_VOL_ALGORITHM_H
