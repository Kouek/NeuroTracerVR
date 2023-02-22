#ifndef KOUEK_COLOR_TBL_H
#define KOUEK_COLOR_TBL_H

#include <queue>

#include <glm/glm.hpp>

namespace kouek {

class ColorTable {
  private:
    static constexpr std::array<glm::vec3, 5> tbl = []() {
        std::array<glm::vec3, 5> tbl{
            glm::vec3{255.f, 192.f, 203.f}, glm::vec3{34.0f, 139.f, 34.0f},
            glm::vec3{255.f, 000.f, 255.f}, glm::vec3{3.00f, 168.f, 158.f},
            glm::vec3{227.f, 23.0f, 13.0f}};
        float inv255 = 1.f / 255.f;
        for (auto &col : tbl) {
            col[0] *= inv255;
            col[1] *= inv255;
            col[2] *= inv255;
        }
        return tbl;
    }();
    size_t currIdx = 0;

  public:
    const glm::vec3 &NextColor() {
        if (currIdx == tbl.size())
            currIdx = 0;
        return tbl[currIdx++];
    }
};

} // namespace kouek

#endif // !KOUEK_COLOR_TBL_H
