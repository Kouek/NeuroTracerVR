#ifndef KOUEK_VOL_CFG_H
#define KOUEK_VOL_CFG_H

#include <fstream>
#include <sstream>

#include <glm/glm.hpp>

#include <json.hpp>

namespace kouek {

class VolCfg {
  private:
    float baseSpace;
    glm::vec3 spaces;
    glm::vec3 startPos;
    std::array<glm::vec4, 256> tfPnts;
    std::string volumePath;
    std::string SWCPath;

    nlohmann::json json;

  public:
    /// <summary>
    /// Volume Config loaded from volumeCfgPath.
    /// </summary>
    /// <param name="volumeCfgPath">absolute Path of Volume Config File</param>
    VolCfg(const std::string &volumeCfgPath) {
        using namespace std;
        using namespace nlohmann;

        ifstream in(volumeCfgPath.data());
        if (!in.is_open())
            throw runtime_error("Cannot open file: " + volumeCfgPath);

        try {
            in >> json;
            in.close();

            auto jsonItem = json.at("spaces");
            spaces.x = jsonItem[0];
            spaces.y = jsonItem[1];
            spaces.z = jsonItem[2];
            baseSpace = min({spaces.x, spaces.y, spaces.z});

            volumePath = json.at("volume_path");
            SWCPath = json.at("swc_path");

            jsonItem = json.at("start_pos");
            {
                startPos.x = jsonItem[0];
                startPos.y = jsonItem[1];
                startPos.z = jsonItem[2];
            }

            jsonItem = json.at("tf");
            {
                using KeyPntTy = std::pair<uint8_t, glm::vec4>;
                std::vector<KeyPntTy> keyPnts;
                for (auto &[key, vals] : jsonItem.items()) {
                    keyPnts.emplace_back(
                        (uint8_t)stoi(key),
                        glm::vec4{vals[0], vals[1], vals[2], vals[3]});
                }
                std::sort(keyPnts.begin(), keyPnts.end(),
                          [](const KeyPntTy &a, const KeyPntTy &b) {
                              return a.first < b.first;
                          });

                auto keyPntItr = keyPnts.begin();
                uint8_t lftIdx = 0, idxDlt;
                glm::vec4 valsDlt;
                idxDlt = keyPntItr->first - lftIdx;
                valsDlt = keyPntItr->second;
                for (uint32_t idx = 0; idx < 256; ++idx) {
                    if (idx < keyPntItr->first)
                        tfPnts[idx] = keyPntItr->second +
                                      (float)(idx - lftIdx) / idxDlt * valsDlt;
                    else if (idx == keyPntItr->first) {
                        tfPnts[idx] = keyPntItr->second;
                        lftIdx = keyPntItr->first;

                        auto nextItr = keyPntItr;
                        ++nextItr;
                        if (nextItr != keyPnts.end()) {
                            idxDlt = nextItr->first - lftIdx;
                            valsDlt = nextItr->second - keyPntItr->second;
                            keyPntItr = nextItr;
                        }
                    } else
                        tfPnts[idx] = keyPntItr->second;
                }
            }
        } catch (json::parse_error &e) {
            throw runtime_error(
                "Problem occurs at byte: " + std::to_string(e.byte) +
                ". Problem is: " + e.what());
        }
    }
    const auto &GetTFPoints() const { return tfPnts; }
    const std::string &GetSWCPath() const { return SWCPath; }

#define GETTER(retType, firstChInLowerCase, firstChInUpperCase, successor)     \
    retType Get##firstChInUpperCase##successor() const {                       \
        return firstChInLowerCase##successor;                                  \
    }
    GETTER(glm::vec3, s, S, tartPos)
    GETTER(glm::vec3, s, S, paces)
    GETTER(float, b, B, aseSpace)
    GETTER(const std::string &, v, V, olumePath)
#undef GETTER
};

} // namespace kouek

#endif // !KOUEK_VOL_CFG_H
