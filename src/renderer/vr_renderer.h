#ifndef KOUEK_VR_RENDERER_H
#define KOUEK_VR_RENDERER_H

#include <array>
#include <memory>

#include <glad/glad.h>
#include <glm/gtc/matrix_transform.hpp>

namespace kouek {

class VRRenderer {
  public:
    static constexpr auto MAX_SUBSAMPLE_LEVEL = 6;
    static constexpr auto SUBSAMPLE_MAP_TO_EMPTY =
        std::numeric_limits<uint32_t>::max();

    struct CUDAxGLParam {
        std::array<GLuint, 2> outputTex2;
        std::array<GLuint, 2> inputDepTex2;
        glm::uvec2 renderSz;
    };

    struct LightParam {
        float ka;
        float kd;
        float ks;
        float shininess;
    };

    struct RenderParam {
        uint8_t FAVRLvl;
        uint32_t maxStepCnt;
        glm::uvec2 renderSz;
        float step;
    };

    /// <summary>
    /// In World Space, but after pre-translated and pre-scaled(angle kept)
    /// </summary>
    struct CameraParam {
        float OBBBorderDistToEyeCntr;
        glm::vec3 OBBHfSz;
        glm::vec3 pos2[2];
        glm::mat3 rotation;
    };

    struct ProjectionParam {
        glm::mat4 unProjection2[2];
        float projection22[2], projection23[2];
    };

    enum class RenderTarget : uint8_t {
        FullResVol,
        FAVRVol,
        FAVRSubsampleCoord,
        FAVRReconsCoord,
        End
    };

  protected:
    uint8_t FAVRLvl, FAVRIdx;
    std::array<uint32_t, MAX_SUBSAMPLE_LEVEL> FAVRSubsampleWidths;
    glm::uvec2 renderSz;

  public:
    virtual void SetCUDAxGLParam(const CUDAxGLParam &param);
    virtual void SetLightParam(const LightParam &param);
    virtual void SetRenderParam(const RenderParam &param);
    virtual void SetTransferFunction(const std::array<glm::vec4, 256> &tfPnts);
    virtual void SetCameraParam(const CameraParam &param);
    virtual void SetProjectionParam(const ProjectionParam &param);
    virtual void
    Render(RenderTarget renderTarget = RenderTarget::FullResVol) = 0;

  private:
    void createSubsampleAndReconsTex();
};
} // namespace kouek

#endif // !KOUEK_VR_RENDERER_H
