#ifndef KOUEK_COMP_VOL_VR_RENDERER_H
#define KOUEK_COMP_VOL_VR_RENDERER_H

#include "../vr_renderer.h"

#include <unordered_set>

#include <VolumeSlicer/volume.hpp>
#include <VolumeSlicer/volume_cache.hpp>

#include <Common/boundingbox.hpp>
#include <Common/hash_function.hpp>

namespace kouek {
class CompVolVRRenderer : public VRRenderer {
  public:
    struct RendererParam {
        uint32_t texUnitNum;
        glm::uvec3 texUnitShape;
    };

    struct VolumeParam {
        uint32_t blockLength;
        uint32_t padding;
        uint32_t noPaddingBlockLength;
        glm::uvec3 LOD0BlockDim;
        glm::vec3 spaces;
    };

  public:
    static constexpr auto MAX_LOD = 6;
    static constexpr auto MAX_TEX_UNIT_NUM = 10;

  private:
    RendererParam rendererParam;

    std::unique_ptr<vs::CUDAVolumeBlockCache> blockCache;
    std::shared_ptr<vs::CompVolume> volume;

    std::unordered_map<std::array<uint32_t, 3>, vs::AABB, Hash_UInt32Array3>
        blockAABBs;
    std::unordered_set<std::array<uint32_t, 4>, Hash_UInt32Array4> needBlocks,
        currNeedBlocks;
    std::unordered_set<std::array<uint32_t, 4>, Hash_UInt32Array4> loadBlocks,
        unloadBlocks;

  public:
    CompVolVRRenderer(const RendererParam &cudaParam);

    /// <summary>
    /// Note: Should be called after SetVolume()
    /// </summary>
    virtual void SetCameraParam(const CameraParam &param) override;
    virtual void
    Render(RenderTarget renderTarget = RenderTarget::FullResVol) override;

    void SetVolume(std::shared_ptr<vs::CompVolume> volume,
                   const glm::vec3 &spaces);

  private:
    void render(RenderTarget renderTarget);
};
} // namespace kouek

#endif // !KOUEK_COMP_VOL_VR_RENDERER_H
