#ifndef KOUEK_GL_PATH_RENDERER_H
#define KOUEK_GL_PATH_RENDERER_H

#include <limits>

#include <array>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <glad/glad.h>

#include <cg/shader.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace kouek {

class GLPathRenderer {
  public:
    static constexpr GLuint MAX_VERT_NUM = 10000;
    static constexpr GLuint MAX_PATH_NUM = 100;
    static constexpr GLuint NONE = std::numeric_limits<GLuint>::max();
    static constexpr GLuint DIV_NUM = 8;
    static constexpr GLuint BLOCK_NUM = 100;

    inline static GLfloat szScale = 1.f;
    inline static GLfloat rootVertSize = .015f;
    inline static GLfloat endVertSize = .004f;
    inline static GLfloat selectedVertSize = .01f;
    inline static GLfloat secondSelectedVertSize = .007f;
    inline static GLfloat lineWidth = .003f;
    inline static GLfloat selectedLineWidth = .006f;
    inline static glm::vec3 selectedVertColor{1.f, .5f, 1.f};

  private:
    static constexpr GLubyte VERT_DAT_POS_NUM = 3;
    static constexpr GLsizeiptr VERT_DAT_POS_SIZE =
        sizeof(GLfloat) * VERT_DAT_POS_NUM;
    static constexpr GLsizeiptr VERT_DAT_STRIDE = VERT_DAT_POS_SIZE;

#define VERTEX_ARRAY_DEF                                                       \
    glVertexAttribPointer(0, VERT_DAT_POS_NUM, GL_FLOAT, GL_FALSE,             \
                          VERT_DAT_STRIDE, nullptr);                           \
    glEnableVertexAttribArray(0);

    struct SubPath {
        friend class GLPathRenderer;

        bool needUpload = false;
        GLuint VAO = 0, EBO = 0;
        GLuint vertGPUCap = 8;
        std::vector<GLuint> verts;
        SubPath(GLuint startVertID) {
            verts.emplace_back(startVertID);

            glGenVertexArrays(1, &VAO);
            glBindVertexArray(VAO);
            VERTEX_ARRAY_DEF;
            glGenBuffers(1, &EBO);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint) * vertGPUCap,
                         nullptr, GL_DYNAMIC_DRAW);
            glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, sizeof(GLuint),
                            &startVertID);
            glBindVertexArray(0);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        }
        ~SubPath() {
            if (VAO != 0)
                glDeleteVertexArrays(1, &VAO);
            VAO = 0;
            if (EBO != 0)
                glDeleteBuffers(1, &EBO);
            EBO = 0;
        }
        SubPath(SubPath &&right) noexcept {
            needUpload = right.needUpload;
            VAO = right.VAO;
            EBO = right.EBO;
            right.VAO = right.EBO = 0;
            vertGPUCap = right.vertGPUCap;
            verts = std::move(right.verts);
        }

      private:
        inline void addVertex(GLuint vertID) {
            verts.emplace_back(vertID);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
            if (vertGPUCap < verts.size()) {
                vertGPUCap = vertGPUCap << 1;
                glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                             sizeof(GLuint) * vertGPUCap, nullptr,
                             GL_DYNAMIC_DRAW);
                glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0,
                                sizeof(GLuint) * verts.size(), verts.data());
            } else
                glBufferSubData(GL_ELEMENT_ARRAY_BUFFER,
                                sizeof(GLuint) * (verts.size() - 1),
                                sizeof(GLuint), &vertID);
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        }
        inline void upload() {
            if (!needUpload)
                return;

            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
            if (vertGPUCap < verts.size()) {
                vertGPUCap = vertGPUCap << 1;
                glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                             sizeof(GLuint) * vertGPUCap, nullptr,
                             GL_DYNAMIC_DRAW);
            }
            glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0,
                            sizeof(GLuint) * verts.size(), verts.data());
            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

            needUpload = false;
        }
        inline void drawLineStrips() {
            glBindVertexArray(VAO);
            glDrawElements(GL_LINE_STRIP, verts.size(), GL_UNSIGNED_INT, 0);
        }
        inline void drawVerts() {
            glBindVertexArray(VAO);
            glDrawElements(GL_POINTS, verts.size(), GL_UNSIGNED_INT, 0);
        }
        inline void drawEndVerts() {
            glBindVertexArray(VAO);
            glDrawElements(GL_POINTS, 1, GL_UNSIGNED_INT, 0);
            glDrawElements(GL_POINTS, 1, GL_UNSIGNED_INT,
                           (const void *)(sizeof(GLuint) * (verts.size() - 1)));
        }
    };
    struct Path {
        friend class GLPathRenderer;

        GLuint rootID;
        glm::vec3 color;
        std::unordered_map<GLuint, SubPath> subPaths;
        std::queue<GLuint> recycledSubpathIDs;
        Path(const glm::vec3 &color, GLuint rootID)
            : color(color), rootID(rootID) {}
        ~Path() {}

      private:
        inline GLuint getRootID() const { return rootID; }
        inline const auto &getSubPaths() const { return subPaths; }
        inline void addSubPath(GLuint subPathID, GLuint startVertID) {
            subPaths.emplace(std::piecewise_construct,
                             std::forward_as_tuple(subPathID),
                             std::forward_as_tuple(startVertID));
        }
    };

    GLuint VBO = 0, VAO;
    struct {
        std::unique_ptr<Shader> shader;
        GLint MPos, VPPos;
        GLint widthPos;
        GLint colPos;
    } lineShader;
    struct {
        std::unique_ptr<Shader> shader;
        GLint MPos, VPPos;
        GLint szPos;
        GLint colPos;
    } vertShader;
    struct {
        std::unique_ptr<Shader> shader;
        GLint MPos, VPPos;
        GLint widthPos;
    } lineDepShader;
    struct {
        std::unique_ptr<Shader> shader;
        GLint MPos, VPPos;
        GLint szPos;
    } vertDepShader;

    struct GLuint3Hash {
        size_t operator()(const std::array<GLuint, 3> &idx3) const {
            std::array hash3 = {std::hash<GLuint>()(idx3[0]),
                                std::hash<GLuint>()(idx3[1]),
                                std::hash<GLuint>()(idx3[2])};
            constexpr auto BitNum = sizeof(size_t) * 8;
            return (hash3[0] << ((BitNum >> 1) - 1)) ^
                   (hash3[1] << ((BitNum >> 2) - 1)) ^ hash3[2];
        }
    };
    struct GLuint3Eq {
        bool operator()(const std::array<GLuint, 3> &a,
                        const std::array<GLuint, 3> &b) const {
            for (uint8_t i = 0; i < 3; ++i)
                if (a[i] != b[i])
                    return false;
            return true;
        }
    };

    GLuint extraVertID = NONE;
    GLuint selectedPathID = NONE;
    GLuint selectedVertID = NONE;
    GLuint scndSelectedVertID = NONE; // for 2 verts op
    GLuint selectedSubPathID = NONE;

    glm::vec3 bboxMax;

    std::vector<glm::vec3> verts;
    std::vector<GLuint> pathIDOfVerts;
    std::unordered_map<GLuint, Path> paths;
    std::unordered_map<std::array<GLuint, 3>, std::vector<GLuint>, GLuint3Hash,
                       GLuint3Eq>
        blkVertIDs;

    std::queue<GLuint> availableVertIDs;
    std::queue<GLuint> availableSubPathIDs;
    std::queue<GLuint> availablePathIDs;

  private:
    std::array<GLuint, 3> computeBlock(glm::vec3 pos) {
        std::array<GLuint, 3> ret;

        pos = glm::max(glm::zero<glm::vec3>(),
                       glm::min(glm::one<glm::vec3>(), pos / bboxMax));
        ret[0] = glm::floor(pos.x * BLOCK_NUM);
        ret[1] = glm::floor(pos.x * BLOCK_NUM);
        ret[2] = glm::floor(pos.x * BLOCK_NUM);

        return ret;
    }
    std::array<GLuint, 3> computeBlock(GLuint vertID) {
        auto &pos = verts[vertID];
        return computeBlock(pos);
    }
    void addVertToBlock(GLuint vertID) {
        auto blk = computeBlock(vertID);
        auto itr = blkVertIDs.find(blk);
        if (itr == blkVertIDs.end()) {
            auto [newItr, inserted] = blkVertIDs.emplace(
                std::piecewise_construct, std::forward_as_tuple(blk),
                std::forward_as_tuple());
            itr = newItr;
        }
        itr->second.emplace_back(vertID);
    }
    void deleteVertInBlock(GLuint vertID) {
        auto blk = computeBlock(vertID);
        auto &vertIDs = blkVertIDs.at(blk);
        auto itr = std::find(vertIDs.begin(), vertIDs.end(), vertID);
        vertIDs.erase(itr);
    }

  public:
    GLPathRenderer(const std::string &shaderDirPath, const glm::vec3 &bboxMax)
        : bboxMax(bboxMax) {
        glGenVertexArrays(1, &VAO);
        glBindVertexArray(VAO);
        glGenBuffers(1, &VBO);
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferStorage(GL_ARRAY_BUFFER, VERT_DAT_STRIDE * MAX_VERT_NUM,
                        nullptr, GL_DYNAMIC_STORAGE_BIT);
        VERTEX_ARRAY_DEF;
        glBindVertexArray(0);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        for (GLuint id = 0; id < MAX_PATH_NUM; ++id)
            availablePathIDs.emplace(id);
        for (GLuint id = 0; id < MAX_VERT_NUM; ++id)
            availableSubPathIDs.emplace(id);
        for (GLuint id = 0; id < MAX_VERT_NUM; ++id)
            availableVertIDs.emplace(id);
        verts.resize(MAX_VERT_NUM);
        pathIDOfVerts.resize(MAX_VERT_NUM);

        std::array<GLfloat, DIV_NUM> coss, sins;
        for (uint8_t idx = 0; idx < DIV_NUM; ++idx) {
            auto rad = glm::pi<float>() * 2.f * idx / DIV_NUM;
            coss[idx] = cosf(rad);
            sins[idx] = sinf(rad);
        }

        lineShader.shader =
            std::make_unique<Shader>((shaderDirPath + "/line.vs").c_str(),
                                     (shaderDirPath + "/diffuse.fs").c_str(),
                                     (shaderDirPath + "/line.gs").c_str());
        lineShader.MPos = glGetUniformLocation(lineShader.shader->ID, "M");
        lineShader.VPPos = glGetUniformLocation(lineShader.shader->ID, "VP");
        lineShader.widthPos =
            glGetUniformLocation(lineShader.shader->ID, "width");
        lineShader.colPos =
            glGetUniformLocation(lineShader.shader->ID, "color");

        lineShader.shader->use();
        glUniform1ui(glGetUniformLocation(lineShader.shader->ID, "divNum"),
                     DIV_NUM);
        glUniform1fv(glGetUniformLocation(lineShader.shader->ID, "coss"),
                     DIV_NUM, coss.data());
        glUniform1fv(glGetUniformLocation(lineShader.shader->ID, "sins"),
                     DIV_NUM, sins.data());

        vertShader.shader =
            std::make_unique<Shader>((shaderDirPath + "/vert.vs").c_str(),
                                     (shaderDirPath + "/diffuse.fs").c_str(),
                                     (shaderDirPath + "/vert.gs").c_str());
        vertShader.MPos = glGetUniformLocation(vertShader.shader->ID, "M");
        vertShader.VPPos = glGetUniformLocation(vertShader.shader->ID, "VP");
        vertShader.szPos = glGetUniformLocation(vertShader.shader->ID, "size");
        vertShader.colPos =
            glGetUniformLocation(vertShader.shader->ID, "color");

        vertShader.shader->use();
        glUniform1ui(glGetUniformLocation(vertShader.shader->ID, "divNum"),
                     DIV_NUM);
        glUniform1fv(glGetUniformLocation(vertShader.shader->ID, "coss"),
                     DIV_NUM, coss.data());
        glUniform1fv(glGetUniformLocation(vertShader.shader->ID, "sins"),
                     DIV_NUM, sins.data());

        lineDepShader.shader =
            std::make_unique<Shader>((shaderDirPath + "/line.vs").c_str(),
                                     (shaderDirPath + "/depth.fs").c_str(),
                                     (shaderDirPath + "/line.gs").c_str());
        lineDepShader.MPos =
            glGetUniformLocation(lineDepShader.shader->ID, "M");
        lineDepShader.VPPos =
            glGetUniformLocation(lineDepShader.shader->ID, "VP");
        lineDepShader.widthPos =
            glGetUniformLocation(lineDepShader.shader->ID, "width");
        lineDepShader.shader->use();
        glUniform1ui(glGetUniformLocation(lineDepShader.shader->ID, "divNum"),
                     DIV_NUM);
        glUniform1fv(glGetUniformLocation(lineDepShader.shader->ID, "coss"),
                     DIV_NUM, coss.data());
        glUniform1fv(glGetUniformLocation(lineDepShader.shader->ID, "sins"),
                     DIV_NUM, sins.data());

        vertDepShader.shader =
            std::make_unique<Shader>((shaderDirPath + "/vert.vs").c_str(),
                                     (shaderDirPath + "/depth.fs").c_str(),
                                     (shaderDirPath + "/vert.gs").c_str());
        vertDepShader.MPos =
            glGetUniformLocation(vertDepShader.shader->ID, "M");
        vertDepShader.VPPos =
            glGetUniformLocation(vertDepShader.shader->ID, "VP");
        vertDepShader.szPos =
            glGetUniformLocation(vertDepShader.shader->ID, "size");

        vertDepShader.shader->use();
        glUniform1ui(glGetUniformLocation(vertDepShader.shader->ID, "divNum"),
                     DIV_NUM);
        glUniform1fv(glGetUniformLocation(vertDepShader.shader->ID, "coss"),
                     DIV_NUM, coss.data());
        glUniform1fv(glGetUniformLocation(vertDepShader.shader->ID, "sins"),
                     DIV_NUM, sins.data());
    }

#undef VERTEX_ARRAY_DEF

    ~GLPathRenderer() {
        if (VBO != 0)
            glDeleteBuffers(1, &VBO);
        if (VAO != 0)
            glDeleteVertexArrays(1, &VAO);
    }
    GLPathRenderer(const GLPathRenderer &) = delete;
    GLPathRenderer(GLPathRenderer &&) = delete;
    GLPathRenderer &operator=(const GLPathRenderer &) = delete;
    GLPathRenderer &operator=(GLPathRenderer &&) = delete;
    void Clear() {
        std::vector<GLuint> needDelPathIDs;
        for (auto &[id, path] : paths)
            needDelPathIDs.emplace_back(id);
        for (GLuint id : needDelPathIDs)
            DeletePath(id);
    }
    inline GLuint GetSelectedPathID() const { return selectedPathID; }
    inline GLuint GetSelectedSubPathID() const { return selectedSubPathID; }
    inline GLuint GetSelectedVertID() const { return selectedVertID; }
    inline GLuint GetPathIDOf(GLuint vertID) const {
        return pathIDOfVerts[vertID];
    }
    inline GLuint GetRootIDOf(GLuint pathID) const {
        return paths.at(pathID).rootID;
    }
    inline const auto &GetPaths() const { return paths; }
    inline const auto &GetVertexPositions() const { return verts; }
    inline GLuint AddPath(const glm::vec3 &color, const glm::vec3 &rootPos) {
        GLuint pathID = availablePathIDs.front();
        GLuint rootID = availableVertIDs.front();
        availablePathIDs.pop();
        availableVertIDs.pop();

        pathIDOfVerts[rootID] = pathID;
        verts[rootID] = rootPos;
        addVertToBlock(rootID);

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, VERT_DAT_STRIDE * rootID,
                        VERT_DAT_STRIDE, &rootPos[0]);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        paths.emplace(std::piecewise_construct, std::forward_as_tuple(pathID),
                      std::forward_as_tuple(color, rootID));

        return pathID;
    }
    inline void DeletePath(GLuint pathID) {
        std::unordered_set<GLuint> recycledVertIDs; // de-repeat
        for (auto &[subPathID, subPath] : paths.at(pathID).subPaths) {
            for (GLuint vertID : subPath.verts)
                recycledVertIDs.emplace(vertID);
            availableSubPathIDs.emplace(subPathID);
        }

        for (GLuint vertID : recycledVertIDs)
            availableVertIDs.emplace(vertID);
        availablePathIDs.emplace(pathID);

        paths.erase(pathID);
    }
    inline void JoinPath() {
        GLuint outPathID = pathIDOfVerts[selectedVertID];
        GLuint inPathID = pathIDOfVerts[scndSelectedVertID];
        if (outPathID == inPathID || outPathID == NONE || inPathID == NONE)
            return;

        Path &inPath = paths.at(inPathID);
        Path &outPath = paths.at(outPathID);

        for (auto &[id, subPath] : inPath.subPaths)
            for (GLuint vertID : subPath.verts)
                pathIDOfVerts[vertID] = outPathID;

        for (auto &[subPathID, subPath] : inPath.subPaths)
            outPath.subPaths.emplace(std::piecewise_construct,
                                     std::forward_as_tuple(subPathID),
                                     std::forward_as_tuple(std::move(subPath)));

        DeletePath(inPathID);

        // append a sub path linking 2 paths
        StartPath(outPathID);
        StartVertex(selectedVertID);
        GLuint subPathID = AddSubPath();
        outPath.subPaths.at(subPathID).addVertex(scndSelectedVertID);
        EndPath();
    }
    inline void StartPath(GLuint pathID) {
        selectedPathID = pathID;
        selectedSubPathID = selectedVertID = scndSelectedVertID = NONE;
    }
    inline void EndPath() {
        selectedPathID = selectedVertID = selectedSubPathID =
            scndSelectedVertID = NONE;
    }
    inline GLuint AddSubPath() {
        Path &path = paths.at(selectedPathID);
        GLuint subPathID = availableSubPathIDs.front();
        availableSubPathIDs.pop();

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        path.addSubPath(subPathID,
                        selectedVertID == NONE ? path.rootID : selectedVertID);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
        return subPathID;
    }
    inline void DeleteSubPath(GLuint subPathID) {
        paths.at(selectedPathID).subPaths.erase(subPathID);
        availableSubPathIDs.emplace(subPathID);
    }
    inline void StartSubPath(GLuint subPathID) {
        selectedSubPathID = subPathID;
    }
    inline void EndSubPath() {
        selectedSubPathID = selectedVertID = scndSelectedVertID = NONE;
    }
    inline GLuint AddVertex(const glm::vec3 &pos) {
        GLuint vertID = availableVertIDs.front();
        availableVertIDs.pop();

        pathIDOfVerts[vertID] = selectedPathID;
        addVertToBlock(vertID);

        verts[vertID] = pos;
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, VERT_DAT_STRIDE * vertID,
                        VERT_DAT_STRIDE, &pos[0]);
        glBindBuffer(GL_ARRAY_BUFFER, 0);

        auto &subPath = paths.at(selectedPathID).subPaths.at(selectedSubPathID);
        subPath.addVertex(vertID);

        return vertID;
    }
    inline void MoveVertex(GLuint vertID, glm::vec3 &pos) {
        verts[vertID] = pos;

        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, VERT_DAT_STRIDE * vertID,
                        VERT_DAT_POS_SIZE, &pos);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
    inline void DeleteVertex(GLuint vertID) {
        GLuint linkCnt = 0;
        GLuint tarSPID;
        SubPath *tarSP = nullptr;

        auto &path = paths.at(pathIDOfVerts[vertID]);
        if (vertID == path.rootID)
            return;
        for (auto &[id, subPath] : path.subPaths)
            if (vertID == subPath.verts.front() ||
                vertID == subPath.verts.back()) {
                tarSPID = id;
                tarSP = &subPath;
                ++linkCnt;
            }
        if (linkCnt > 1)
            return; // vert doesn't exist or is not vert at the end
        tarSP->verts.pop_back();
        availableVertIDs.push(vertID);

        // if sub path is empty, delete it
        if (tarSP->verts.size() == 1)
            DeleteSubPath(tarSPID);

        // if path is empty, delete it
        if (path.subPaths.size() == 0) {
            DeletePath(selectedPathID);
            selectedPathID = NONE;
        }
    }
    inline GLuint GetEndVertexSubPathID(GLuint vertID) {
        auto &path = paths.at(pathIDOfVerts[vertID]);
        
        auto tarSPID = NONE;
        GLuint linkCnt = 0;
        if (vertID == path.rootID)
            return NONE;
        for (auto &[id, subPath] : path.subPaths)
            for (size_t i = 0; i < subPath.verts.size();++i)
                if (subPath.verts[i] == vertID) {
                    if (i == 0 || i == subPath.verts.size() - 1) {
                        tarSPID = id;
                        ++linkCnt;
                    } else
                        linkCnt = 2;

                    if (linkCnt > 1)
                        return NONE;
                }

        return tarSPID;
    }
    inline void StartVertex(GLuint vertID) { selectedVertID = vertID; }
    inline void StartSecondVertex(GLuint vertID) {
        scndSelectedVertID = vertID;
    }
    inline void EndVertex() { selectedVertID = NONE; }
    inline void EndSecondVertex() { scndSelectedVertID = NONE; }
    inline GLuint SearchNearest(const glm::vec3 &pos) {
        auto blk = computeBlock(pos);

        auto blkMin = decltype(blk){blk[0] == 0 ? blk[0] : blk[0] - 1,
                                      blk[1] == 0 ? blk[1] : blk[1] - 1,
                                      blk[2] == 0 ? blk[2] : blk[2] - 1};
        auto blkMax =
            decltype(blk){blk[0] == BLOCK_NUM - 1 ? blk[0] : blk[0] + 1,
                          blk[1] == BLOCK_NUM - 1 ? blk[1] : blk[1] + 1,
                          blk[2] == BLOCK_NUM - 1 ? blk[2] : blk[2] + 1};

        auto minDistID = NONE;
        auto minDistSqr = std::numeric_limits<float>::max();
        for (auto bz = blkMin[2]; bz <= blkMax[2];++bz)
            for (auto by = blkMin[1]; by <= blkMax[1]; ++by)
                for (auto bx = blkMin[0]; bx <= blkMax[0]; ++bx) {
                    blk = {bx, by, bz};
                    auto itr = blkVertIDs.find(blk);
                    if (itr == blkVertIDs.end())
                        continue;

                    for (auto vertID : itr->second) {
                        auto &vertPos = verts[vertID];
                        auto d = pos - vertPos;
                        auto ds = glm::dot(d, d);
                        if (minDistSqr > ds) {
                            minDistSqr = ds;
                            minDistID = vertID;
                        }
                    }
                }

        return minDistID;
    }
    inline void Draw(const glm::mat4 &M, const glm::mat4 &VP) {
        lineShader.shader->use();
        glUniformMatrix4fv(lineShader.MPos, 1, GL_FALSE, &M[0][0]);
        glUniformMatrix4fv(lineShader.VPPos, 1, GL_FALSE, &VP[0][0]);

        vertShader.shader->use();
        glUniformMatrix4fv(vertShader.MPos, 1, GL_FALSE, &M[0][0]);
        glUniformMatrix4fv(vertShader.VPPos, 1, GL_FALSE, &VP[0][0]);

        // draw selected vert
        if (selectedVertID != NONE) {
            glBindVertexArray(VAO);
            glUniform3fv(vertShader.colPos, 1, &selectedVertColor[0]);
            glUniform1f(vertShader.szPos, selectedVertSize * szScale);
            glDrawArrays(GL_POINTS, selectedVertID, 1);
        }

        for (auto &[id, path] : paths) {
            // draw lines
            lineShader.shader->use();
            glUniform3fv(lineShader.colPos, 1, &path.color[0]);
            glUniform1f(lineShader.widthPos,
                        (id == selectedPathID ? selectedLineWidth : lineWidth) *
                            szScale);

            for (auto &[id, subPath] : path.subPaths) {
                subPath.upload();
                subPath.drawLineStrips();
            }

            // draw end verts
            vertShader.shader->use();
            glUniform3fv(vertShader.colPos, 1, &path.color[0]);
            glUniform1f(vertShader.szPos, endVertSize * szScale);
            for (auto &[id, subPath] : path.subPaths)
                subPath.drawEndVerts();

            // draw root vert
            glBindVertexArray(VAO);
            glUniform1f(vertShader.szPos, rootVertSize * szScale);
            glDrawArrays(GL_POINTS, path.rootID, 1);
        }
        glBindVertexArray(0);
    }
    inline void DrawDepth(const glm::mat4 &M, const glm::mat4 &VP) {
        lineDepShader.shader->use();
        glUniformMatrix4fv(lineDepShader.MPos, 1, GL_FALSE, &M[0][0]);
        glUniformMatrix4fv(lineDepShader.VPPos, 1, GL_FALSE, &VP[0][0]);

        vertDepShader.shader->use();
        glUniformMatrix4fv(vertDepShader.MPos, 1, GL_FALSE, &M[0][0]);
        glUniformMatrix4fv(vertDepShader.VPPos, 1, GL_FALSE, &VP[0][0]);

        for (auto &[id, path] : paths) {
            // draw lines
            lineDepShader.shader->use();
            glUniform1f(lineDepShader.widthPos,
                        (id == selectedPathID ? selectedLineWidth : lineWidth) *
                            szScale);

            for (auto &[id, subPath] : path.subPaths) {
                subPath.upload();
                subPath.drawLineStrips();
            }

            // draw end verts
            vertDepShader.shader->use();
            glUniform1f(vertDepShader.szPos, endVertSize * szScale);
            for (auto &[id, subPath] : path.subPaths)
                subPath.drawEndVerts();

            // draw root vert
            glUniform1f(vertDepShader.szPos, rootVertSize * szScale);
            glBindVertexArray(VAO);
            glDrawArrays(GL_POINTS, path.rootID, 1);
        }
        glBindVertexArray(0);
    }
    inline void PrepareExtraVertex(const glm::vec3 &pos) {
        extraVertID = availableVertIDs.front(); // use a extra id
        glBindBuffer(GL_ARRAY_BUFFER, VBO);
        glBufferSubData(GL_ARRAY_BUFFER, VERT_DAT_STRIDE * extraVertID,
                        VERT_DAT_POS_SIZE, &pos);
        glBindBuffer(GL_ARRAY_BUFFER, 0);
    }
    inline void DrawExtraVertex(const glm::mat4 &M, const glm::mat4 &VP,
                                const glm::vec3 &color, float size) {
        vertShader.shader->use();
        glUniformMatrix4fv(vertShader.MPos, 1, GL_FALSE, &M[0][0]);
        glUniformMatrix4fv(vertShader.VPPos, 1, GL_FALSE, &VP[0][0]);

        glUniform3fv(vertShader.colPos, 1, &color[0]);
        glUniform1f(vertShader.szPos, size);
        glBindVertexArray(VAO);
        glDrawArrays(GL_POINTS, extraVertID, 1);
        glBindVertexArray(0);
    }
    inline void DrawExtraVertexDepth(const glm::mat4 &M, const glm::mat4 &VP,
                                     float size) {
        vertDepShader.shader->use();
        glUniformMatrix4fv(vertDepShader.MPos, 1, GL_FALSE, &M[0][0]);
        glUniformMatrix4fv(vertDepShader.VPPos, 1, GL_FALSE, &VP[0][0]);

        glUniform1f(vertDepShader.szPos, size);
        glBindVertexArray(VAO);
        glDrawArrays(GL_POINTS, extraVertID, 1);
        glBindVertexArray(0);
    }
};

} // namespace kouek

#endif // !KOUEK_GL_PATH_RENDERER_H
