#version 450 core

uniform uint divNum;
uniform mat4 M;
uniform mat4 VP;
uniform float size;
uniform float coss[32];
uniform float sins[32];

layout(points) in;
layout(triangle_strip, max_vertices = 256) out;

in VertDat {
    vec3 pos; 
} gs_in[];

out vec4 posInWdSp;
out vec4 normal;

void main() {
    vec3 cntrPos = gs_in[0].pos;
    uint pDivNum = divNum / 2 + 1;
    uint pTopIdx, pBotIdx;
    uint pStartIdx = divNum * 3 / 4;
    vec3 dlt = vec3(0, 0, 0);

    vec4 N0, N1;
    N0.w = N1.w = 0;
    vec4 pos0, posInWdSp0;

    // bottom strip
    dlt.x = +coss[pStartIdx] * coss[0];
    dlt.y = +sins[pStartIdx];
    dlt.z = -coss[pStartIdx] * sins[0];
    vec4 endPosInWdSp = M * vec4(cntrPos + size * dlt, 1.0);
    vec4 endPos = VP * endPosInWdSp;

    pTopIdx = 1 + pStartIdx;
    if (pTopIdx >= divNum)
        pTopIdx -= divNum;
    dlt.x = +coss[pTopIdx] * coss[0];
    dlt.y = +sins[pTopIdx];
    dlt.z = -coss[pTopIdx] * sins[0];
    posInWdSp0 = M * vec4(cntrPos + size * dlt, 1.0);
    pos0 = VP * posInWdSp0;
    N0.xyz = dlt;
    N0 = M * N0;

    for (uint yDiv = 1; yDiv < divNum; ++yDiv) {
        dlt.x = +coss[pTopIdx] * coss[yDiv];
        dlt.y = +sins[pTopIdx];
        dlt.z = -coss[pTopIdx] * sins[yDiv];
        N1.xyz = dlt;
        N1 = M * N1;

        posInWdSp = posInWdSp0;
        gl_Position = pos0;
        normal = N0;
        EmitVertex();

        posInWdSp = endPosInWdSp;
        gl_Position = endPos;
        normal = (N0 + N1) / 2;
        EmitVertex();

        posInWdSp = posInWdSp0 = M * vec4(cntrPos + size * dlt, 1.0);
        gl_Position = pos0 = VP * posInWdSp;
        normal = N0 = N1;
        EmitVertex();

        EndPrimitive();
    }

    dlt.x = +coss[pTopIdx] * coss[0];
    dlt.y = +sins[pTopIdx];
    dlt.z = -coss[pTopIdx] * sins[0];
    N1.xyz = dlt;
    N1 = M * N1;

    posInWdSp = posInWdSp0;
    gl_Position = pos0;
    normal = N0;
    EmitVertex();

    posInWdSp = endPosInWdSp;
    gl_Position = endPos;
    normal = (N0 + N1) / 2;
    EmitVertex();

    posInWdSp = M * vec4(cntrPos + size * dlt, 1.0);
    gl_Position = VP * posInWdSp;
    normal = N1;
    EmitVertex();

    EndPrimitive();

    // p for pitch and y for yaw
    for (uint pDiv = 1; pDiv < pDivNum - 2; ++pDiv) {
        pTopIdx = pDiv + 1 + pStartIdx;
        if (pTopIdx >= divNum)
            pTopIdx -= divNum;
        pBotIdx = pDiv + pStartIdx;
        if (pBotIdx >= divNum)
            pBotIdx -= divNum;

        for (uint yDiv = 0; yDiv < divNum; ++yDiv) {
            dlt.x = +coss[pTopIdx] * coss[yDiv];
            dlt.y = +sins[pTopIdx];
            dlt.z = -coss[pTopIdx] * sins[yDiv];
            N1.xyz = dlt;
            N1 = M * N1;
            posInWdSp = M * vec4(cntrPos + size * dlt, 1.0);
            gl_Position = VP * posInWdSp;
            normal = N1;
            EmitVertex();

            dlt.x = +coss[pBotIdx] * coss[yDiv];
            dlt.y = +sins[pBotIdx];
            dlt.z = -coss[pBotIdx] * sins[yDiv];
            N1.xyz = dlt;
            N1 = M * N1;
            posInWdSp = M * vec4(cntrPos + size * dlt, 1.0);
            gl_Position = VP * posInWdSp;
            normal = N1;
            EmitVertex();
        }
        dlt.x = +coss[pTopIdx] * coss[0];
        dlt.y = +sins[pTopIdx];
        dlt.z = -coss[pTopIdx] * sins[0];
        N1.xyz = dlt;
        N1 = M * N1;
        posInWdSp = M * vec4(cntrPos + size * dlt, 1.0);
        gl_Position = VP * posInWdSp;
        normal = N1;
        EmitVertex();

        dlt.x = +coss[pBotIdx] * coss[0];
        dlt.y = +sins[pBotIdx];
        dlt.z = -coss[pBotIdx] * sins[0];
        N1.xyz = dlt;
        N1 = M * N1;
        posInWdSp = M * vec4(cntrPos + size * dlt, 1.0);
        gl_Position = VP * posInWdSp;
        normal = N1;
        EmitVertex();

        EndPrimitive();
    }

    // top strip
    pTopIdx = pStartIdx + pDivNum - 1;
    pBotIdx = pTopIdx - 1;
    if (pTopIdx >= divNum)
        pTopIdx -= divNum;
    if (pBotIdx >= divNum)
        pBotIdx -= divNum;

    dlt.x = +coss[pTopIdx] * coss[0];
    dlt.y = +sins[pTopIdx];
    dlt.z = -coss[pTopIdx] * sins[0];
    endPosInWdSp = M * vec4(cntrPos + size * dlt, 1.0);
    endPos = VP * endPosInWdSp;

    dlt.x = +coss[pBotIdx] * coss[0];
    dlt.y = +sins[pBotIdx];
    dlt.z = -coss[pBotIdx] * sins[0];
    posInWdSp0 = M * vec4(cntrPos + size * dlt, 1.0);
    pos0 = VP * posInWdSp0;
    N0.xyz = dlt;
    N0 = M * N0;

    for (uint yDiv = 1; yDiv < divNum; ++yDiv) {
        dlt.x = +coss[pBotIdx] * coss[yDiv];
        dlt.y = +sins[pBotIdx];
        dlt.z = -coss[pBotIdx] * sins[yDiv];
        N1.xyz = dlt;
        N1 = M * N1;

        posInWdSp = endPosInWdSp;
        gl_Position = endPos;
        normal = (N0 + N1) / 2;
        EmitVertex();

        posInWdSp = posInWdSp0;
        gl_Position = pos0;
        normal = N0;
        EmitVertex();

        posInWdSp = posInWdSp0 = M * vec4(cntrPos + size * dlt, 1.0);
        gl_Position = pos0 = VP * posInWdSp;
        normal = N0 = N1;
        EmitVertex();

        EndPrimitive();
    }

    dlt.x = +coss[pBotIdx] * coss[0];
    dlt.y = +sins[pBotIdx];
    dlt.z = -coss[pBotIdx] * sins[0];
    N1.xyz = dlt;
    N1 = M * N1;

    posInWdSp = endPosInWdSp;
    gl_Position = endPos;
    normal = (N0 + N1) / 2;
    EmitVertex();

    posInWdSp = posInWdSp0;
    gl_Position = pos0;
    normal = N0;
    EmitVertex();

    posInWdSp = M * vec4(cntrPos + size * dlt, 1.0);
    gl_Position = VP * posInWdSp;
    normal = N1;
    EmitVertex();

    EndPrimitive();
}