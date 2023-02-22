#version 450 core

uniform float width;
uniform uint divNum;
uniform mat4 M;
uniform mat4 VP;
uniform float coss[32];
uniform float sins[32];

in VertDat {
	vec3 pos;
} gs_in[];

layout(lines) in;
layout(triangle_strip, max_vertices = 128) out;

out vec4 posInWdSp;
out vec4 normal;

void main() {
	vec3 axis;
    vec3 pos;
    vec3 delta = vec3(0, width, 0);
    mat3 rot;
    normal.w = 0;

    for (uint idx = 0; idx < divNum; ++idx) {
        float s = sins[idx];
        float c = coss[idx];
        float oc = 1.0 - c;
        axis = normalize(gs_in[1].pos - gs_in[0].pos);
        rot = mat3(oc * axis.x * axis.x + c, oc * axis.x * axis.y - axis.z * s,
                   oc * axis.z * axis.x + axis.y * s,
                   oc * axis.x * axis.y + axis.z * s, oc * axis.y * axis.y + c,
                   oc * axis.y * axis.z - axis.x * s,
                   oc * axis.z * axis.x - axis.y * s,
                   oc * axis.y * axis.z + axis.x * s, oc * axis.z * axis.z + c);
        pos = gs_in[0].pos + (normal.xyz = rot * delta);
        gl_Position = VP * (posInWdSp = M * vec4(pos, 1.0));
        normal = M * normal;
        EmitVertex();

        pos = gs_in[1].pos + (normal.xyz = rot * delta);
        gl_Position = VP * (posInWdSp = M * vec4(pos, 1.0));
        normal = M * normal;
        EmitVertex();
    }
    pos = gs_in[0].pos + (normal.xyz = delta);
    gl_Position = VP * (posInWdSp = M * vec4(pos, 1.0));
    normal = M * normal;
    EmitVertex();

    pos = gs_in[1].pos + (normal.xyz = delta);
    gl_Position = VP * (posInWdSp = M * vec4(pos, 1.0));
    normal = M * normal;    
    EmitVertex();

    EndPrimitive();
}
