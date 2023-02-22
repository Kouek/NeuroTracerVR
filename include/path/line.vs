#version 450 core

layout(location = 0) in vec3 position;

out VertDat {
	vec3 pos;
} vs_out;

void main() {
	vs_out.pos = position;
}