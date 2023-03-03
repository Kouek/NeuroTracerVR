#version 450 core

uniform mat4 M;
uniform mat4 matrix;

layout(location = 0) in vec3 position;

out vec3 posWdSp;

void main() {
	posWdSp = (M * vec4(position, 1.0)).xyz;
	gl_Position = matrix * vec4(position.xyz, 1.0);
}