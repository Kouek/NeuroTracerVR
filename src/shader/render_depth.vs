#version 450 core

uniform mat4 matrix;

layout(location = 0) in vec3 position;

void main() {
	gl_Position = matrix * vec4(position.xyz, 1.0);
}