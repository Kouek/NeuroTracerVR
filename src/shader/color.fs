#version 450 core

uniform vec3 color;

out vec4 outputColor;

void main() {
	outputColor = vec4(color, 1.0);
}