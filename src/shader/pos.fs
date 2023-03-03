#version 450 core

uniform vec3 maxPos;

in vec3 posWdSp;
out vec4 outputColor;

void main() {
	outputColor = vec4(posWdSp / maxPos, 1.0);
}