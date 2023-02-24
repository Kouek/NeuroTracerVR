#version 450 core

out vec4 outputColor;

void main() {
	outputColor = vec4(vec3(gl_FragCoord.z), 1.0);
}
