#version 450 core

uniform vec3 color;

in vec4 posInWdSp;
in vec4 normal;

out vec4 fragColor;

void main() {
	fragColor = vec4(color, 1.0);
}
