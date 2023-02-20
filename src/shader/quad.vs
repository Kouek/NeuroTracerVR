#version 450 core

layout(location = 0) in vec3 posIn;
layout(location = 1) in vec2 texCoordIn;

out vec2 texCoord;

void main() {
	gl_Position = vec4(posIn, 1.0);
    texCoord = texCoordIn;
}
