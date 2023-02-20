#version 450 core

uniform sampler2D tex;

in vec2 texCoord;

out vec4 outputColor;

void main() { outputColor = texture(tex, texCoord); }
