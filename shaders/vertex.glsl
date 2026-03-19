#version 330 core

layout(location = 0) in vec2 in_pos;
layout(location = 1) in vec2 in_vel;
layout(location = 2) in float in_size;


// uniform float pointSize;
uniform vec2 displaySize;
uniform vec2 displayOffset;

out float vSpeed;
out float vSize;

void main() {
    vec2 pos = in_pos - displayOffset;
    vec2 ndc = (pos / displaySize) * 2.0 - 1.0;

    gl_Position = vec4(ndc, 0.0, 1.0);
    gl_PointSize = in_size;

    vSpeed = length(in_vel);
    vSize = in_size;
}
