#version 330 core

layout(location = 0) in vec2 in_pos;
layout(location = 1) in vec2 in_vel;

uniform float pointSize;

out float vSpeed;

void main() {
    // positions are passed in NDC coordinates already (x,y in [-1,1])
    gl_Position = vec4(in_pos, 0.0, 1.0);
    gl_PointSize = pointSize;

    vSpeed = length(in_vel);
}
