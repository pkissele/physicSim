#version 330 core

in float vSpeed;
in float vSize;
in float vDens;
out vec4 fragColor;

uniform float massThreshold;
uniform float visibility;
uniform int passIndex;


void main() {
    // make points circular
    vec2 coord = 2.0 * gl_PointCoord - 1.0;
    if (dot(coord, coord) > 1.0) discard;

    bool isBig = vSize > massThreshold;

    if(passIndex == 0 &&  isBig) discard;
    if(passIndex == 1 && !isBig) discard;


    // float t = clamp(vSpeed * 0.1, 0.0, 1.0);
    float t = vDens;

    vec3 color;
    // white -> yellow -> red
    if (t < 0.5) {
        float tt = t / 0.5;
        color = mix(vec3(1.0,1.0,1.0), vec3(1.0,1.0,0.0), tt);
    } else {
        float tt = (t - 0.5) / 0.5;
        color = mix(vec3(1.0,1.0,0.0), vec3(1.0,0.0,0.0), tt);
    }

    if(isBig) {
        fragColor = vec4(color, 1);
    } else {
        fragColor = vec4(color, visibility);
    }
}
