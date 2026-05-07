#version 330 core

in float vSpeed;
in float vSize;
in float vDens;
out vec4 fragColor;

uniform float massThreshold;
uniform float visibility;

void main() {
    // make points circular
    vec2 coord = 2.0 * gl_PointCoord - 1.0;
    if (dot(coord, coord) > 1.0) discard;

    // map speed to 0..1
    // float t = clamp(vSpeed * 0.1, 0.0, 1.0);
    // float t = clamp(100*vDens, 0.0, 1.0);
    float t = vDens;

    vec3 color;
    if (t < 0.5) {
        // white → yellow
        float tt = t / 0.5;  // normalize to 0..1
        color = mix(vec3(1.0,1.0,1.0), vec3(1.0,1.0,0.0), tt);
    } else {
        // yellow → red
        float tt = (t - 0.5) / 0.5; // normalize to 0..1
        color = mix(vec3(1.0,1.0,0.0), vec3(1.0,0.0,0.0), tt);
    }

    if(vSize > massThreshold) {
        fragColor = vec4(color, 1);
    } else {
        fragColor = vec4(color, visibility);
    }
}
