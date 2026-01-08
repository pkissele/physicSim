#version 330 core
in vec2 uv;
out vec4 fragColor;

void main() {
    // simple vertical gradient
    // vec3 bottom = vec3(0.0, 0.0, 0.0);
    // vec3 top = vec3(0.05, 0.05, 0.05);
    // fragColor = vec4(mix(bottom, top, uv.y), 1.0);
    vec3 bgColor = vec3(0.0, 0.0, 0.0);
    fragColor = vec4(bgColor, 1);
}
