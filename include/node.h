#pragma once

struct Node {
    float comx = 0.0f;
    float comy = 0.0f;
    float size = 0.0f;
    float mass = 0.0f;

    int next = -1;
    int firstChild = -1;

    int lo = 0;
    int hi = 0;
};

struct NodeCold {
    float cx = 0.0f;
    float cy = 0.0f;
};
