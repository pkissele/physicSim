#include "utils.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

#include <png.h>
#include <GLFW/glfw3.h>

using namespace std;

string load_file(const string& path) {
    ifstream in(path);
    if (!in.is_open()) {
        cerr << "Failed to open file: " << path << endl;
        exit(1);
    }
    stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

void save_framebuffer_png(const string& filename, int width, int height) {
    // Step 1: Read from framebuffer (bottom-left origin)
    vector<unsigned char> raw(width * height * 3);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadPixels(0, 0, width, height, GL_RGB, GL_UNSIGNED_BYTE, raw.data());

    // Step 2: Prepare PNG write structures
    FILE* fp = fopen(filename.c_str(), "wb");
    if (!fp) {
        cerr << "Failed to open " << filename << " for writing\n";
        return;
    }

    png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, nullptr, nullptr, nullptr);
    if (!png_ptr) {
        fclose(fp);
        cerr << "Failed to create PNG write struct\n";
        return;
    }

    png_infop info_ptr = png_create_info_struct(png_ptr);
    if (!info_ptr) {
        png_destroy_write_struct(&png_ptr, nullptr);
        fclose(fp);
        cerr << "Failed to create PNG info struct\n";
        return;
    }

    if (setjmp(png_jmpbuf(png_ptr))) {
        png_destroy_write_struct(&png_ptr, &info_ptr);
        fclose(fp);
        cerr << "PNG write error\n";
        return;
    }

    png_init_io(png_ptr, fp);

    // Output is RGB8
    png_set_IHDR(
        png_ptr, info_ptr,
        width, height,
        8,
        PNG_COLOR_TYPE_RGB,
        PNG_INTERLACE_NONE,
        PNG_COMPRESSION_TYPE_BASE,
        PNG_FILTER_TYPE_BASE
    );

    png_write_info(png_ptr, info_ptr);

    // Step 3: PNG expects top-left origin → flip rows
    vector<png_bytep> row_pointers(height);
    for (int y = 0; y < height; y++) {
        int flipped = height - 1 - y;
        row_pointers[y] = (png_bytep)&raw[flipped * width * 3];
    }

    // Step 4: Write PNG
    png_write_image(png_ptr, row_pointers.data());
    png_write_end(png_ptr, nullptr);

    png_destroy_write_struct(&png_ptr, &info_ptr);
    fclose(fp);

    cout << "Saved PNG: " << filename << "\n";
}