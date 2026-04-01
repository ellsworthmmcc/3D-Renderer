/*
To compile on windows:
cmake -Bbuild && cmake --build build -j && build\Debug\tinyrenderer obj/diablo3_pose/diablo3_pose.obj obj/floor.obj
*/
#include <cmath>
#include <cstdlib>
#include <ctime>
#include "tgaimage.h"

constexpr TGAColor white   = {255, 255, 255, 255}; // attention, BGRA order
constexpr TGAColor green   = {  0, 255,   0, 255};
constexpr TGAColor red     = {  0,   0, 255, 255};
constexpr TGAColor blue    = {255, 128,  64, 255};
constexpr TGAColor yellow  = {  0, 200, 255, 255};

void line(int ax, int ay, int bx, int by, TGAImage& framebuffer, TGAColor color) {
  bool steep = std::abs(ax - bx) < std::abs(ay - by);
  if (steep) { // if line is steep, tranpose the image
    std::swap(ax, ay);
    std::swap(bx, by);
  }

  if (ax > bx) { // make it left-to-right
    std::swap(ax, bx);
    std::swap(ay, by);
  }
  
  int y = ay;
  int ierror = 0;
  for (int x = ax; x <= bx; x++) {
    float t = (x - ax) / static_cast<float>(bx - ax);
    if (steep)
      framebuffer.set(y, x, color);
    else
      framebuffer.set(x, y, color);
    ierror += 2 * std::abs(by - ay);
    y += (by > ay ? 1 : -1) * (ierror > bx - ax);
    ierror -= 2 * (bx - ax) * (ierror > bx - ax);
  }
}

int base() {
  constexpr int width = 64;
  constexpr int height = 64;
  TGAImage framebuffer(width, height, TGAImage::RGB);

  int ax = 7, ay = 3;
  int bx = 12, by = 37;
  int cx = 62, cy = 53;

  line(ax, ay, bx, by, framebuffer, blue);
  line(cx, cy, bx, by, framebuffer, green);
  line(cx, cy, ax, ay, framebuffer, yellow);
  line(ax, ay, cx, cy, framebuffer, red);

  framebuffer.set(ax, ay, white);
  framebuffer.set(bx, by, white);
  framebuffer.set(cx, cy, white);

  framebuffer.write_tga_file("framebuffer.tga");
  return 0;
}

int optimization() {
  constexpr int width = 64;
  constexpr int height = 64;
  TGAImage framebuffer(width, height, TGAImage::RGB);

  std::srand(std::time({}));
  for (int i = 0; i < (1 << 24); i++) {
    int ax = rand() % width, ay = rand() % height;
    int bx = rand() % width, by = rand() % height;
    line(ax, ay, bx, by, framebuffer, { 
      (uint8_t)(rand() % 255),
      (uint8_t)(rand() % 255),
      (uint8_t)(rand() % 255),
      (uint8_t)(rand() % 255 )
    });
  }

  framebuffer.write_tga_file("framebuffer.tga");
  return 0;
}

int main(int argc, char** argv) {
  // return base();
  return optimization();
}

