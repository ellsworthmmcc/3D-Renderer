/*
* How to run for Windows Visual Studios:
* 1. mkdir build
* 2. cd build
* 3. cmake ..
* 4. cmake --build .
* 5. cd ..
* 6. build\Debug\tinyrenderer.exe obj\diablo3_pose\diablo3_pose.obj
* 
* Alternative:
* cmake -Bbuild && cmake --build build -j && build\Debug\tinyrenderer obj/african_head/african_head.obj
* cmake -Bbuild && cmake --build build -j && build\Debug\tinyrenderer obj/boggie/body.obj
* cmake -Bbuild && cmake --build build -j && build\Debug\tinyrenderer obj/diablo3_pose/diablo3_pose.obj
*/
#include <cmath>
#include <tuple>
#include "geometry.h"
#include "model.h"
#include "tgaimage.h"

constexpr int width = 800;
constexpr int height = 800;

constexpr TGAColor white = { 255, 255, 255, 255 }; // attention, BGRA order
constexpr TGAColor green = { 0, 255,   0, 255 };
constexpr TGAColor red = { 0,   0, 255, 255 };
constexpr TGAColor blue = { 255, 128,  64, 255 };
constexpr TGAColor yellow = { 0, 200, 255, 255 };


void line(int ax, int ay, int bx, int by, TGAImage& framebuffer, TGAColor color) {
  bool steep = std::abs(ax - bx) < std::abs(ay - by);
  if (steep) { // if the line is steep, we transpose the image
    std::swap(ax, ay);
    std::swap(bx, by);
  }
  if (ax > bx) { // make it left−to−right
    std::swap(ax, bx);
    std::swap(ay, by);
  }
  int y = ay;
  int ierror = 0;
  for (int x = ax; x <= bx; x++) {
    if (steep) // if transposed, de−transpose
      framebuffer.set(y, x, color);
    else
      framebuffer.set(x, y, color);
    ierror += 2 * std::abs(by - ay);
    y += (by > ay ? 1 : -1) * (ierror > bx - ax);
    ierror -= 2 * (bx - ax) * (ierror > bx - ax);
  }
}

int line_testing() {
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
      (uint8_t)(rand() % 255)
    });
  }

  framebuffer.write_tga_file("framebuffer.tga");
  return 0;
}

std::tuple<int, int> project(vec3 v) { // First of all, (x,y) is an orthogonal projection of the vector (x,y,z).
  return { (v.x + 1.) * width / 2,   // Second, since the input models are scaled to have fit in the [-1,1]^3 world coordinates,
           (v.y + 1.) * height / 2 }; // we want to shift the vector (x,y) and then scale it to span the entire screen.
}

int generate_wireframe(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " obj/model.obj" << std::endl;
    return 1;
  }

  Model model(argv[1]);
  TGAImage framebuffer(width, height, TGAImage::RGB);

  for (int i = 0; i < model.nfaces(); i++) { // iterate through all triangles
    auto [ax, ay] = project(model.vert(i, 0));
    auto [bx, by] = project(model.vert(i, 1));
    auto [cx, cy] = project(model.vert(i, 2));
    line(ax, ay, bx, by, framebuffer, red);
    line(bx, by, cx, cy, framebuffer, red);
    line(cx, cy, ax, ay, framebuffer, red);
  }

  for (int i = 0; i < model.nverts(); i++) { // iterate through all vertices
    vec3 v = model.vert(i);            // get i-th vertex
    auto [x, y] = project(v);          // project it to the screen
    framebuffer.set(x, y, white);
  }

  framebuffer.write_tga_file("framebuffer.tga");
  return 0;
}



void triangle_linear_interpolation(int ax, int ay, int bx, int by, int cx, int cy, TGAImage& framebuffer, TGAColor color) {
  // sort the vertices, a,b,c in ascending y order (bubblesort yay!)
  if (ay > by) { std::swap(ax, bx); std::swap(ay, by); }
  if (ay > cy) { std::swap(ax, cx); std::swap(ay, cy); }
  if (by > cy) { std::swap(bx, cx); std::swap(by, cy); }
  int total_height = cy - ay;

  if (ay != by) { // if the bottom half is not degenerate
    int segment_height = by - ay;
    for (int y = ay; y <= by; y++) { // sweep the horizontal line from ay to by
      int x1 = ax + ((cx - ax) * (y - ay)) / total_height;
      int x2 = ax + ((bx - ax) * (y - ay)) / segment_height;
      for (int x = std::min(x1, x2); x < std::max(x1, x2); x++)  // draw a horizontal line
        framebuffer.set(x, y, color);
    }
  }
  if (by != cy) { // if the upper half is not degenerate
    int segment_height = cy - by;
    for (int y = by; y <= cy; y++) { // sweep the horizontal line from by to cy
      int x1 = ax + ((cx - ax) * (y - ay)) / total_height;
      int x2 = bx + ((cx - bx) * (y - by)) / segment_height;
      for (int x = std::min(x1, x2); x < std::max(x1, x2); x++)  // draw a horizontal line
        framebuffer.set(x, y, color);
    }
  }
}


double signed_triangle_area(int ax, int ay, int bx, int by, int cx, int cy) {
  return .5 * ((by - ay) * (bx + ax) + (cy - by) * (cx + bx) + (ay - cy) * (ax + cx));
}

void triangle_bounding_box(int ax, int ay, int bx, int by, int cx, int cy, TGAImage& framebuffer, TGAColor color) {
  int bbminx = std::min(std::min(ax, bx), cx); // bounding box for the triangle
  int bbminy = std::min(std::min(ay, by), cy); // defined by its top left and bottom right corners
  int bbmaxx = std::max(std::max(ax, bx), cx);
  int bbmaxy = std::max(std::max(ay, by), cy);
  double total_area = signed_triangle_area(ax, ay, bx, by, cx, cy);
  if (total_area < 1) return; // backface culling + discarding triangles that cover less than a pixel

  #pragma omp parallel for
  for (int x = bbminx; x <= bbmaxx; x++) {
    for (int y = bbminy; y <= bbmaxy; y++) {
      double alpha = signed_triangle_area(x, y, bx, by, cx, cy) / total_area;
      double beta = signed_triangle_area(x, y, cx, cy, ax, ay) / total_area;
      double gamma = signed_triangle_area(x, y, ax, ay, bx, by) / total_area;
      if (alpha < 0 || beta < 0 || gamma < 0) continue; // negative barycentric coordinate => the pixel is outside the triangle
      framebuffer.set(x, y, color);
    }
  }
}

int triangle_testing() {
  TGAImage framebuffer(width, height, TGAImage::RGB);
  triangle_bounding_box(7, 45, 35, 100, 45, 60, framebuffer, red);
  triangle_bounding_box(120, 35, 90, 5, 45, 110, framebuffer, white);
  triangle_bounding_box(115, 83, 80, 90, 85, 120, framebuffer, green);
  framebuffer.write_tga_file("framebuffer.tga");
  return 0;
}

// TODO: Repeat code, fix later
int generate_triangle_rasterization(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " obj/model.obj" << std::endl;
    return 1;
  }

  Model model(argv[1]);
  TGAImage framebuffer(width, height, TGAImage::RGB);

  for (int i = 0; i < model.nfaces(); i++) { // iterate through all triangles
    auto [ax, ay] = project(model.vert(i, 0));
    auto [bx, by] = project(model.vert(i, 1));
    auto [cx, cy] = project(model.vert(i, 2));
    TGAColor rnd;
    for (int c = 0; c < 3; c++) rnd[c] = std::rand() % 255;
    triangle_bounding_box(ax, ay, bx, by, cx, cy, framebuffer, rnd);
  }

  framebuffer.write_tga_file("framebuffer.tga");
  return 0;
}

int main(int argc, char** argv) {
  // line_testing();
  // generate_wireframe(argc, argv);
  // triangle_testing();
  // generate_triangle_rasterization(argc, argv);
}