#include "Flock.hpp"

#include <chrono>
#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#define MAX_STEPS -1
#define NUM_BOIDS 200

#define WINDOW_WIDTH 1000
#define WINDOW_HEIGHT 800

using namespace murmurations;

void UpdatePositions(Flock& f, float positions[]) {
  int i = 0;
  for (auto& b : f.boids) {
    // cheap normalization
    positions[i] = b.position.x() / (float)(WINDOW_WIDTH);
    positions[i+1] = b.position.y() / (float)(WINDOW_HEIGHT);
    i += 2;
  }
}

int main() {
  srand((unsigned int)time(0));
  std::cout << "Starting murmur program" << std::endl;

  // Setup
  Flock flock{NUM_BOIDS};

  if (!glfwInit())
    return -1;
  GLFWwindow* window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT,"Murmurations", NULL, NULL);
  if (window == NULL) {
    glfwTerminate();
    return -1;
  }
  glfwMakeContextCurrent(window);
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cout << "Failed to initialize GLAD" << std::endl;
    return -1;
  }
  glViewport(0, 0, WINDOW_WIDTH, WINDOW_HEIGHT);

  float positions[2 * NUM_BOIDS] = {};
  UpdatePositions(flock, positions);

  unsigned int vbo;
  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, 2 * NUM_BOIDS * sizeof(float), positions, GL_DYNAMIC_DRAW);

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), 0);
  
  // Loop
  int step = 0;
  while (!glfwWindowShouldClose(window)) {
    auto start = std::chrono::high_resolution_clock::now();
    std::cout << "------ STEP " << step << " ------" << std::endl;

    // flock.print();
    flock.flock();

    // map buffer object into client memory
    // alternative is to call glBindBuffer() followed by glBufferSubData()
    float* p_Buffer = (float*)glMapBuffer(GL_ARRAY_BUFFER, GL_WRITE_ONLY);
    if (p_Buffer) {
        UpdatePositions(flock, p_Buffer);
        glUnmapBuffer(GL_ARRAY_BUFFER);
    }

    glClear(GL_COLOR_BUFFER_BIT);
    glDrawArrays(GL_POINTS, 0, NUM_BOIDS);

    glfwSwapBuffers(window);
    glfwPollEvents();

    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Time taken by loop: " << duration.count()
              << " microseconds" << std::endl;

    if (step == MAX_STEPS)
      break;
    ++step;
  }

  glfwTerminate();
  return 0;
}
