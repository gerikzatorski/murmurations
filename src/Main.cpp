#include "Flock.hpp"
#include "Shader.hpp"

#include <chrono>
#include <iostream>
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#define MAX_STEPS -1
#define NUM_BOIDS 200

#define WINDOW_WIDTH 1000
#define WINDOW_HEIGHT 800

using namespace murmurations;

void processInput(GLFWwindow *window);
void UpdatePositions(Flock& f, float positions[]);

int main() {
  srand((unsigned int)time(0));
  std::cout << "Starting murmur program" << std::endl;

  // Setup
  Flock flock{NUM_BOIDS};

  if (!glfwInit())
    return -1;
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
  
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

  // Scoped to ensure gl objects are deleted before termination
  {
    Shader shader("../res/shaders/Basic.shader");
    shader.Bind();
    shader.SetUniform4f("u_Color", 0.7f, 0.3f, 0.8f, 1.0f);
    
    float positions[2 * NUM_BOIDS] = {};
    UpdatePositions(flock, positions);
  
    unsigned int vbo, vao;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
  
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, 2 * NUM_BOIDS * sizeof(float), positions, GL_DYNAMIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), 0);
  
    // optional: we only have one buffer and vertex array
    // glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    
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
      glBindVertexArray(vao);
      glDrawArrays(GL_POINTS, 0, NUM_BOIDS);
  
      processInput(window);
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
  
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);

  }

  glfwTerminate();
  return 0;
}

void processInput(GLFWwindow *window) {
  // 
  if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
    glfwSetWindowShouldClose(window, true);
}

void UpdatePositions(Flock& f, float positions[]) {
  int i = 0;
  for (auto& b : f.boids) {
    // cheap normalization
    positions[i] = b.position.x() / (float)(WINDOW_WIDTH);
    positions[i+1] = b.position.y() / (float)(WINDOW_HEIGHT);
    i += 2;
  }
}
