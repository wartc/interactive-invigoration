#include <iostream>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include "core/Camera.h"
#include "core/PlantGraph.h"
#include "core/Shader.h"
#include "core/Tree.h"

constexpr float ASPECT_RATIO = 16.0f / 9.0f;
constexpr unsigned int WINDOW_WIDTH = 1920, WINDOW_HEIGHT = WINDOW_WIDTH / ASPECT_RATIO;

// control
bool g_wireframeActive = false;
bool g_rotatingCamera = false;
bool g_panningCamera = false;

// camera controls
Camera camera({0.0f, 2.0f, 3.0f}, {0.0f, 3.0f, -1.0f});

bool g_firstMouse = true;
double g_lastX = 0.0, g_lastY = 0.0;
double g_panLastX = 0.0, g_panLastY = 0.0;

// time
float deltaTime = 0.0f;
float lastTime = 0.0f;

// callbacks
void framebufferSizeCallback(GLFWwindow* window, int width, int height);
void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
void mouseCallback(GLFWwindow* window, double xpos, double ypos);
void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);

//
void processInput(GLFWwindow* window);
void clear(GLFWwindow* window);

GLFWwindow* initWindow() {
  // initialize GLFW
  if (!glfwInit()) {
    std::cerr << "ERROR: Failed to init GLFW" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  GLFWwindow* win =
      glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Interactive Invigoration", nullptr, nullptr);

  if (!win) {
    std::cerr << "ERROR: Failed to open window" << std::endl;
    glfwTerminate();
    std::exit(EXIT_FAILURE);
  }

  glfwMakeContextCurrent(win);
  glfwSetFramebufferSizeCallback(win, framebufferSizeCallback);
  glfwSetKeyCallback(win, keyCallback);
  glfwSetCursorPosCallback(win, mouseCallback);
  glfwSetMouseButtonCallback(win, mouseButtonCallback);
  glfwSetScrollCallback(win, scrollCallback);

  // initialize OpenGL
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cerr << "ERROR: Failed to initialize GLAD" << std::endl;
    clear(win);
    std::exit(EXIT_FAILURE);
  }

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LINE_SMOOTH);
  // glLineWidth(5);

  glPointSize(5);

  // GLfloat lineWidthRange[2] = {0.0f, 0.0f};
  // glGetFloatv(GL_ALIASED_LINE_WIDTH_RANGE, lineWidthRange);

  return win;
}

int main(int argc, char** argv) {
  GLFWwindow* window = initWindow();

  PlantGraph pg({0.0f, 0.0f, 0.0f});

  int id1 = pg.addNode({0.0f, 2.0f, 0.0f}, 0);
  int id2 = pg.addNode({-0.5f, 2.8f, 0.4f}, id1);
  int id3 = pg.addNode({0.9f, 3.3f, -0.4f}, id1);
  int id4 = pg.addNode({1.0f, 4.8f, 0.4f}, id3);
  int id5 = pg.addNode({0.8f, 4.2f, -0.6f}, id3);

  Tree tree(pg);

  tree.computeStrandsPosition();
  tree.computeCrossSections();

  auto mesh = tree.generateMesh();

  tree.initializeStrandBuffers();

  Shader sh("shaders/basic.vert", "shaders/basic.frag");

  while (!glfwWindowShouldClose(window)) {
    // time calculation per frame
    float currentFrame = static_cast<float>(glfwGetTime());
    deltaTime = currentFrame - lastTime;
    lastTime = currentFrame;

    // input processing
    processInput(window);

    // rendering
    glClearColor(0.20f, 0.20f, 0.20f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    sh.use();

    sh.setMat4("projection", camera.getProjectionMatrix());
    sh.setMat4("view", camera.getViewMatrix());
    sh.setMat4("model", glm::mat4(1.0f));

    // sh.setVec4("color", {0.70f, 0.98f, 0.64f, 1.0f});
    // sh.setVec3("lightDir", glm::normalize(glm::vec3(0.5f, 1.0f, 0.3f)));
    // sh.setVec3("viewPos", camera.getPosition());

    mesh.render();

    // sh.setVec4("color", {1.0f, 0.0f, 0.0f, 1.0f});
    // tree.renderStrands(sh);

    // glfw processes
    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  clear(window);

  return 0;
}

void processInput(GLFWwindow* window) {
  if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
    camera.move(Camera::Direction::FORWARD, deltaTime);  //
  if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
    camera.move(Camera::Direction::BACKWARD, deltaTime);  //
  if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
    camera.move(Camera::Direction::LEFT, deltaTime);  //
  if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
    camera.move(Camera::Direction::RIGHT, deltaTime);  //
}

void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
  glViewport(0, 0, width, height);
}

void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
  if (action == GLFW_PRESS && key == GLFW_KEY_ESCAPE) {
    glfwSetWindowShouldClose(window, true);
  }

  if (action == GLFW_PRESS && key == GLFW_KEY_F) {
    if (g_wireframeActive)
      glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    else
      glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    g_wireframeActive = !g_wireframeActive;
  }
}

void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
  if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {
    g_panningCamera = true;
    glfwGetCursorPos(window, &g_panLastX, &g_panLastY);
  } else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_RELEASE) {
    g_panningCamera = false;
  } else if (button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_PRESS) {
    g_rotatingCamera = true;
    glfwGetCursorPos(window, &g_lastX, &g_lastY);
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);  // lock cursor
  } else if (button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_RELEASE) {
    g_rotatingCamera = false;
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);  // unlock cursor
  }
}

void mouseCallback(GLFWwindow* window, double xpos, double ypos) {
  if (g_rotatingCamera) {
    if (g_firstMouse) {
      g_lastX = xpos;
      g_lastY = ypos;
      g_firstMouse = false;
    }

    float dx = static_cast<float>(xpos - g_lastX);
    float dy = static_cast<float>(g_lastY - ypos);

    camera.rotate(dx, dy);

    g_lastX = xpos;
    g_lastY = ypos;
  } else if (g_panningCamera) {
    float dx = static_cast<float>(xpos - g_panLastX);
    float dy = static_cast<float>(ypos - g_panLastY);

    camera.pan(dx, dy);

    g_panLastX = xpos;
    g_panLastY = ypos;
  }
}

void scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
  camera.zoom(static_cast<float>(yoffset));
}

void clear(GLFWwindow* window) {
  glfwDestroyWindow(window);
  glfwTerminate();
}
