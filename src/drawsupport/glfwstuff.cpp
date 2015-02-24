#include <GLFW/glfw3.h>
#include <stdio.h>
#include <chrono>
#include <thread>

static void glfwErrorCallback(int error, const char *description) {
  fprintf(stderr, "ERROR %i: %s\n", error, description);
}

#include "DemoApplication.h"
#include "GLFWDemoApplication.h"

// glfw is C code, this global gDemoApplication links glfw to the C++ demo
static DemoApplication* gDemoApplication = nullptr;
static GLFWwindow *gWindow = nullptr;

static void glfwMouseButtonCallback(GLFWwindow *window, int button, int action, int mods) {
  double _x, _y;
  glfwGetCursorPos(window, &_x, &_y);
  int x(_x), y(_y);

  int &m_modifierKeys = gDemoApplication->m_modifierKeys;
  m_modifierKeys = 0;
  if (mods & GLFW_MOD_ALT) m_modifierKeys |= BT_ACTIVE_ALT;
  if (mods & GLFW_MOD_CONTROL) m_modifierKeys |= BT_ACTIVE_CTRL;
  if (mods & GLFW_MOD_SHIFT) m_modifierKeys |= BT_ACTIVE_SHIFT;

  gDemoApplication->mouseFunc(button, action, x, y);
}

static void glfwCursorPosCallback(GLFWwindow *window, double xoffset, double yoffset) {
}

static void glfwCursorEnterCallback(GLFWwindow *window, int entered) {
}

static void glfwScrollCallback(GLFWwindow *window, double xoffset, double yoffset) {
}

static void glfwKeyCallback(GLFWwindow *window, int key, int scancode, int action, int mods) {
  double _x, _y;
  glfwGetCursorPos(window, &_x, &_y);
  int x(_x), y(_y);

  int &m_modifierKeys = gDemoApplication->m_modifierKeys;
  m_modifierKeys = 0;
  if (mods & GLFW_MOD_ALT) m_modifierKeys |= BT_ACTIVE_ALT;
  if (mods & GLFW_MOD_CONTROL) m_modifierKeys |= BT_ACTIVE_CTRL;
  if (mods & GLFW_MOD_SHIFT) m_modifierKeys |= BT_ACTIVE_SHIFT;

  gDemoApplication->specialKeyboard(key, x, y);
}

//static void glfwCharCallback(GLFWwindow *window, unsigned int codepoint) {
//}

static void glfwCharModsCallback(GLFWwindow *window, unsigned int codepoint, int mods) {
  double _x, _y;
  glfwGetCursorPos(window, &_x, &_y);
  int x(_x), y(_y);

  int &m_modifierKeys = gDemoApplication->m_modifierKeys;
  m_modifierKeys = 0;
  if (mods & GLFW_MOD_ALT) m_modifierKeys |= BT_ACTIVE_ALT;
  if (mods & GLFW_MOD_CONTROL) m_modifierKeys |= BT_ACTIVE_CTRL;
  if (mods & GLFW_MOD_SHIFT) m_modifierKeys |= BT_ACTIVE_SHIFT;

  gDemoApplication->keyboardCallback(codepoint, x, y);
  //switch (action) {
  //  case GLFW_PRESS:
  //    gDemoApplication->keyboardCallback(key, x, y);
  //    break;
  //  case GLFW_RELEASE:
  //    gDemoApplication->keyboardUpCallback(key, x, y);
  //    break;
  //  case GLFW_REPEAT:
  //  default:
  //    break;
  //}
}

static void glfwDropCallback(GLFWwindow *window, int count, const char **names) {
}

static void glfwWindowRefreshCallback(GLFWwindow *window) {
  int w, h;
  glfwGetFramebufferSize(window, &w, &h);
  gDemoApplication->reshape(w, h);
  gDemoApplication->moveAndDisplay();
  gDemoApplication->displayCallback();
}

static bool isActive = false;
static void glfwWindowFocusCallback(GLFWwindow *window, int focus) {
  isActive = (focus == GL_TRUE);
}

int glfwmain(int argc, char **argv, int width, int height, const char* title, DemoApplication* demoApp) {
  if (demoApp == nullptr)
    return EXIT_FAILURE;
  gDemoApplication = demoApp;

  if (!glfwInit())
    return EXIT_FAILURE;

  // basics
  glfwSetErrorCallback(glfwErrorCallback);
  glfwWindowHint(GLFW_SAMPLES, 4);

  GLFWwindow *window = glfwCreateWindow(width, height, title, NULL, NULL);
  if (!window) {
    glfwTerminate();
    return EXIT_FAILURE;
  }
  gWindow = window;

  gDemoApplication->myinit();

  // callbacks
  glfwSetKeyCallback(window, glfwKeyCallback);
  //glfwSetCharCallback(window, glfwCharCallback);
  glfwSetCharModsCallback(window, glfwCharModsCallback);
  glfwSetMouseButtonCallback(window, glfwMouseButtonCallback);
  glfwSetCursorPosCallback(window, glfwCursorPosCallback);
  glfwSetCursorEnterCallback(window, glfwCursorEnterCallback);
  glfwSetScrollCallback(window, glfwScrollCallback);
  glfwSetDropCallback(window, glfwDropCallback);
  glfwSetWindowRefreshCallback(window, glfwWindowRefreshCallback);
  glfwSetWindowFocusCallback(window, glfwWindowFocusCallback);

  // more options
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  // main loop
  while (!glfwWindowShouldClose(window)) {
    glfwWindowRefreshCallback(window);
    if (!isActive) { // if running in background idle avoid high cpu usage,
      // empirical parameter
      std::this_thread::sleep_for(std::chrono::milliseconds(96));
    }

    glfwPollEvents();
  }

  glfwDestroyWindow(window);
  return EXIT_SUCCESS;
}

#include "BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h"
#include "BulletDynamics/Dynamics/btRigidBody.h"

void GLFWDemoApplication::updateModifierKeys() {}

void GLFWDemoApplication::specialKeyboard(int key, int x, int y) {
  (void)x;
  (void)y;

  switch (key) {
    case GLFW_KEY_F1: {
      break;
    }
    case GLFW_KEY_F2: {
      break;
    }
    case GLFW_KEY_END: {
      int numObj = getDynamicsWorld()->getNumCollisionObjects();
      if (numObj) {
        btCollisionObject* obj = getDynamicsWorld()->getCollisionObjectArray()[numObj-1];

        getDynamicsWorld()->removeCollisionObject(obj);
        btRigidBody* body = btRigidBody::upcast(obj);
        if (body && body->getMotionState()) {
          delete body->getMotionState();
        }
        delete obj;
      }
      break;
    }
    case GLFW_KEY_LEFT : stepLeft(); break;
    case GLFW_KEY_RIGHT : stepRight(); break;
    case GLFW_KEY_UP : stepFront(); break;
    case GLFW_KEY_DOWN : stepBack(); break;
    case GLFW_KEY_PAGE_UP : zoomIn(); break;
    case GLFW_KEY_PAGE_DOWN : zoomOut(); break;
    case GLFW_KEY_HOME : toggleIdle(); break;
    default:
     //        std::cout << "unused (special) key : " << key << std::endl;
     break;
  }
}

void GLFWDemoApplication::swapBuffers() {
  glfwSwapBuffers(gWindow);
}
