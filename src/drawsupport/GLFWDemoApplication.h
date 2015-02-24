#ifndef GLFW_DEMO_APPLICATION_H
#define GLFW_DEMO_APPLICATION_H

#include "DemoApplication.h"

ATTRIBUTE_ALIGNED16(class) GLFWDemoApplication : public DemoApplication {
 public:
  BT_DECLARE_ALIGNED_ALLOCATOR();

  void specialKeyboard(int key, int x, int y);
  virtual void swapBuffers();
  virtual void updateModifierKeys();
};
#endif //GLFW_DEMO_APPLICATION_H
