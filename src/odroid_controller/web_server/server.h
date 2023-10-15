#include "libwebsockets.h"
#include "src/odroid_controller/controller.h"

#pragma once

class WebServer {
 public:
  void Tick();
  int Init(const Controller *controller);

 private:
  struct lws_context *context_;
  const Controller *controller_;

  std::unique_ptr<std::thread> communication_thread_;
};