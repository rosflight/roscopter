#pragma once

#include <Eigen/Dense>
#include <cstdint>
#include <deque>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>
#include <unistd.h>

#define LOGGER_BUFFER_SIZE 1024 * 1024

namespace roscopter
{

class Logger
{
public:
  Logger() {}

  Logger(const std::string filename) { open(filename); }

  void open(const std::string & filename) { file_.open(filename); }

  ~Logger() { file_.close(); }
  template<typename... T>
  void log(T... data)
  {
    int dummy[sizeof...(data)] = {(file_.write((char *) &data, sizeof(T)), 1)...};
  }

  template<typename... T>
  void logVectors(T... data)
  {
    int dummy[sizeof...(data)] = {
      (file_.write((char *) data.data(), sizeof(typename T::Scalar) * data.rows() * data.cols()),
       1)...};
  }

  std::ofstream file_;
};

} // namespace roscopter
