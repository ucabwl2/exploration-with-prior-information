#pragma once

#include <algorithm>
#include <cmath>
#include <limits.h>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/time.h>
#include <time.h>
#include <vector>

namespace ndt_generic {

void getVectorMeanStdev(const std::vector<double> &v, double &mean,
                        double &stdev);

void getVectorMinMax(const std::vector<double> &v, double &min, double &max);

void getVectorQuartiles(const std::vector<double> &vec, double &q1,
                        double &median, double &q3);

void normalizeVector(std::vector<double> &v);

template <class T> std::string toString(const T &x) {
  std::ostringstream o;

  if (!(o << x))
    throw std::runtime_error("::toString()");

  return o.str();
}

template <class T> T fromString(const std::string &s) {
  T t;
  std::istringstream iss(s);
  iss >> t;
  return t;
}

std::string getVectorStatisticStr(const std::vector<double> &data);

std::string getVectorStatisticGnuplotStr(const std::vector<double> &data);

double getDoubleTime();

const std::string currentDateTimeString();

} // namespace
