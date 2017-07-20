#ifndef DEBUG_H
#define DEBUG_H
#include <sys/time.h>
#include <stdio.h>
#include <unistd.h>
#include <map>
#include <string>
#include <iostream>
#include <fstream>

namespace svo
{

class Timer
{
private:
  timeval start_time_;
  double time_;
  double accumulated_;
public:

  /// The constructor directly starts the timer.
  Timer() :
    time_(0.0),
    accumulated_(0.0)
  {
    start();
  }

  ~Timer()
  {}

  inline void start()
  {
    accumulated_ = 0.0;
    gettimeofday(&start_time_, NULL);
  }

  inline void resume()
  {
    gettimeofday(&start_time_, NULL);
  }

  inline double stop()
  {
    timeval end_time;
    gettimeofday(&end_time, NULL);
    long seconds  = end_time.tv_sec  - start_time_.tv_sec;
    long useconds = end_time.tv_usec - start_time_.tv_usec;
    time_ = ((seconds) + useconds*0.000001) + accumulated_;
    accumulated_ = time_;
    return time_;
  }

  inline double getTime() const
  {
    return time_;
  }

  inline void reset()
  {
    time_ = 0.0;
    accumulated_ = 0.0;
  }

  static double getCurrentTime()
  {
    timeval time_now;
    gettimeofday(&time_now, NULL);
    return time_now.tv_sec + time_now.tv_usec*0.000001;
  }

  static double getCurrentSecond()
  {
    timeval time_now;
    gettimeofday(&time_now, NULL);
    return time_now.tv_sec;
  }

};

struct LogItem
{
  double data;
  bool   set;
};

class DeBuger
{
public:
  DeBuger();
  ~DeBuger();
  void init(const std::string& trace_name, const std::string& trace_dir);
  void addTimer(const std::string& name);
  void addLog(const std::string& name);
  void writeToFile();
  void startTimer(const std::string& name);
  void stopTimer(const std::string& name);
  double getTime(const std::string& name) const;
  void log(const std::string& name, double data);

private:
  std::map<std::string, Timer>      timers_;
  std::map<std::string, LogItem>    logs_;
  std::string                       trace_name_;        //<! name of the thread that started the performance monitor
  std::string                       trace_dir_;         //<! directory where the logfiles are saved
  std::ofstream                     ofs_;

  void trace();
  void traceHeader();
};
} // end namespace vk
#endif // DEBUG_H
