/*
 * DeBuger.cpp
 *
 *  Created on: Aug 26, 2011
 *      Author: Christian Forster
 */

#include <svo/debug.h>
#include <stdio.h>
#include <stdexcept>

namespace svo
{
using namespace std;

DeBuger::DeBuger()
{}

DeBuger::~DeBuger()
{
  ofs_.flush();
  ofs_.close();
}

void DeBuger::init(
    const string& trace_name,
    const string& trace_dir)
{
  trace_name_ = trace_name;
  trace_dir_ = trace_dir;
  string filename(trace_dir + "/" + trace_name + ".csv");
  ofs_.open(filename.c_str());
  if(!ofs_.is_open())
  {
    printf("Tracefile = %s\n", filename.c_str());
    throw runtime_error("Could not open tracefile.");
  }
  traceHeader();
}

void DeBuger::addTimer(const string& name)
{
  timers_.insert(make_pair(name, Timer()));
}

void DeBuger::addLog(const string& name)
{
  logs_.insert(make_pair(name, LogItem()));
}

void DeBuger::writeToFile()
{
  trace();

  for(auto it = timers_.begin(); it!=timers_.end(); ++it)
    it->second.reset();
  for(auto it=logs_.begin(); it!=logs_.end(); ++it)
  {
    it->second.set = false;
    it->second.data = -1;
  }
}

void DeBuger::startTimer(const string& name)
{
  auto t = timers_.find(name);
  if(t == timers_.end()) {
    printf("Timer = %s\n", name.c_str());
    throw std::runtime_error("startTimer: Timer not registered");
  }
  t->second.start();
}

void DeBuger::stopTimer(const string& name)
{
  auto t = timers_.find(name);
  if(t == timers_.end()) {
    printf("Timer = %s\n", name.c_str());
    throw std::runtime_error("stopTimer: Timer not registered");
  }
  t->second.stop();
}

double DeBuger::getTime(const string& name) const
{
  auto t = timers_.find(name);
  if(t == timers_.end()) {
    printf("Timer = %s\n", name.c_str());
    throw std::runtime_error("Timer not registered");
  }
  return t->second.getTime();
}

void DeBuger::log(const string& name, double data)
{
  auto l = logs_.find(name);
  if(l == logs_.end()) {
    printf("Logger = %s\n", name.c_str());
    throw std::runtime_error("Logger not registered");
  }
  l->second.data = data;
  l->second.set = true;
}

void DeBuger::trace()
{
  char buffer[128];
  bool first_value = true;
  if(!ofs_.is_open())
    throw std::runtime_error("Performance monitor not correctly initialized");
  ofs_.precision(15);
  ofs_.setf(std::ios::fixed, std::ios::floatfield );
  for(auto it = timers_.begin(); it!=timers_.end(); ++it)
  {
    if(first_value) {
      ofs_ << it->second.getTime();
      first_value = false;
    }
    else
      ofs_ << "," << it->second.getTime();
  }
  for(auto it=logs_.begin(); it!=logs_.end(); ++it)
  {
    if(first_value) {
      ofs_ << it->second.data;
      first_value = false;
    }
    else
      ofs_ << "," << it->second.data;
  }
  ofs_ << "\n";
}

void DeBuger::traceHeader()
{
  if(!ofs_.is_open())
    throw std::runtime_error("Performance monitor not correctly initialized");
  bool first_value = true;
  for(auto it = timers_.begin(); it!=timers_.end(); ++it)
  {
    if(first_value) {
      ofs_ << it->first;
      first_value = false;
    }
    else
      ofs_ << "," << it->first;
  }
  for(auto it=logs_.begin(); it!=logs_.end(); ++it)
  {
    if(first_value) {
      ofs_ << it->first;
      first_value = false;
    }
    else
      ofs_ << "," << it->first;
  }
  ofs_ << "\n";
}

} // namespace vk
