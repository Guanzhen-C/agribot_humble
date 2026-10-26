#include "gsplat_rviz_plugin/perf_monitor.hpp"
namespace gsplat_rviz_plugin {
PerfMonitor &PerfMonitor::instance(){static PerfMonitor instance;return instance;}
void PerfMonitor::recordFrame(){}
void PerfMonitor::startTimer(const std::string&){}
void PerfMonitor::stopTimer(const std::string&){}
void PerfMonitor::setBufferStats(uint32_t,std::size_t,std::size_t){}
void PerfMonitor::logAll(){}
}
