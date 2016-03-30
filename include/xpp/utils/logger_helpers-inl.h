/**
@file    logger_helpers-inl.h
@author  Alexander Winkler (winklera@ethz.ch)
@date    Oct 21, 2014
@brief   Specific helper functions for the  logging facility "Apache log4cxx".
 */

#ifndef _XPP_UTILS_LOGGER_HELPERS_INL_H_
#define _XPP_UTILS_LOGGER_HELPERS_INL_H_

#include <xpp/utils/geometric_structs.h>
#include <xpp/zmp/zmp_spline.h>
#include <log4cxx/logger.h>

#include <iomanip> // std::setprecision, std::fixed
#include "../hyq/support_polygon.h"

namespace xpp {
namespace utils {
namespace logger_helpers {

typedef std::vector<xpp::hyq::SupportPolygon> SuppTriangles;


// make inline to avoid function call overhead when logging level disabled
inline void print_spline_info(const std::vector<zmp::ZmpSpline>& splines, log4cxx::LoggerPtr log);

inline void print_struct(const utils::MatVec& s, std::string name, log4cxx::LoggerPtr log);

inline void print_opt_result(Eigen::VectorXd solution, double cost, clock_t start,
                             clock_t stop, log4cxx::LoggerPtr log);

inline void print_triangles(const SuppTriangles& tr, log4cxx::LoggerPtr log);

inline void PrintTriaglesMatlabInfo(const SuppTriangles& tr, log4cxx::LoggerPtr log);


/// definitions
void print_opt_result(Eigen::VectorXd solution, double cost, clock_t start,
                    clock_t stop, log4cxx::LoggerPtr log)
{
  if (log->isInfoEnabled()) {
    double time = double(stop - start) / CLOCKS_PER_SEC * 1000.0;
    std::stringstream ss;
    ss << std::setprecision(1) << std::fixed << '\n'
       << "solved qp-problem in " << time << "ms, "
       << "cost: " << cost
       << "\nx = " << std::setprecision(4) << solution.transpose();
    LOG4CXX_INFO(log, ss.str());
  }
}

void print_spline_info(const std::vector<zmp::ZmpSpline>& splines, log4cxx::LoggerPtr log)
{
  if (log->isDebugEnabled()) {
    int nr = 0;
    std::stringstream ss;
    ss << std::setprecision(2) << std::fixed << '\n';
    for (zmp::ZmpSpline s : splines) {
      ss << "Spline: id= " << nr++ << ":\t"
         << "duration=" << s.duration_<< "\t"
         << "four_leg_supp=" << s.four_leg_supp_ << "\t"
         << "step=" << s.step_ << "\n";
    }
    LOG4CXX_DEBUG(log,ss.str());
  }
}

void print_struct(const utils::MatVec& s, std::string name,
    log4cxx::LoggerPtr log)
{
  if (log->isTraceEnabled()) {
    std::stringstream ss;
    ss << std::setprecision(2) //<< std::fixed
       << name << ":\n"
       << "M: " << s.M.rows() << "x" << s.M.cols() << "\n"
       << s.M   << "\n"
       << "v: " << s.v.rows() << "\n"
       << s.v.transpose() << "\n";
    LOG4CXX_TRACE(log,ss.str());
  }
}

void print_triangles(const SuppTriangles& tr, log4cxx::LoggerPtr log)
{
  if (log->isDebugEnabled()) {
    std::stringstream ss;
    ss << std::setprecision(2) << std::fixed << '\n';
    int tc = 0;
    for (const hyq::SupportPolygon& t : tr)
      ss << "Triangle " << tc++ << ":\n" << t << "\n";
    LOG4CXX_TRACE(log, ss.str());
  }
}

void PrintTriaglesMatlabInfo(const SuppTriangles& tr, log4cxx::LoggerPtr log)
{
  if (log->isInfoEnabled()) {
    std::stringstream ss_x_values, ss_y_values;
    ss_x_values << std::setprecision(3) << std::fixed;
    ss_y_values << std::setprecision(3) << std::fixed;
    for (const hyq::SupportPolygon& t : tr) {
      ss_x_values << t.GetFootholds()[0].p(X) << " "
                  << t.GetFootholds()[1].p(X) << " "
                  << t.GetFootholds()[2].p(X) << " ";

      ss_y_values << t.GetFootholds()[0].p(Y) << " "
                  << t.GetFootholds()[1].p(Y) << " "
                  << t.GetFootholds()[2].p(Y) << " ";
    }
    LOG4CXX_INFO(log, ss_x_values.str() << "\n" << ss_y_values.str());
  }
}

/**
Ugly macro for logging at a specific frequency and level

@param interval time in seconds to display message (write as integer)
@param logger the logger used.
@param level set TRACE < DEBUG < INFO < WARN < ERROR < FATAL.
@param message string or << operators
 */
#define DEBUG_SL_TASK

#ifdef  DEBUG_SL_TASK
#define LOG_HYQ_STATES(logger, level, hyq_states) { \
    std::stringstream ss; \
    ss << std::setprecision(2) << std::fixed << "HyqStates:\n"; \
    int s = 0; \
    for (hyq::HyqState hyq : hyq_states) { \
      ss << "state " << s++ << ":\n" << hyq; \
    } \
    LOG4CXX_##level(logger, ss.str()); \
}
#else
#define LOG_HYQ_STATES(interval,logger,level,message)
#endif


#ifdef  DEBUG_SL_TASK
#define LOG4CXX_SPLINE_NODES(logger, level, spline_nodes_vec) { \
    std::stringstream ss; \
    ss << std::setprecision(2) << std::fixed << "SplineNodes:\n"; \
    int s = 0; \
    for (SplineNode sn : spline_nodes_vec) { \
      ss << "Node "     << s++            << ":\n" \
         << "pos="       << sn.pos         << "\n" \
         << "ori="       << sn.ori         << "\n" \
         << "feet="      << sn.feet        << "\n" \
         << "swingleg="  << sn.swingleg    << "\n" \
         << "T=" << sn.T << "\n"; \
    } \
    LOG4CXX_##level(logger, ss.str()); \
}
#else
#define LOG4CXX_SPLINE_NODES(logger, level, spline_nodes_vec)
#endif




} // namespace logger_helpers
} // namespace utils
} // namespace xpp

#endif // _XPP_UTILS_LOGGER_HELPERS_INL_H_
