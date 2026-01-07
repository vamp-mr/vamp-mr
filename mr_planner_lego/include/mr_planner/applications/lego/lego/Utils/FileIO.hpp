#ifndef LEGO_UTILS_FILEIO_HPP
#define LEGO_UTILS_FILEIO_HPP

#include <mr_planner/applications/lego/lego/Utils/Math.hpp>
#include <mr_planner/applications/lego/lego/Utils/ErrorHandling.hpp>

namespace lego_manipulation
{
namespace io
{
Eigen::MatrixXd LoadMatFromFile(const std::string fname);
void SaveMatToFile(const Eigen::MatrixXd& mat, const std::string& fname);

}

}
#endif // LEGO_UTILS_FILEIO_HPP