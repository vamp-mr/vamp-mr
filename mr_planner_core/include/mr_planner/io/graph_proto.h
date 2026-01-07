#pragma once

#include <string>

namespace tpg
{
class TPG;
class ADG;
}  // namespace tpg

namespace mr_planner::graph_proto
{

bool write_tpg(const tpg::TPG &tpg, const std::string &path, std::string *error = nullptr);
bool write_adg(const tpg::ADG &adg, const std::string &path, std::string *error = nullptr);

}  // namespace mr_planner::graph_proto

