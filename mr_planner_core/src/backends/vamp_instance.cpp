#define MR_PLANNER_VAMP_INSTANCE_FORCE_HEADER_ONLY

#include <mr_planner/backends/vamp_presets.h>

// Explicit instantiations for common planning scenarios.
template class VampInstance<vamp::robots::GP4, vamp::robots::GP4>;
template class VampInstance<
    vamp::robots::GP4,
    vamp::robots::GP4,
    vamp::robots::GP4,
    vamp::robots::GP4>;
template class VampInstance<vamp::robots::Panda, vamp::robots::Panda>;
template class VampInstance<
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda>;
template class VampInstance<
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda>;
template class VampInstance<
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda>;
template class VampInstance<
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda>;
template class VampInstance<
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda>;
