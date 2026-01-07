#ifndef VAMP_PRESETS_H
#define VAMP_PRESETS_H

#if !MR_PLANNER_WITH_VAMP
#error "vamp_presets.h included without VAMP support"
#endif

#include <mr_planner/backends/vamp_instance.h>

#include <vamp/robots/gp4.hh>
#include <vamp/robots/panda.hh>

extern template class VampInstance<vamp::robots::GP4, vamp::robots::GP4>;
extern template class VampInstance<
    vamp::robots::GP4,
    vamp::robots::GP4,
    vamp::robots::GP4,
    vamp::robots::GP4>;
extern template class VampInstance<vamp::robots::Panda, vamp::robots::Panda>;
extern template class VampInstance<
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda>;
extern template class VampInstance<
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda>;
extern template class VampInstance<
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda>;
extern template class VampInstance<
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda>;
extern template class VampInstance<
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

using VampDualGp4Instance = VampInstance<vamp::robots::GP4, vamp::robots::GP4>;
using VampQuadGp4Instance = VampInstance<
    vamp::robots::GP4,
    vamp::robots::GP4,
    vamp::robots::GP4,
    vamp::robots::GP4>;
using VampPandaPairInstance = VampInstance<vamp::robots::Panda, vamp::robots::Panda>;
using VampPandaTripleInstance = VampInstance<
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda>;
using VampPandaQuadInstance = VampInstance<
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda>;
using VampPandaHexInstance = VampInstance<
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda>;
using VampPandaOctInstance = VampInstance<
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda,
    vamp::robots::Panda>;
using VampPandaDecInstance = VampInstance<
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

#endif // VAMP_PRESETS_H
