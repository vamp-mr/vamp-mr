#include <vamp_python_init.hh>

#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <vamp/bindings/multi_robot_runtime.hh>
#include <vamp/collision/environment.hh>
#include <vamp/collision/sphere_sphere.hh>
#include <vamp/collision/multi_robot.hh>

namespace nb = nanobind;
namespace vc = vamp::collision;
namespace vr = vamp::binding::multi_robot;

namespace
{
    struct RawState
    {
        std::string robot;
        std::vector<float> configuration;
        Eigen::Isometry3f transform{Eigen::Isometry3f::Identity()};
        const vc::Attachment<float> *attachment = nullptr;
        nb::object attachment_owner;
        std::string label;
        bool has_label = false;
    };

    inline auto to_float_vector(nb::handle value) -> std::vector<float>
    {
        nb::sequence seq = nb::cast<nb::sequence>(value);
        std::size_t count = nb::len(seq);

        std::vector<float> result;
        result.reserve(count);
        for (std::size_t i = 0; i < count; ++i)
        {
            result.emplace_back(nb::cast<float>(seq[i]));
        }

        return result;
    }

    inline auto to_transform(nb::handle value) -> Eigen::Isometry3f
    {
        Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();

        if (not value || value.is_none())
        {
            return transform;
        }

        try
        {
            Eigen::Matrix4f matrix = nb::cast<Eigen::Matrix4f>(value);
            transform.matrix() = matrix;
            return transform;
        }
        catch (const nb::cast_error &)
        {
            // Fall through to handle generic sequences below.
        }

        nb::sequence seq = nb::cast<nb::sequence>(value);
        const std::size_t outer = nb::len(seq);

        Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
        if (outer == 16)
        {
            for (std::size_t i = 0; i < 16; ++i)
            {
                matrix(static_cast<Eigen::Index>(i / 4), static_cast<Eigen::Index>(i % 4)) =
                    nb::cast<float>(seq[i]);
            }
        }
        else if (outer == 4)
        {
            for (std::size_t row = 0; row < 4; ++row)
            {
                nb::sequence row_seq = nb::cast<nb::sequence>(seq[row]);
                if (nb::len(row_seq) != 4)
                {
                    throw nb::value_error("Transform rows must contain exactly four elements.");
                }

                for (std::size_t col = 0; col < 4; ++col)
                {
                    matrix(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col)) =
                        nb::cast<float>(row_seq[col]);
                }
            }
        }
        else
        {
            throw nb::value_error("Transform must be a 4x4 array-like object.");
        }

        transform.matrix() = matrix;
        return transform;
    }

    inline auto parse_state(nb::handle value) -> RawState
    {
        if (not nb::isinstance<nb::dict>(value))
        {
            throw nb::type_error("Each state must be a mapping with keys 'robot' and 'configuration'.");
        }

        nb::dict mapping = nb::cast<nb::dict>(value);

        if (not mapping.contains("robot"))
        {
            throw nb::value_error("State is missing required key 'robot'.");
        }

        if (not mapping.contains("configuration"))
        {
            throw nb::value_error("State is missing required key 'configuration'.");
        }

        RawState state;
        state.robot = nb::cast<std::string>(mapping["robot"]);
        state.configuration = to_float_vector(mapping["configuration"]);

        if (mapping.contains("label"))
        {
            state.label = nb::cast<std::string>(mapping["label"]);
            state.has_label = true;
        }

        nb::handle transform_handle;
        if (mapping.contains("transform"))
        {
            transform_handle = mapping["transform"];
        }
        else if (mapping.contains("base_transform"))
        {
            transform_handle = mapping["base_transform"];
        }

        if (transform_handle and not transform_handle.is_none())
        {
            state.transform = to_transform(transform_handle);
        }

        if (mapping.contains("attachment"))
        {
            nb::object attachment_obj = nb::cast<nb::object>(mapping["attachment"]);
            if (not attachment_obj.is_none())
            {
                state.attachment_owner = attachment_obj;
                state.attachment = nb::cast<const vc::Attachment<float> *>(attachment_obj);
            }
        }

        return state;
    }

    inline auto parse_states(const nb::sequence &states_seq) -> std::vector<RawState>
    {
        const std::size_t count = nb::len(states_seq);
        std::vector<RawState> states;
        states.reserve(count);

        for (std::size_t i = 0; i < count; ++i)
        {
            states.emplace_back(parse_state(states_seq[i]));
        }

        return states;
    }

    inline auto lookup_entries(const std::vector<RawState> &states) -> std::vector<const vr::RegistryEntry *>
    {
        std::vector<const vr::RegistryEntry *> entries;
        entries.reserve(states.size());

        for (const auto &raw : states)
        {
            const auto *entry = vr::lookup(raw.robot);
            if (entry == nullptr)
            {
                const auto message = std::string("Unknown robot '") + raw.robot + "'.";
                throw nb::value_error(message.c_str());
            }

            if (raw.configuration.size() != entry->dimension)
            {
                std::ostringstream oss;
                oss << "Configuration for robot '" << raw.robot << "' must contain " << entry->dimension
                    << " values (received " << raw.configuration.size() << ").";
                throw nb::value_error(oss.str().c_str());
            }

            entries.push_back(entry);
        }

        return entries;
    }

    inline auto penetration(const vc::Sphere<float> &a, const vc::Sphere<float> &b) -> float
    {
        const float dx = a.x - b.x;
        const float dy = a.y - b.y;
        const float dz = a.z - b.z;
        const float distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        return distance - (a.r + b.r);
    }

    inline auto lookup_index(const nb::dict &mapping, const char *primary, const char *fallback) -> std::size_t
    {
        if (mapping.contains(primary))
        {
            return nb::cast<std::size_t>(mapping[primary]);
        }

        if (fallback != nullptr && mapping.contains(fallback))
        {
            return nb::cast<std::size_t>(mapping[fallback]);
        }

        std::ostringstream oss;
        oss << "Allowed collision entry is missing index key '" << primary << "'.";
        throw nb::value_error(oss.str().c_str());
    }

    inline auto parse_string_list(const nb::dict &mapping, const char *single_key, const char *plural_key)
        -> std::vector<std::string>
    {
        std::vector<std::string> values;

        if (mapping.contains(single_key))
        {
            values.emplace_back(nb::cast<std::string>(mapping[single_key]));
            return values;
        }

        if (plural_key != nullptr && mapping.contains(plural_key))
        {
            nb::sequence seq = nb::cast<nb::sequence>(mapping[plural_key]);
            const std::size_t count = nb::len(seq);
            values.reserve(count);
            for (std::size_t i = 0; i < count; ++i)
            {
                values.emplace_back(nb::cast<std::string>(seq[i]));
            }
            return values;
        }

        return values;
    }

    inline auto parse_allowed_collisions(
        nb::handle allowed_obj,
        const std::vector<const vr::RegistryEntry *> &entries,
        vc::MultiRobotCollisionFilter &filter) -> void
    {
        if (not allowed_obj or allowed_obj.is_none())
        {
            return;
        }

        nb::sequence seq = nb::cast<nb::sequence>(allowed_obj);
        std::vector<std::size_t> spheres;

        const std::size_t count = nb::len(seq);
        for (std::size_t idx = 0; idx < count; ++idx)
        {
            nb::handle item = seq[idx];
            if (not nb::isinstance<nb::dict>(item))
            {
                throw nb::type_error("Each allowed collision must be a mapping.");
            }

            nb::dict mapping = nb::cast<nb::dict>(item);

            std::size_t state_index = lookup_index(mapping, "state", "state_index");

            if (state_index >= entries.size())
            {
                throw nb::value_error("Allowed collision references an out-of-range state index.");
            }

            bool disable = true;
            if (mapping.contains("enabled"))
            {
                disable = not nb::cast<bool>(mapping["enabled"]);
            }
            else if (mapping.contains("collide"))
            {
                disable = not nb::cast<bool>(mapping["collide"]);
            }

            if (not disable)
            {
                continue;
            }

            const auto *entry = entries[state_index];
            auto &robot_filter = filter.ensure_robot(state_index, entry->sphere_count);

            const auto objects = parse_string_list(mapping, "object", "objects");
            if (objects.empty())
            {
                throw nb::value_error("Allowed collision entry must specify at least one object name.");
            }

            const bool attachment = mapping.contains("attachment") && nb::cast<bool>(mapping["attachment"]);

            if (attachment)
            {
                for (const auto &object : objects)
                {
                    robot_filter.allow_attachment(object);
                }

                continue;
            }

            if (entry->link_lookup == nullptr)
            {
                throw nb::value_error("Robot does not expose link mapping for collision filtering.");
            }

            const auto links = parse_string_list(mapping, "link", "links");
            if (links.empty())
            {
                throw nb::value_error("Allowed collision entry must specify at least one link name.");
            }

            for (const auto &link_name : links)
            {
                spheres.clear();
                if (not entry->link_lookup(link_name, spheres) || spheres.empty())
                {
                    std::ostringstream oss;
                    oss << "Unknown link '" << link_name << "' for robot state " << state_index << ".";
                    throw nb::value_error(oss.str().c_str());
                }

                for (const auto sphere_index : spheres)
                {
                    for (const auto &object : objects)
                    {
                        robot_filter.allow_sphere(sphere_index, object);
                    }
                }
            }
        }
    }

    inline auto spheres_collide(
        const std::vector<vc::Sphere<float>> &a,
        const std::vector<vc::Sphere<float>> &b) -> bool
    {
        for (const auto &sa : a)
        {
            for (const auto &sb : b)
            {
                if (vc::sphere_sphere_sql2(sa, sb) <= 0.F)
                {
                    return true;
                }
            }
        }

        return false;
    }

    inline auto attachments_vs_spheres(
        const std::vector<vc::Sphere<float>> &attachments,
        const std::vector<vc::Sphere<float>> &spheres) -> bool
    {
        if (attachments.empty())
        {
            return false;
        }

        return spheres_collide(attachments, spheres);
    }

    inline auto attachments_vs_attachments(
        const std::vector<vc::Sphere<float>> &a,
        const std::vector<vc::Sphere<float>> &b) -> bool
    {
        if (a.empty() || b.empty())
        {
            return false;
        }

        return spheres_collide(a, b);
    }

    inline auto collision_free(const std::vector<vr::PreparedState> &states) -> bool
    {
        const std::size_t count = states.size();

        for (std::size_t i = 0; i < count; ++i)
        {
            for (std::size_t j = i + 1; j < count; ++j)
            {
                if (spheres_collide(states[i].spheres, states[j].spheres))
                {
                    return false;
                }
            }
        }

        for (std::size_t i = 0; i < count; ++i)
        {
            if (states[i].attachments.empty())
            {
                continue;
            }

            for (std::size_t j = 0; j < count; ++j)
            {
                if (i == j)
                {
                    continue;
                }

                if (attachments_vs_spheres(states[i].attachments, states[j].spheres))
                {
                    return false;
                }
            }
        }

        for (std::size_t i = 0; i < count; ++i)
        {
            if (states[i].attachments.empty())
            {
                continue;
            }

            for (std::size_t j = i + 1; j < count; ++j)
            {
                if (attachments_vs_attachments(states[i].attachments, states[j].attachments))
                {
                    return false;
                }
            }
        }

        return true;
    }

    template <typename Callback>
    inline auto prepare_states(
        const std::vector<RawState> &raw_states,
        const std::vector<const vr::RegistryEntry *> &entries,
        const vc::Environment<float> *environment,
        const vc::MultiRobotCollisionFilter *filter,
        Callback &&on_invalid) -> bool
    {
        std::vector<vr::PreparedState> prepared(raw_states.size());

        for (std::size_t i = 0; i < raw_states.size(); ++i)
        {
            const auto &raw = raw_states[i];
            const auto *entry = entries[i];

            const auto *robot_filter = (environment != nullptr && filter != nullptr) ? filter->robot(i) : nullptr;

            if (not entry->fn(
                    raw.configuration,
                    raw.transform,
                    raw.attachment,
                    environment,
                    robot_filter,
                    prepared[i]))
            {
                return on_invalid();
            }

            prepared[i].registry = entry;
        }

        return collision_free(prepared);
    }

    inline auto prepare_debug_states(
        const std::vector<RawState> &raw_states,
        const std::vector<const vr::RegistryEntry *> &entries,
        const vc::Environment<float> &environment,
        const vc::MultiRobotCollisionFilter *filter) -> std::vector<vr::DebugState>
    {
        std::vector<vr::DebugState> prepared(raw_states.size());

        for (std::size_t i = 0; i < raw_states.size(); ++i)
        {
            const auto &raw = raw_states[i];
            const auto *entry = entries[i];

            if (entry->debug_fn == nullptr)
            {
                throw nb::value_error("Robot does not expose debug preparation routine.");
            }

            const auto *robot_filter = (filter != nullptr) ? filter->robot(i) : nullptr;

            if (not entry->debug_fn(
                    raw.configuration,
                    raw.transform,
                    raw.attachment,
                    &environment,
                    robot_filter,
                    prepared[i]))
            {
                prepared[i].valid = false;
            }

            prepared[i].registry = entry;
        }

        return prepared;
    }
}  // namespace

void vamp::binding::init_multi_robot(nanobind::module_ &pymodule)
{
    using namespace nb::literals;

    pymodule.def(
        "fkcc_multi_all",
        [](const nb::sequence &states_seq,
           const vc::Environment<float> &environment,
           nb::handle allowed_collisions)
        {
            auto states = parse_states(states_seq);
            if (states.empty())
            {
                throw nb::value_error("fkcc_multi_all requires at least one robot state.");
            }

            auto entries = lookup_entries(states);
            vc::MultiRobotCollisionFilter filter;
            const vc::MultiRobotCollisionFilter *filter_ptr = nullptr;

            if (allowed_collisions && not allowed_collisions.is_none())
            {
                parse_allowed_collisions(allowed_collisions, entries, filter);
                filter_ptr = &filter;
            }

            return prepare_states(
                states,
                entries,
                &environment,
                filter_ptr,
                []() { return false; });
        },
        "states"_a,
        "environment"_a,
        nb::arg("allowed_collisions") = nb::none(),
        "Checks whether multiple robots are collision-free against each other and the shared environment.");

    pymodule.def(
        "compute_contacts",
        [](const nb::sequence &states_seq,
           const vc::Environment<float> &environment,
           nb::handle allowed_collisions)
        {
            auto states = parse_states(states_seq);
            if (states.empty())
            {
                throw nb::value_error("compute_contacts requires at least one robot state.");
            }

            auto entries = lookup_entries(states);
            vc::MultiRobotCollisionFilter filter;
            const vc::MultiRobotCollisionFilter *filter_ptr = nullptr;

            if (allowed_collisions && not allowed_collisions.is_none())
            {
                parse_allowed_collisions(allowed_collisions, entries, filter);
                filter_ptr = &filter;
            }

            auto prepared = prepare_debug_states(states, entries, environment, filter_ptr);

            nb::dict result;
            nb::list state_list;
            const std::size_t count = prepared.size();

            for (std::size_t i = 0; i < count; ++i)
            {
                const auto &debug_state = prepared[i];

                nb::dict state_dict;
                state_dict["index"] = nb::int_(i);
                state_dict["robot"] = nb::str(states[i].robot.c_str());
                if (states[i].has_label)
                {
                    state_dict["label"] = nb::str(states[i].label.c_str());
                }
                state_dict["valid"] = nb::bool_(debug_state.valid);

                nb::list spheres_list;
                for (const auto &sphere : debug_state.spheres)
                {
                    spheres_list.append(sphere);
                }
                state_dict["spheres"] = std::move(spheres_list);

                nb::list env_list;
                for (const auto &per_sphere : debug_state.sphere_environment_collisions)
                {
                    nb::list per;
                    for (const auto &object : per_sphere)
                    {
                        per.append(nb::str(object.c_str()));
                    }
                    env_list.append(std::move(per));
                }
                state_dict["environment"] = std::move(env_list);

                nb::list self_list;
                for (const auto &pair : debug_state.self_collisions)
                {
                    if (pair.first >= debug_state.spheres.size() || pair.second >= debug_state.spheres.size())
                    {
                        continue;
                    }

                    nb::dict entry;
                    entry["a"] = nb::int_(pair.first);
                    entry["b"] = nb::int_(pair.second);
                    entry["penetration"] = nb::float_(penetration(
                        debug_state.spheres[pair.first],
                        debug_state.spheres[pair.second]));
                    self_list.append(std::move(entry));
                }
                state_dict["self"] = std::move(self_list);

                nb::list attachments_list;
                for (const auto &sphere : debug_state.attachments)
                {
                    attachments_list.append(sphere);
                }
                state_dict["attachments"] = std::move(attachments_list);

                nb::list attachment_env_list;
                for (const auto &per_attachment : debug_state.attachment_environment_collisions)
                {
                    nb::list per;
                    for (const auto &object : per_attachment)
                    {
                        per.append(nb::str(object.c_str()));
                    }
                    attachment_env_list.append(std::move(per));
                }
                state_dict["attachments_environment"] = std::move(attachment_env_list);

                nb::list attachment_self_list;
                for (std::size_t ai = 0; ai < debug_state.attachments.size(); ++ai)
                {
                    const auto &attachment_sphere = debug_state.attachments[ai];
                    for (std::size_t si = 0; si < debug_state.spheres.size(); ++si)
                    {
                        const float pen = penetration(attachment_sphere, debug_state.spheres[si]);
                        if (pen <= 0.F)
                        {
                            nb::dict entry;
                            entry["attachment_index"] = nb::int_(ai);
                            entry["sphere_index"] = nb::int_(si);
                            entry["penetration"] = nb::float_(pen);
                            attachment_self_list.append(std::move(entry));
                        }
                    }
                }
                state_dict["attachment_self"] = std::move(attachment_self_list);

                state_list.append(std::move(state_dict));
            }

            result["states"] = std::move(state_list);

            nb::list cross_sphere_list;
            nb::list cross_attachment_sphere_list;
            nb::list cross_attachment_attachment_list;

            for (std::size_t i = 0; i < count; ++i)
            {
                for (std::size_t j = i + 1; j < count; ++j)
                {
                    const auto &spheres_i = prepared[i].spheres;
                    const auto &spheres_j = prepared[j].spheres;

                    for (std::size_t si = 0; si < spheres_i.size(); ++si)
                    {
                        for (std::size_t sj = 0; sj < spheres_j.size(); ++sj)
                        {
                            const float pen = penetration(spheres_i[si], spheres_j[sj]);
                            if (pen <= 0.F)
                            {
                                nb::dict entry;
                                entry["first_state"] = nb::int_(i);
                                entry["first_sphere"] = nb::int_(si);
                                entry["second_state"] = nb::int_(j);
                                entry["second_sphere"] = nb::int_(sj);
                                entry["penetration"] = nb::float_(pen);
                                cross_sphere_list.append(std::move(entry));
                            }
                        }
                    }

                    const auto &attachments_i = prepared[i].attachments;
                    const auto &attachments_j = prepared[j].attachments;

                    for (std::size_t ai = 0; ai < attachments_i.size(); ++ai)
                    {
                        for (std::size_t sj = 0; sj < spheres_j.size(); ++sj)
                        {
                            const float pen = penetration(attachments_i[ai], spheres_j[sj]);
                            if (pen <= 0.F)
                            {
                                nb::dict entry;
                                entry["attachment_state"] = nb::int_(i);
                                entry["attachment_index"] = nb::int_(ai);
                                entry["sphere_state"] = nb::int_(j);
                                entry["sphere_index"] = nb::int_(sj);
                                entry["penetration"] = nb::float_(pen);
                                cross_attachment_sphere_list.append(std::move(entry));
                            }
                        }
                    }

                    for (std::size_t aj = 0; aj < attachments_j.size(); ++aj)
                    {
                        for (std::size_t si = 0; si < spheres_i.size(); ++si)
                        {
                            const float pen = penetration(attachments_j[aj], spheres_i[si]);
                            if (pen <= 0.F)
                            {
                                nb::dict entry;
                                entry["attachment_state"] = nb::int_(j);
                                entry["attachment_index"] = nb::int_(aj);
                                entry["sphere_state"] = nb::int_(i);
                                entry["sphere_index"] = nb::int_(si);
                                entry["penetration"] = nb::float_(pen);
                                cross_attachment_sphere_list.append(std::move(entry));
                            }
                        }
                    }

                    for (std::size_t ai = 0; ai < attachments_i.size(); ++ai)
                    {
                        for (std::size_t aj = 0; aj < attachments_j.size(); ++aj)
                        {
                            const float pen = penetration(attachments_i[ai], attachments_j[aj]);
                            if (pen <= 0.F)
                            {
                                nb::dict entry;
                                entry["first_state"] = nb::int_(i);
                                entry["first_attachment"] = nb::int_(ai);
                                entry["second_state"] = nb::int_(j);
                                entry["second_attachment"] = nb::int_(aj);
                                entry["penetration"] = nb::float_(pen);
                                cross_attachment_attachment_list.append(std::move(entry));
                            }
                        }
                    }
                }
            }

            result["cross_sphere"] = std::move(cross_sphere_list);
            result["cross_attachment_sphere"] = std::move(cross_attachment_sphere_list);
            result["cross_attachment_attachment"] = std::move(cross_attachment_attachment_list);

            return result;
        },
        "states"_a,
        "environment"_a,
        nb::arg("allowed_collisions") = nb::none(),
        "Compute per-state and cross-state contact information, including attachments.");

    pymodule.def(
        "fkcc_multi_self",
        [](const nb::sequence &states_seq, nb::handle allowed_collisions)
        {
            auto states = parse_states(states_seq);
            if (states.empty())
            {
                throw nb::value_error("fkcc_multi_self requires at least one robot state.");
            }

            auto entries = lookup_entries(states);
            vc::MultiRobotCollisionFilter filter;
            const vc::MultiRobotCollisionFilter *filter_ptr = nullptr;

            if (allowed_collisions && not allowed_collisions.is_none())
            {
                parse_allowed_collisions(allowed_collisions, entries, filter);
                filter_ptr = &filter;
            }

            return prepare_states(
                states,
                entries,
                nullptr,
                filter_ptr,
                []() { return false; });
        },
        "states"_a,
        nb::arg("allowed_collisions") = nb::none(),
        "Checks whether multiple robots are collision-free against each other, ignoring the environment.");
}
