#pragma once

#include <string>
#include <memory>
#include <type_traits>

#include <vamp/vector.hh>
#include <vamp/collision/math.hh>

#include <Eigen/Geometry>

namespace vamp::collision
{
    namespace detail
    {
        template <typename TargetT, typename SourceT>
        inline constexpr auto cast_component(const SourceT &value) noexcept -> TargetT
        {
            if constexpr (std::is_same_v<TargetT, SourceT>)
            {
                return value;
            }
            else if constexpr (vamp::is_vector<SourceT>::value && not vamp::is_vector<TargetT>::value)
            {
                return static_cast<TargetT>(value[{0, 0}]);
            }
            else if constexpr (vamp::is_vector<TargetT>::value && not vamp::is_vector<SourceT>::value)
            {
                using ScalarT = typename TargetT::S::ScalarT;
                return TargetT(static_cast<ScalarT>(value));
            }
            else if constexpr (vamp::is_vector<TargetT>::value && vamp::is_vector<SourceT>::value)
            {
                if constexpr (std::is_convertible_v<SourceT, TargetT>)
                {
                    return static_cast<TargetT>(value);
                }
                else
                {
                    using ScalarT = typename TargetT::S::ScalarT;
                    return TargetT(static_cast<ScalarT>(value[{0, 0}]));
                }
            }
            else
            {
                return static_cast<TargetT>(value);
            }
        }
    }  // namespace detail

    template <typename DataT>
    struct Shape
    {
        std::string name;
        DataT min_distance;

        Shape() = default;

        template <typename OtherDataT>
        explicit Shape(const Shape<OtherDataT> &other)
          : name(other.name)
          , min_distance(detail::cast_component<DataT>(other.min_distance))
        {
        }
    };

    // A cuboid, or rectangular prism, shape oriented in an arbitrary frame. The
    // cube is defined by its center (x, y, z), the three axes defining its frame
    // (axis_1_x, axis_1_y, axis_1_z, etc.), and the half-lengths along each axis
    // (axis_1_r, etc.)
    template <typename DataT>
    struct Cuboid : public Shape<DataT>
    {
        DataT x;
        DataT y;
        DataT z;

        DataT axis_1_x;
        DataT axis_1_y;
        DataT axis_1_z;
        DataT axis_2_x;
        DataT axis_2_y;
        DataT axis_2_z;
        DataT axis_3_x;
        DataT axis_3_y;
        DataT axis_3_z;

        DataT axis_1_r;
        DataT axis_2_r;
        DataT axis_3_r;

        inline constexpr auto compute_min_distance() -> DataT
        {
            auto d_1 = dot_3(-x, -y, -z, axis_1_x, axis_1_y, axis_1_z);
            auto d_2 = dot_3(-x, -y, -z, axis_2_x, axis_2_y, axis_2_z);
            auto d_3 = dot_3(-x, -y, -z, axis_3_x, axis_3_y, axis_3_z);

            auto v_1 = clamp(d_1, -axis_1_r, axis_1_r);
            auto v_2 = clamp(d_2, -axis_2_r, axis_2_r);
            auto v_3 = clamp(d_3, -axis_3_r, axis_3_r);

            auto x_n = x + axis_1_x * v_1 + axis_2_x * v_2 + axis_3_x * v_3;
            auto y_n = y + axis_1_y * v_1 + axis_2_y * v_2 + axis_3_y * v_3;
            auto z_n = z + axis_1_z * v_1 + axis_2_z * v_2 + axis_3_z * v_3;

            return sqrt(x_n * x_n + y_n * y_n + z_n * z_n);
        }

        Cuboid() = default;

        explicit Cuboid(
            DataT x,
            DataT y,
            DataT z,  //
            DataT axis_1_x,
            DataT axis_1_y,
            DataT axis_1_z,  //
            DataT axis_2_x,
            DataT axis_2_y,
            DataT axis_2_z,  //
            DataT axis_3_x,
            DataT axis_3_y,
            DataT axis_3_z,  //
            DataT axis_1_r,
            DataT axis_2_r,
            DataT axis_3_r)
          : Shape<DataT>()
          , x(x)
          , y(y)
          , z(z)
          , axis_1_x(axis_1_x)
          , axis_1_y(axis_1_y)
          , axis_1_z(axis_1_z)
          , axis_2_x(axis_2_x)
          , axis_2_y(axis_2_y)
          , axis_2_z(axis_2_z)
          , axis_3_x(axis_3_x)
          , axis_3_y(axis_3_y)
          , axis_3_z(axis_3_z)
          , axis_1_r(axis_1_r)
          , axis_2_r(axis_2_r)
          , axis_3_r(axis_3_r)
        {
            Shape<DataT>::min_distance = compute_min_distance();
        }

        template <typename OtherDataT>
        explicit Cuboid(const Cuboid<OtherDataT> &other)
          : Shape<DataT>(other)
          , x(detail::cast_component<DataT>(other.x))
          , y(detail::cast_component<DataT>(other.y))
          , z(detail::cast_component<DataT>(other.z))
          , axis_1_x(detail::cast_component<DataT>(other.axis_1_x))
          , axis_1_y(detail::cast_component<DataT>(other.axis_1_y))
          , axis_1_z(detail::cast_component<DataT>(other.axis_1_z))
          , axis_2_x(detail::cast_component<DataT>(other.axis_2_x))
          , axis_2_y(detail::cast_component<DataT>(other.axis_2_y))
          , axis_2_z(detail::cast_component<DataT>(other.axis_2_z))
          , axis_3_x(detail::cast_component<DataT>(other.axis_3_x))
          , axis_3_y(detail::cast_component<DataT>(other.axis_3_y))
          , axis_3_z(detail::cast_component<DataT>(other.axis_3_z))
          , axis_1_r(detail::cast_component<DataT>(other.axis_1_r))
          , axis_2_r(detail::cast_component<DataT>(other.axis_2_r))
          , axis_3_r(detail::cast_component<DataT>(other.axis_3_r))
        {
        }
    };

    // A cylinder shape oriented in an arbitrary frame. The cylinder is defined by
    // two endpoints, which we store as the first point (x1, y1, z1) and the vector
    // from the first point to the second point (xv, yv, zv), and its radius (r)
    template <typename DataT>
    struct Cylinder : public Shape<DataT>
    {
        DataT x1;
        DataT y1;
        DataT z1;

        DataT xv;
        DataT yv;
        DataT zv;

        DataT r;

        // This is the reciprocal of the dot product of the vector between the
        // cylinder's endpoints, i.e. rdv = 1 / ([xv yv zv] . [xv yv zv]) We store it
        // for convenience to make some computations faster
        DataT rdv;

        inline constexpr auto x2() -> DataT
        {
            return x1 + xv;
        }

        inline constexpr auto y2() -> DataT
        {
            return y1 + yv;
        }

        inline constexpr auto z2() -> DataT
        {
            return z1 + zv;
        }

        inline constexpr auto compute_min_distance() -> DataT
        {
            auto dot = clamp(dot_3(-x1, -y1, -z1, xv, yv, zv) * rdv, 0.F, 1.F);

            auto xp = x1 + xv * dot;
            auto yp = y1 + yv * dot;
            auto zp = z1 + zv * dot;

            auto xo = -xp;
            auto yo = -yp;
            auto zo = -zp;

            auto ol = sqrt(dot_3(xo, yo, zo, xo, yo, zo));
            xo = xo / ol;
            yo = yo / ol;
            zo = zo / ol;

            auto ro = clamp(ol, 0.F, r);

            auto xn = xp + ro * xo;
            auto yn = yp + ro * yo;
            auto zn = zp + ro * zo;

            return sqrt(xn * xn + yn * yn + zn * zn);
        }

        Cylinder() = default;

        explicit Cylinder(
            DataT x1,
            DataT y1,
            DataT z1,  //
            DataT xv,
            DataT yv,
            DataT zv,  //
            DataT r,
            DataT rdv)
          : Shape<DataT>(), x1(x1), y1(y1), z1(z1), xv(xv), yv(yv), zv(zv), r(r), rdv(rdv)
        {
            Shape<DataT>::min_distance = compute_min_distance();
        }

        template <typename OtherDataT>
        explicit Cylinder(const Cylinder<OtherDataT> &other)
          : Shape<DataT>(other)
          , x1(detail::cast_component<DataT>(other.x1))
          , y1(detail::cast_component<DataT>(other.y1))
          , z1(detail::cast_component<DataT>(other.z1))
          , xv(detail::cast_component<DataT>(other.xv))
          , yv(detail::cast_component<DataT>(other.yv))
          , zv(detail::cast_component<DataT>(other.zv))
          , r(detail::cast_component<DataT>(other.r))
          , rdv(detail::cast_component<DataT>(other.rdv))
        {
        }
    };

    template <typename DataT>
    using Capsule = Cylinder<DataT>;

    // A sphere shape, represented as its center (x, y, z) and radius (r)
    template <typename DataT>
    struct Sphere : public Shape<DataT>
    {
        DataT x;
        DataT y;
        DataT z;
        DataT r;

        Sphere() = default;

        explicit Sphere(DataT x, DataT y, DataT z, DataT r) : Shape<DataT>(), x(x), y(y), z(z), r(r)
        {
            Shape<DataT>::min_distance = sqrt(x * x + y * y + z * z) - r;
        }

        template <typename OtherDataT>
        explicit Sphere(const Sphere<OtherDataT> &other)
          : Shape<DataT>(other)
          , x(detail::cast_component<DataT>(other.x))
          , y(detail::cast_component<DataT>(other.y))
          , z(detail::cast_component<DataT>(other.z))
          , r(detail::cast_component<DataT>(other.r))
        {
        }
    };

    // A heighfield. Defined by height pixel buffer.
    template <typename DataT>
    struct HeightField : public Shape<DataT>
    {
        DataT x;  // offset
        DataT y;
        DataT z;

        DataT xs;  // scaling factor
        DataT ys;
        DataT zs;

        std::size_t xd;  // image size
        std::size_t yd;

        std::size_t xd2;  // image size half
        std::size_t yd2;

        std::vector<float> data;  // flattened row-major data

        HeightField() = default;

        explicit HeightField(
            DataT x,
            DataT y,
            DataT z,
            DataT xs,
            DataT ys,
            DataT zs,
            std::size_t xd,
            std::size_t yd,
            const std::vector<float> &data)
          : Shape<DataT>()
          , x(x)
          , y(y)
          , z(z)
          , xs(xs)
          , ys(ys)
          , zs(zs)
          , xd(xd)
          , yd(yd)
          , xd2(xd / 2)
          , yd2(yd / 2)
          , data(data)
        {
            Shape<DataT>::min_distance = 0;
        }

        template <typename OtherDataT>
        explicit HeightField(const HeightField<OtherDataT> &other)
          : Shape<DataT>(other)
          , x(detail::cast_component<DataT>(other.x))
          , y(detail::cast_component<DataT>(other.y))
          , z(detail::cast_component<DataT>(other.z))
          , xs(detail::cast_component<DataT>(other.xs))
          , ys(detail::cast_component<DataT>(other.ys))
          , zs(detail::cast_component<DataT>(other.zs))
          , xd(other.xd)
          , yd(other.yd)
          , xd2(other.xd2)
          , yd2(other.yd2)
          , data(other.data)
        {
        }
    };
}  // namespace vamp::collision
