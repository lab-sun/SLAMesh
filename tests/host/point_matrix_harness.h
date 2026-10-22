#pragma once

// Shared operations and reference model for the PointMatrix fuzz and stress harnesses.

#include "../../src/utility/pointMatrix.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <sstream>
#include <streambuf>
#include <string>
#include <vector>

// Stop growing past this so one input stays cheap to check.
static const int kPointCap = 5000;

enum OpKind {
    kAddPoint = 0,
    kAddPointVar,
    kAPlusB,
    kBPlusA,
    kAPlusA,
    kCopyCtor,
    kCopyAssign,
    kAssignMatrix,
    kClear,
    kClearQuick,
    kGetPoint,
    kCopyTransform,
    kTransform,
    kEigen,
    kPrintVar,
    kOpCount
};

inline const char* op_name(int op) {
    static const char* names[] = {
        "addPoint",
        "addPointWithVariance",
        "A += B",
        "B += A",
        "A += A",
        "copy construct",
        "copy assign",
        "assign matrix",
        "clear",
        "clear_quick",
        "getPointWithVariance",
        "copyAndTransform",
        "transformPoints",
        "eigenDecomposition",
        "printVariance"
    };
    static_assert(sizeof(names) / sizeof(names[0]) == static_cast<std::size_t>(kOpCount),
                  "op name count");
    if (op < 0 || op >= kOpCount) {
        return "op";
    }
    return names[op];
}

struct RefPoint {
    double x;
    double y;
    double z;
    double v;
};

struct Model {
    std::vector<RefPoint> pts;
    double stamp;
    Model() : stamp(-1.0) {}
};

struct Harness {
    PointMatrix a;
    PointMatrix b;
    Model ma;
    Model mb;
    std::uint64_t op_counts[kOpCount];
    const char* current_op;

    Harness() : current_op("init") {
        for (int i = 0; i < kOpCount; ++i) {
            op_counts[i] = 0;
        }
    }
};

[[noreturn]] inline void fail_trap(const std::string& msg) {
    std::cerr << msg << std::endl;
    __builtin_trap();
}

[[noreturn]] inline void fail_exit(const std::string& msg) {
    std::cerr << msg << std::endl;
    std::exit(1);
}

typedef void (*FailFn)(const std::string& msg);

#define FAIL_AT(fail, text)                         \
    do {                                            \
        std::ostringstream oss_;                    \
        oss_ << __FILE__ << ":" << __LINE__ << " " \
             << text;                               \
        fail(oss_.str());                           \
    } while (0)

class NullBuf : public std::streambuf {
public:
    int overflow(int c) override { return c; }
};

class StreamRedirect {
public:
    StreamRedirect(std::ostream& stream, std::ostream& sink)
        : stream_(stream), old_(stream.rdbuf(sink.rdbuf())) {}
    ~StreamRedirect() { stream_.rdbuf(old_); }

private:
    std::ostream& stream_;
    std::streambuf* old_;
};

inline std::ostream& null_stream() {
    static NullBuf buf;
    static std::ostream out(&buf);
    return out;
}

inline bool same_double(double a, double b) {
    if (std::isnan(a) && std::isnan(b)) {
        return true;
    }
    return a == b;
}

// Transformed coordinates are compared with a small relative tolerance.
inline bool close_double(double a, double b) {
    if (same_double(a, b)) {
        return true;
    }
    double diff = std::abs(a - b);
    double scale = 1.0;
    if (std::abs(a) > scale) {
        scale = std::abs(a);
    }
    if (std::abs(b) > scale) {
        scale = std::abs(b);
    }
    return diff <= 1e-8 * scale;
}

inline void check_model(const Harness& h, const PointMatrix& pm, const Model& m,
                        const char* name, FailFn fail) {
    if (pm.num_point != static_cast<int>(m.pts.size()) || !same_double(pm.stamp, m.stamp)) {
        FAIL_AT(fail, h.current_op << " " << name
            << " num_point " << pm.num_point << " model " << m.pts.size()
            << " stamp " << pm.stamp << " model_stamp " << m.stamp);
    }
    if (pm.point.cols() != pm.variance.cols() || pm.point.cols() < pm.num_point) {
        FAIL_AT(fail, h.current_op << " " << name
            << " cols point " << pm.point.cols()
            << " variance " << pm.variance.cols()
            << " num_point " << pm.num_point);
    }
    int n = pm.num_point;
    for (int i = 0; i < n; ++i) {
        const RefPoint& r = m.pts[static_cast<std::size_t>(i)];
        if (!same_double(pm.point(0, i), r.x) ||
            !same_double(pm.point(1, i), r.y) ||
            !same_double(pm.point(2, i), r.z) ||
            !same_double(pm.variance(0, i), r.v)) {
            FAIL_AT(fail, h.current_op << " " << name << " index " << i
                << " got " << pm.point(0, i) << " " << pm.point(1, i) << " "
                << pm.point(2, i) << " v " << pm.variance(0, i)
                << " model " << r.x << " " << r.y << " " << r.z << " v " << r.v);
        }
    }
}

inline bool points_finite(const Model& m) {
    for (std::size_t i = 0; i < m.pts.size(); ++i) {
        if (!std::isfinite(m.pts[i].x) || !std::isfinite(m.pts[i].y) ||
            !std::isfinite(m.pts[i].z)) {
            return false;
        }
    }
    return true;
}

template <typename Source>
bool pull_u8(Source& in, std::uint8_t& v) {
    return in.pull(&v, 1);
}

template <typename Source>
bool pull_u16(Source& in, std::uint16_t& v) {
    std::uint8_t b[2];
    if (!in.pull(b, 2)) {
        return false;
    }
    v = static_cast<std::uint16_t>(b[0]) |
        static_cast<std::uint16_t>(static_cast<std::uint16_t>(b[1]) << 8);
    return true;
}

inline double decode_range(std::uint16_t u, double lo, double hi) {
    // One reserved code stands for a non-finite sample.
    if (u == 0xFFFF) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    double t = static_cast<double>(u) / 65535.0;
    return lo + (hi - lo) * t;
}

template <typename Source>
bool pull_coord(Source& in, double& v) {
    std::uint16_t u;
    if (!pull_u16(in, u)) {
        return false;
    }
    v = decode_range(u, -4000.0, 4000.0);
    return true;
}

template <typename Source>
bool pull_variance(Source& in, double& v) {
    std::uint16_t u;
    if (!pull_u16(in, u)) {
        return false;
    }
    v = decode_range(u, -1.0, 120.0);
    return true;
}

template <typename Source>
bool pull_rigid(Source& in, Transf& T) {
    std::uint8_t raw[4];
    if (!in.pull(raw, 4)) {
        return false;
    }
    std::uint8_t tb[6];
    if (!in.pull(tb, 6)) {
        return false;
    }
    // raw[0] == 255 stores a non-finite translation so the caller can skip it.
    if (raw[0] == 255) {
        T.setIdentity();
        T(0, 3) = std::numeric_limits<double>::quiet_NaN();
        return true;
    }
    double angle = (static_cast<double>(raw[0]) / 255.0) * 2.0 * 3.14159265358979323846;
    Eigen::Vector3d axis(static_cast<int>(raw[1]) - 128,
                         static_cast<int>(raw[2]) - 128,
                         static_cast<int>(raw[3]) - 128);
    if (axis.squaredNorm() == 0.0) {
        axis = Eigen::Vector3d::UnitZ();
    } else {
        axis.normalize();
    }
    unsigned ux = static_cast<unsigned>(tb[0]) | (static_cast<unsigned>(tb[1]) << 8);
    unsigned uy = static_cast<unsigned>(tb[2]) | (static_cast<unsigned>(tb[3]) << 8);
    unsigned uz = static_cast<unsigned>(tb[4]) | (static_cast<unsigned>(tb[5]) << 8);
    T.setIdentity();
    T.block<3, 3>(0, 0) = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    T(0, 3) = (static_cast<double>(ux) - 32768.0) / 16.0;
    T(1, 3) = (static_cast<double>(uy) - 32768.0) / 16.0;
    T(2, 3) = (static_cast<double>(uz) - 32768.0) / 16.0;
    return true;
}

inline PointMatrix& pick_pm(Harness& h, int which) {
    return which ? h.b : h.a;
}

inline Model& pick_model(Harness& h, int which) {
    return which ? h.mb : h.ma;
}

inline const char* side_name(int which) {
    return which ? "B" : "A";
}

inline void append_from(PointMatrix& dst, Model& dst_m, const PointMatrix& src, const Model& src_m) {
    if (src_m.pts.empty()) {
        return;
    }
    if (dst_m.pts.size() + src_m.pts.size() > static_cast<std::size_t>(kPointCap)) {
        return;
    }
    dst += src;
    dst_m.pts.insert(dst_m.pts.end(), src_m.pts.begin(), src_m.pts.end());
}

inline void compare_transformed(const Harness& h, const PointMatrix& pm, const Model& src_m,
                                const Transf& T, FailFn fail, Model& synced) {
    synced = src_m;
    int n = pm.num_point;
    if (n != static_cast<int>(src_m.pts.size())) {
        FAIL_AT(fail, h.current_op << " transformed num_point " << n
            << " model " << src_m.pts.size());
    }
    for (int i = 0; i < n; ++i) {
        const RefPoint& r = src_m.pts[static_cast<std::size_t>(i)];
        Eigen::Vector4d hom;
        hom << r.x, r.y, r.z, 1.0;
        Eigen::Vector4d got = T * hom;
        if (!close_double(pm.point(0, i), got(0)) ||
            !close_double(pm.point(1, i), got(1)) ||
            !close_double(pm.point(2, i), got(2)) ||
            !same_double(pm.variance(0, i), r.v)) {
            FAIL_AT(fail, h.current_op << " transform index " << i
                << " got " << pm.point(0, i) << " " << pm.point(1, i) << " " << pm.point(2, i)
                << " v " << pm.variance(0, i)
                << " expect " << got(0) << " " << got(1) << " " << got(2)
                << " v " << r.v);
        }
        // Keep the stored doubles so later checks can compare exactly.
        synced.pts[static_cast<std::size_t>(i)].x = pm.point(0, i);
        synced.pts[static_cast<std::size_t>(i)].y = pm.point(1, i);
        synced.pts[static_cast<std::size_t>(i)].z = pm.point(2, i);
    }
    if (!same_double(pm.stamp, src_m.stamp)) {
        FAIL_AT(fail, h.current_op << " stamp " << pm.stamp << " model_stamp " << src_m.stamp);
    }
    synced.stamp = pm.stamp;
}

inline bool eigen_result_cleared(const PointMatrix& pm) {
    // Failure path matches clear_quick: min_value -1 and zero vectors.
    if (pm.min_value != -1.0 || !pm.eig_sorted.empty()) {
        return false;
    }
    return pm.min_vector.isZero() && pm.eigenvalMat.isZero() && pm.eigenvectorMat.isZero();
}

inline void check_eigen(const Harness& h, PointMatrix& pm, const Model& m, FailFn fail) {
    bool finite = points_finite(m);
    {
        // Empty input prints an error line. Drop it.
        StreamRedirect guard(std::cout, null_stream());
        pm.eigenDecomposition();
    }
    if (pm.num_point <= 0) {
        return;
    }
    // Non-finite points make EigenSolver fail. It then leaves a defined cleared state.
    if (!finite || eigen_result_cleared(pm)) {
        if (!eigen_result_cleared(pm)) {
            FAIL_AT(fail, h.current_op << " non-finite eigen result was not cleared"
                << " min_value " << pm.min_value);
        }
        return;
    }
    double sx = 0.0;
    double sy = 0.0;
    double sz = 0.0;
    int n = pm.num_point;
    for (int i = 0; i < n; ++i) {
        const RefPoint& r = m.pts[static_cast<std::size_t>(i)];
        sx += r.x;
        sy += r.y;
        sz += r.z;
    }
    double inv = 1.0 / static_cast<double>(n);
    sx *= inv;
    sy *= inv;
    sz *= inv;
    if (!close_double(pm.centroid(0), sx) ||
        !close_double(pm.centroid(1), sy) ||
        !close_double(pm.centroid(2), sz)) {
        FAIL_AT(fail, h.current_op << " centroid got "
            << pm.centroid(0) << " " << pm.centroid(1) << " " << pm.centroid(2)
            << " mean " << sx << " " << sy << " " << sz);
    }
    double evs[3];
    evs[0] = pm.eigenvalMat(0, 0);
    evs[1] = pm.eigenvalMat(1, 1);
    evs[2] = pm.eigenvalMat(2, 2);
    const double eps = 1e-8;
    for (int k = 0; k < 3; ++k) {
        if (!std::isfinite(evs[k]) || !std::isfinite(pm.min_value) ||
            !(pm.min_value <= evs[k] + eps)) {
            FAIL_AT(fail, h.current_op << " min_value " << pm.min_value
                << " ev " << evs[0] << " " << evs[1] << " " << evs[2]);
        }
    }
    if (pm.min_vector.allFinite()) {
        double nrm = pm.min_vector.norm();
        if (std::abs(nrm - 1.0) > 1e-3) {
            FAIL_AT(fail, h.current_op << " min_vector norm " << nrm
                << " vec " << pm.min_vector(0) << " " << pm.min_vector(1) << " "
                << pm.min_vector(2));
        }
    }
}

template <typename Source>
bool harness_step(Harness& h, Source& in, FailFn fail) {
    std::uint8_t opb;
    if (!pull_u8(in, opb)) {
        return false;
    }
    int op = static_cast<int>((opb >> 1) % static_cast<unsigned>(kOpCount));
    int which = opb & 1;
    h.current_op = op_name(op);

    switch (op) {
    case kAddPoint: {
        double x, y, z;
        if (!pull_coord(in, x) || !pull_coord(in, y) || !pull_coord(in, z)) {
            return false;
        }
        ++h.op_counts[op];
        Model& m = pick_model(h, which);
        if (m.pts.size() + 1 <= static_cast<std::size_t>(kPointCap)) {
            pick_pm(h, which).addPoint(Point(x, y, z));
            RefPoint r;
            r.x = x;
            r.y = y;
            r.z = z;
            r.v = 100.0;
            m.pts.push_back(r);
        }
        break;
    }
    case kAddPointVar: {
        double x, y, z, v;
        if (!pull_coord(in, x) || !pull_coord(in, y) || !pull_coord(in, z) ||
            !pull_variance(in, v)) {
            return false;
        }
        ++h.op_counts[op];
        Model& m = pick_model(h, which);
        if (m.pts.size() + 1 <= static_cast<std::size_t>(kPointCap)) {
            PointWithVariance pv;
            pv << x, y, z, v;
            pick_pm(h, which).addPointWithVariance(pv);
            RefPoint r;
            r.x = x;
            r.y = y;
            r.z = z;
            r.v = v;
            m.pts.push_back(r);
        }
        break;
    }
    case kAPlusB:
        ++h.op_counts[op];
        append_from(h.a, h.ma, h.b, h.mb);
        break;
    case kBPlusA:
        ++h.op_counts[op];
        append_from(h.b, h.mb, h.a, h.ma);
        break;
    case kAPlusA: {
        ++h.op_counts[op];
        // Self add appends a second copy. The read columns and the write columns differ.
        std::size_t n = h.ma.pts.size();
        if (n == 0 || n * 2 <= static_cast<std::size_t>(kPointCap)) {
            h.a += h.a;
            if (n > 0) {
                h.ma.pts.reserve(n * 2);
                for (std::size_t i = 0; i < n; ++i) {
                    h.ma.pts.push_back(h.ma.pts[i]);
                }
            }
        }
        break;
    }
    case kCopyCtor: {
        std::uint8_t extra;
        if (!pull_u8(in, extra)) {
            return false;
        }
        ++h.op_counts[op];
        int src_i = which;
        int dst_i = extra & 1;
        PointMatrix cloned(pick_pm(h, src_i));
        Model cloned_m = pick_model(h, src_i);
        check_model(h, cloned, cloned_m, "clone", fail);
        pick_pm(h, dst_i) = cloned;
        pick_model(h, dst_i) = cloned_m;
        break;
    }
    case kCopyAssign:
        // Copy A onto B, or B onto A.
        ++h.op_counts[op];
        if (which == 0) {
            h.a = h.b;
            h.ma = h.mb;
        } else {
            h.b = h.a;
            h.mb = h.ma;
        }
        break;
    case kAssignMatrix: {
        std::uint8_t spec;
        std::uint16_t ncols_raw;
        if (!pull_u8(in, spec) || !pull_u16(in, ncols_raw)) {
            return false;
        }
        int rows = (spec & 1) ? 4 : 3;
        int cols = static_cast<int>(ncols_raw % 301);
        Eigen::MatrixXd mat(rows, cols);
        Model next;
        next.stamp = -1.0;
        next.pts.resize(static_cast<std::size_t>(cols));
        for (int c = 0; c < cols; ++c) {
            double x, y, z, v;
            if (!pull_coord(in, x) || !pull_coord(in, y) || !pull_coord(in, z)) {
                return false;
            }
            if (rows == 4) {
                if (!pull_variance(in, v)) {
                    return false;
                }
            } else {
                v = 100.0;
            }
            mat(0, c) = x;
            mat(1, c) = y;
            mat(2, c) = z;
            if (rows == 4) {
                mat(3, c) = v;
            }
            next.pts[static_cast<std::size_t>(c)].x = x;
            next.pts[static_cast<std::size_t>(c)].y = y;
            next.pts[static_cast<std::size_t>(c)].z = z;
            next.pts[static_cast<std::size_t>(c)].v = v;
        }
        ++h.op_counts[op];
        pick_pm(h, which) = mat;
        pick_model(h, which) = next;
        break;
    }
    case kClear:
        ++h.op_counts[op];
        pick_pm(h, which).clear();
        pick_model(h, which).pts.clear();
        pick_model(h, which).stamp = -1.0;
        break;
    case kClearQuick:
        ++h.op_counts[op];
        pick_pm(h, which).clear_quick();
        pick_model(h, which).pts.clear();
        pick_model(h, which).stamp = -1.0;
        break;
    case kGetPoint: {
        std::uint8_t mode;
        std::uint16_t raw;
        if (!pull_u8(in, mode) || !pull_u16(in, raw)) {
            return false;
        }
        ++h.op_counts[op];
        PointMatrix& pm = pick_pm(h, which);
        Model& m = pick_model(h, which);
        int n = pm.num_point;
        if ((mode & 1) && n > 0) {
            int index = static_cast<int>(raw % static_cast<unsigned>(n));
            PointWithVariance got = pm.getPointWithVariance(index);
            const RefPoint& r = m.pts[static_cast<std::size_t>(index)];
            if (!same_double(got(0), r.x) || !same_double(got(1), r.y) ||
                !same_double(got(2), r.z) || !same_double(got(3), r.v)) {
                FAIL_AT(fail, h.current_op << " " << side_name(which) << " index " << index
                    << " got " << got(0) << " " << got(1) << " " << got(2) << " v " << got(3)
                    << " model " << r.x << " " << r.y << " " << r.z << " v " << r.v);
            }
        } else {
            int index;
            if (mode & 2) {
                index = -1 - static_cast<int>(raw % 1000);
            } else {
                index = n + static_cast<int>(raw % 1000);
            }
            PointWithVariance got;
            {
                // Out of range writes an error line. Drop it and check the returned point.
                StreamRedirect guard(std::cerr, null_stream());
                got = pm.getPointWithVariance(index);
            }
            if (!same_double(got(0), 0.0) || !same_double(got(1), 0.0) ||
                !same_double(got(2), 0.0) || !same_double(got(3), 100.0)) {
                FAIL_AT(fail, h.current_op << " " << side_name(which)
                    << " out of range index " << index
                    << " got " << got(0) << " " << got(1) << " " << got(2) << " v " << got(3));
            }
        }
        break;
    }
    case kCopyTransform: {
        std::uint8_t extra;
        Transf T;
        if (!pull_u8(in, extra) || !pull_rigid(in, T)) {
            return false;
        }
        ++h.op_counts[op];
        int src_i = which;
        int dst_i = extra & 1;
        PointMatrix& src = pick_pm(h, src_i);
        Model& src_m = pick_model(h, src_i);
        if (src_m.pts.empty()) {
            PointMatrix out = src.copyAndTransform(T);
            check_model(h, src, src_m, "copy source", fail);
            check_model(h, out, src_m, "copy result", fail);
            pick_pm(h, dst_i) = out;
            pick_model(h, dst_i) = src_m;
        } else if (!T.allFinite() || !points_finite(src_m)) {
            // Skip non-finite inputs. The stored points stay as they are.
        } else {
            PointMatrix out = src.copyAndTransform(T);
            check_model(h, src, src_m, "copy source", fail);
            Model synced;
            compare_transformed(h, out, src_m, T, fail, synced);
            pick_pm(h, dst_i) = out;
            pick_model(h, dst_i) = synced;
        }
        break;
    }
    case kTransform: {
        Transf T;
        if (!pull_rigid(in, T)) {
            return false;
        }
        ++h.op_counts[op];
        PointMatrix& pm = pick_pm(h, which);
        Model& m = pick_model(h, which);
        if (m.pts.empty()) {
            StreamRedirect guard(std::cout, null_stream());
            pm.transformPoints(T);
        } else if (!T.allFinite() || !points_finite(m)) {
            // Skip non-finite inputs. The stored points stay as they are.
        } else {
            Model saved = m;
            {
                StreamRedirect guard(std::cout, null_stream());
                pm.transformPoints(T);
            }
            Model synced;
            compare_transformed(h, pm, saved, T, fail, synced);
            m = synced;
        }
        break;
    }
    case kEigen:
        ++h.op_counts[op];
        check_eigen(h, pick_pm(h, which), pick_model(h, which), fail);
        break;
    case kPrintVar: {
        ++h.op_counts[op];
        StreamRedirect guard(std::cout, null_stream());
        pick_pm(h, which).printVariance();
        break;
    }
    default:
        FAIL_AT(fail, "bad op " << op);
        break;
    }

    check_model(h, h.a, h.ma, "A", fail);
    check_model(h, h.b, h.mb, "B", fail);
    return true;
}

inline void print_op_counts(const Harness& h, std::uint64_t ops) {
    std::cout << "ops " << ops << "\n";
    for (int i = 0; i < kOpCount; ++i) {
        std::cout << op_name(i) << " " << h.op_counts[i] << "\n";
    }
}
