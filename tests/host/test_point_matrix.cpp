#include "../../src/utility/pointMatrix.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <vector>

// Counts failures even when NDEBUG removes assert().
static int g_failures = 0;

#define CHECK(cond)                                                       \
    do {                                                                  \
        if (!(cond)) {                                                    \
            std::cerr << "CHECK failed: " << #cond << " (" << __FILE__   \
                      << ":" << __LINE__ << ")\n";                        \
            ++g_failures;                                                 \
            if (g_failures > 40) {                                        \
                std::cerr << "too many failures\n";                       \
                std::exit(1);                                             \
            }                                                             \
        }                                                                 \
    } while (0)

struct RefPoint {
    double x;
    double y;
    double z;
    double v;
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

static bool close_to(double a, double b) {
    return std::abs(a - b) <= 1e-9;
}

static RefPoint make_ref(int id) {
    RefPoint r;
    r.x = 0.5 * id - 3.0;
    r.y = -0.25 * id + 1.0;
    r.z = 0.125 * id;
    r.v = 0.01 * (id % 20);
    return r;
}

static void push_with_variance(PointMatrix& pm, std::vector<RefPoint>& ref, const RefPoint& r) {
    PointWithVariance pv;
    pv << r.x, r.y, r.z, r.v;
    pm.addPointWithVariance(pv);
    ref.push_back(r);
}

static void push_plain(PointMatrix& pm, std::vector<RefPoint>& ref, const Point& p) {
    pm.addPoint(p);
    RefPoint r;
    r.x = p(0);
    r.y = p(1);
    r.z = p(2);
    r.v = 100.0;
    ref.push_back(r);
}

static void check_values(PointMatrix& pm, const std::vector<RefPoint>& ref) {
    CHECK(pm.num_point == static_cast<int>(ref.size()));
    if (pm.num_point != static_cast<int>(ref.size())) {
        return;
    }
    for (int i = 0; i < pm.num_point; ++i) {
        const RefPoint& r = ref[static_cast<std::size_t>(i)];
        CHECK(pm.point(0, i) == r.x);
        CHECK(pm.point(1, i) == r.y);
        CHECK(pm.point(2, i) == r.z);
        CHECK(pm.variance(0, i) == r.v);
        PointWithVariance got = pm.getPointWithVariance(i);
        CHECK(got(0) == r.x);
        CHECK(got(1) == r.y);
        CHECK(got(2) == r.z);
        CHECK(got(3) == r.v);
    }
}

static void check_free_capacity(const PointMatrix& pm) {
    CHECK(pm.point.cols() >= pm.num_point);
    CHECK(pm.variance.cols() >= pm.num_point);
    CHECK(pm.point.cols() == pm.variance.cols());
    for (Eigen::Index c = pm.num_point; c < pm.point.cols(); ++c) {
        CHECK(pm.point(0, c) == 0.0);
        CHECK(pm.point(1, c) == 0.0);
        CHECK(pm.point(2, c) == 0.0);
        CHECK(pm.variance(0, c) == 100.0);
    }
}

static void check_out_of_range_point(const PointWithVariance& p) {
    CHECK(p(0) == 0.0);
    CHECK(p(1) == 0.0);
    CHECK(p(2) == 0.0);
    CHECK(p(3) == 100.0);
}

static void fill_n(PointMatrix& pm, std::vector<RefPoint>& ref, int n) {
    for (int i = 0; i < n; ++i) {
        push_with_variance(pm, ref, make_ref(static_cast<int>(ref.size())));
    }
}

static void test_constructors() {
    PointMatrix empty;
    CHECK(empty.num_point == 0);
    CHECK(empty.stamp == -1.0);
    CHECK(empty.point.rows() == 3);
    CHECK(empty.point.cols() == 0);
    CHECK(empty.variance.rows() == 1);
    CHECK(empty.variance.cols() == 0);
    CHECK(empty.resize_step == 64);
    check_free_capacity(empty);

    PointMatrix sized(5);
    CHECK(sized.num_point == 0);
    CHECK(sized.point.rows() == 3);
    CHECK(sized.point.cols() == 5);
    CHECK(sized.variance.cols() == 5);
    check_free_capacity(sized);

    PointMatrix zero_size(0);
    CHECK(zero_size.num_point == 0);
    CHECK(zero_size.point.cols() == 0);
    check_free_capacity(zero_size);

    PointMatrix stamped(12.5);
    CHECK(stamped.stamp == 12.5);
    CHECK(stamped.num_point == 0);
    CHECK(stamped.point.cols() == 0);
    check_free_capacity(stamped);
}

static void test_add_growth() {
    PointMatrix plain;
    std::vector<RefPoint> plain_ref;
    for (int i = 0; i < 200; ++i) {
        push_plain(plain, plain_ref, Point(i * 1.0, i * -0.5, i * 0.25));
    }
    CHECK(plain.num_point == 200);
    CHECK(plain.point.cols() >= 200);
    CHECK(plain.point.cols() % plain.resize_step == 0);
    check_values(plain, plain_ref);
    check_free_capacity(plain);

    PointMatrix with_var;
    std::vector<RefPoint> var_ref;
    fill_n(with_var, var_ref, 200);
    CHECK(with_var.num_point == 200);
    CHECK(with_var.point.cols() >= 200);
    CHECK(with_var.point.cols() % with_var.resize_step == 0);
    check_values(with_var, var_ref);
    check_free_capacity(with_var);

    // Cross the 64, 128, and 192 boundaries in one matrix.
    PointMatrix mixed;
    std::vector<RefPoint> mixed_ref;
    for (int i = 0; i < 200; ++i) {
        if (i % 2 == 0) {
            push_plain(mixed, mixed_ref, Point(i, i + 1.0, i + 2.0));
        } else {
            push_with_variance(mixed, mixed_ref, make_ref(i));
        }
    }
    check_values(mixed, mixed_ref);
    check_free_capacity(mixed);
}

static void test_plus_equal() {
    PointMatrix left_empty;
    PointMatrix right_empty;
    left_empty += right_empty;
    CHECK(left_empty.num_point == 0);
    check_free_capacity(left_empty);

    {
        PointMatrix dst;
        std::vector<RefPoint> ref;
        fill_n(dst, ref, 30);
        Eigen::Index cols_before = dst.point.cols();
        double x_before = dst.point(0, 0);
        PointMatrix empty;
        dst += empty;
        CHECK(dst.point.cols() == cols_before);
        CHECK(dst.point(0, 0) == x_before);
        check_values(dst, ref);
        check_free_capacity(dst);
    }

    // space_left == -10. Second resize branch. Adds onto an empty matrix.
    {
        PointMatrix src;
        std::vector<RefPoint> ref;
        fill_n(src, ref, 10);
        PointMatrix dst;
        Eigen::Index src_cols = src.point.cols();
        double src_x = src.point(0, 0);
        dst += src;
        CHECK(src.point.cols() == src_cols);
        CHECK(src.point(0, 0) == src_x);
        CHECK(dst.num_point == 10);
        CHECK(dst.point.cols() == dst.resize_step);
        check_values(dst, ref);
        check_free_capacity(dst);
    }

    // space_left == -resize_step. Still the second branch.
    {
        PointMatrix src;
        std::vector<RefPoint> ref;
        fill_n(src, ref, 64);
        PointMatrix dst;
        dst += src;
        CHECK(dst.num_point == 64);
        CHECK(dst.point.cols() == 64);
        check_values(dst, ref);
        check_free_capacity(dst);
    }

    // space_left == -resize_step - 1. First resize branch.
    {
        PointMatrix src;
        std::vector<RefPoint> ref;
        fill_n(src, ref, 65);
        PointMatrix dst;
        dst += src;
        CHECK(dst.num_point == 65);
        CHECK(dst.point.cols() == 65 + dst.resize_step);
        check_values(dst, ref);
        check_free_capacity(dst);
    }

    // Large add onto an empty matrix.
    {
        PointMatrix src;
        std::vector<RefPoint> ref;
        fill_n(src, ref, 200);
        PointMatrix dst;
        dst += src;
        CHECK(dst.num_point == 200);
        CHECK(dst.point.cols() == 200 + dst.resize_step);
        check_values(dst, ref);
        check_free_capacity(dst);
    }

    // Second branch from a full matrix: 64 points, then 10 more.
    {
        PointMatrix src_a;
        std::vector<RefPoint> ref;
        fill_n(src_a, ref, 64);
        PointMatrix dst;
        dst += src_a;
        PointMatrix src_b;
        std::vector<RefPoint> extra;
        fill_n(src_b, extra, 10);
        ref.insert(ref.end(), extra.begin(), extra.end());
        dst += src_b;
        CHECK(dst.num_point == 74);
        CHECK(dst.point.cols() == 128);
        check_values(dst, ref);
        check_free_capacity(dst);
    }

    // First branch from a full matrix: 64 points, then 200 more.
    {
        PointMatrix src_a;
        std::vector<RefPoint> ref;
        fill_n(src_a, ref, 64);
        PointMatrix dst;
        dst += src_a;
        PointMatrix src_b;
        std::vector<RefPoint> extra;
        fill_n(src_b, extra, 200);
        ref.insert(ref.end(), extra.begin(), extra.end());
        dst += src_b;
        CHECK(dst.num_point == 264);
        CHECK(dst.point.cols() == 264 + dst.resize_step);
        check_values(dst, ref);
        check_free_capacity(dst);
    }

    // No resize. Spare capacity from the size constructor covers the add.
    {
        PointMatrix dst(80);
        PointMatrix src;
        std::vector<RefPoint> ref;
        fill_n(src, ref, 10);
        dst += src;
        CHECK(dst.num_point == 10);
        CHECK(dst.point.cols() == 80);
        check_values(dst, ref);
        check_free_capacity(dst);
    }

    // Second branch with a partially filled size constructor.
    {
        PointMatrix dst(70);
        std::vector<RefPoint> ref;
        for (int i = 0; i < 10; ++i) {
            push_plain(dst, ref, Point(i, i + 0.5, i + 1.5));
        }
        PointMatrix src;
        std::vector<RefPoint> extra;
        fill_n(src, extra, 80);
        ref.insert(ref.end(), extra.begin(), extra.end());
        dst += src;
        CHECK(dst.num_point == 90);
        CHECK(dst.point.cols() == 70 + dst.resize_step);
        check_values(dst, ref);
        check_free_capacity(dst);
    }

    // First branch with a partially filled size constructor.
    {
        PointMatrix dst(10);
        std::vector<RefPoint> ref;
        for (int i = 0; i < 5; ++i) {
            push_plain(dst, ref, Point(-i, i * 0.5, 2.0 * i));
        }
        PointMatrix src;
        std::vector<RefPoint> extra;
        fill_n(src, extra, 200);
        ref.insert(ref.end(), extra.begin(), extra.end());
        dst += src;
        CHECK(dst.num_point == 205);
        CHECK(dst.point.cols() == 205 + dst.resize_step);
        check_values(dst, ref);
        check_free_capacity(dst);
    }
}

static void expect_same_points(PointMatrix& a, PointMatrix& b) {
    CHECK(a.num_point == b.num_point);
    CHECK(a.stamp == b.stamp);
    for (int i = 0; i < a.num_point && i < b.num_point; ++i) {
        CHECK(a.point(0, i) == b.point(0, i));
        CHECK(a.point(1, i) == b.point(1, i));
        CHECK(a.point(2, i) == b.point(2, i));
        CHECK(a.variance(0, i) == b.variance(0, i));
    }
}

static void test_copy() {
    PointMatrix orig;
    std::vector<RefPoint> ref;
    fill_n(orig, ref, 50);
    orig.stamp = 3.25;
    orig.centroid << 1.5, -2.5, 4.0;
    orig.min_value = 0.125;
    orig.min_vector << 0.0, 0.0, 1.0;
    orig.variance_sum = 6.5;

    PointMatrix copy(orig);
    expect_same_points(copy, orig);
    CHECK(copy.centroid(0) == orig.centroid(0));
    CHECK(copy.centroid(1) == orig.centroid(1));
    CHECK(copy.centroid(2) == orig.centroid(2));
    CHECK(copy.min_value == orig.min_value);
    CHECK(copy.min_vector(2) == orig.min_vector(2));
    CHECK(copy.variance_sum == orig.variance_sum);
    check_values(copy, ref);

    copy.point(0, 0) = 42.0;
    copy.variance(0, 1) = 7.0;
    copy.stamp = -9.0;
    copy.addPoint(Point(1.0, 2.0, 3.0));
    CHECK(orig.point(0, 0) == ref[0].x);
    CHECK(orig.variance(0, 1) == ref[1].v);
    CHECK(orig.stamp == 3.25);
    CHECK(orig.num_point == 50);
    check_values(orig, ref);

    orig.point(2, 0) = -5.0;
    CHECK(copy.point(2, 0) == ref[0].z);

    PointMatrix assigned(4);
    assigned.stamp = 99.0;
    assigned.addPoint(Point(9.0, 9.0, 9.0));
    assigned = orig;
    CHECK(assigned.num_point == orig.num_point);
    CHECK(assigned.stamp == orig.stamp);
    CHECK(assigned.point(0, 0) == orig.point(0, 0));
    CHECK(assigned.point(2, 0) == -5.0);
    CHECK(assigned.variance(0, 1) == orig.variance(0, 1));
    assigned.point(0, 0) = 99.0;
    assigned.variance(0, 0) = 3.0;
    assigned.addPointWithVariance(PointWithVariance(4.0, 5.0, 6.0, 0.2));
    CHECK(orig.point(0, 0) == ref[0].x);
    CHECK(orig.point(2, 0) == -5.0);
    CHECK(orig.variance(0, 0) == ref[0].v);
    CHECK(orig.num_point == 50);

    PointMatrix empty;
    assigned = empty;
    CHECK(assigned.num_point == 0);
    CHECK(assigned.stamp == -1.0);
    CHECK(orig.num_point == 50);
}

static void test_assign_matrix() {
    PointMatrix pm;
    pm.addPointWithVariance(PointWithVariance(8.0, 8.0, 8.0, 0.3));

    Eigen::MatrixXd xyz(3, 4);
    xyz << 1, 2, 3, 4,
           5, 6, 7, 8,
           9, 10, 11, 12;
    pm = xyz;
    CHECK(pm.num_point == 4);
    CHECK(pm.point(0, 0) == 1);
    CHECK(pm.point(1, 0) == 5);
    CHECK(pm.point(2, 0) == 9);
    CHECK(pm.point(0, 1) == 2);
    CHECK(pm.point(1, 1) == 6);
    CHECK(pm.point(2, 1) == 10);
    CHECK(pm.point(0, 3) == 4);
    CHECK(pm.point(1, 3) == 8);
    CHECK(pm.point(2, 3) == 12);
    for (int i = 0; i < pm.num_point; ++i) {
        CHECK(pm.variance(0, i) == 100.0);
    }
    check_free_capacity(pm);

    Eigen::MatrixXd xyzv(4, 3);
    xyzv << 1, 2, 3,
            4, 5, 6,
            7, 8, 9,
            0.2, 0.4, 1.5;
    pm = xyzv;
    CHECK(pm.num_point == 3);
    CHECK(pm.point(0, 0) == 1);
    CHECK(pm.point(1, 0) == 4);
    CHECK(pm.point(2, 0) == 7);
    CHECK(pm.point(0, 1) == 2);
    CHECK(pm.point(1, 1) == 5);
    CHECK(pm.point(2, 1) == 8);
    CHECK(pm.point(0, 2) == 3);
    CHECK(pm.point(1, 2) == 6);
    CHECK(pm.point(2, 2) == 9);
    CHECK(pm.variance(0, 0) == 0.2);
    CHECK(pm.variance(0, 1) == 0.4);
    CHECK(pm.variance(0, 2) == 1.5);
    check_free_capacity(pm);

    // 3-row input replaces a previous 4-row variance.
    pm = xyz;
    CHECK(pm.num_point == 4);
    CHECK(pm.variance(0, 0) == 100.0);
    CHECK(pm.variance(0, 2) == 100.0);
}

static void test_get_point() {
    PointMatrix empty;
    std::stringstream err;
    PointWithVariance a;
    PointWithVariance b;
    PointWithVariance c;
    {
        StreamRedirect redirect(std::cerr, err);
        a = empty.getPointWithVariance(0);
        b = empty.getPointWithVariance(-1);
        c = empty.getPointWithVariance(4);
    }
    check_out_of_range_point(a);
    check_out_of_range_point(b);
    check_out_of_range_point(c);
    CHECK(err.str().find("out of range") != std::string::npos);

    PointMatrix pm;
    std::vector<RefPoint> ref;
    fill_n(pm, ref, 5);
    PointWithVariance first = pm.getPointWithVariance(0);
    PointWithVariance last = pm.getPointWithVariance(4);
    CHECK(first(0) == ref[0].x);
    CHECK(first(3) == ref[0].v);
    CHECK(last(1) == ref[4].y);
    CHECK(last(3) == ref[4].v);

    PointMatrix full;
    std::vector<RefPoint> full_ref;
    fill_n(full, full_ref, full.resize_step);
    CHECK(full.num_point == full.point.cols());
    PointWithVariance at_end;
    PointWithVariance negative;
    std::stringstream err_full;
    {
        StreamRedirect redirect(std::cerr, err_full);
        at_end = full.getPointWithVariance(full.num_point);
        negative = full.getPointWithVariance(-5);
    }
    check_out_of_range_point(at_end);
    check_out_of_range_point(negative);
    CHECK(err_full.str().find("out of range") != std::string::npos);
    PointWithVariance back = full.getPointWithVariance(full.num_point - 1);
    CHECK(back(0) == full_ref.back().x);
    CHECK(back(3) == full_ref.back().v);
    check_values(full, full_ref);
}

static void expect_transformed(const PointMatrix& pm, const std::vector<RefPoint>& src, const Transf& transf) {
    CHECK(pm.num_point == static_cast<int>(src.size()));
    for (int i = 0; i < pm.num_point && i < static_cast<int>(src.size()); ++i) {
        Eigen::Vector4d hom;
        hom << src[static_cast<std::size_t>(i)].x, src[static_cast<std::size_t>(i)].y,
            src[static_cast<std::size_t>(i)].z, 1.0;
        Eigen::Vector4d expected = transf * hom;
        CHECK(close_to(pm.point(0, i), expected(0)));
        CHECK(close_to(pm.point(1, i), expected(1)));
        CHECK(close_to(pm.point(2, i), expected(2)));
        CHECK(pm.variance(0, i) == src[static_cast<std::size_t>(i)].v);
    }
}

static void test_transform() {
    Transf turn;
    turn << 0, -1, 0, 1,
            1, 0, 0, 2,
            0, 0, 1, 3,
            0, 0, 0, 1;

    PointMatrix src;
    src.stamp = 8.0;
    src.addPointWithVariance(PointWithVariance(1.0, 0.0, 5.0, 0.3));
    src.addPointWithVariance(PointWithVariance(0.0, 1.0, -2.0, 0.7));
    std::vector<RefPoint> ref(2);
    ref[0] = RefPoint{1.0, 0.0, 5.0, 0.3};
    ref[1] = RefPoint{0.0, 1.0, -2.0, 0.7};

    PointMatrix copied = src.copyAndTransform(turn);
    CHECK(src.point(0, 0) == 1.0);
    CHECK(src.point(1, 0) == 0.0);
    CHECK(src.point(2, 0) == 5.0);
    CHECK(src.variance(0, 0) == 0.3);
    CHECK(src.point(0, 1) == 0.0);
    CHECK(src.num_point == 2);
    CHECK(copied.stamp == 8.0);
    CHECK(copied.num_point == 2);
    // (1, 0, 5) -> (1, 3, 8), (0, 1, -2) -> (0, 2, 1)
    CHECK(close_to(copied.point(0, 0), 1.0));
    CHECK(close_to(copied.point(1, 0), 3.0));
    CHECK(close_to(copied.point(2, 0), 8.0));
    CHECK(close_to(copied.point(0, 1), 0.0));
    CHECK(close_to(copied.point(1, 1), 2.0));
    CHECK(close_to(copied.point(2, 1), 1.0));
    CHECK(copied.variance(0, 0) == 0.3);
    CHECK(copied.variance(0, 1) == 0.7);
    expect_transformed(copied, ref, turn);

    PointMatrix moved(src);
    moved.transformPoints(turn);
    expect_transformed(moved, ref, turn);
    CHECK(moved.stamp == src.stamp);
    CHECK(close_to(moved.point(0, 0), copied.point(0, 0)));
    CHECK(close_to(moved.point(1, 1), copied.point(1, 1)));

    Transf general = Transf::Identity();
    general.block<3, 3>(0, 0) =
        Eigen::AngleAxisd(0.37, Eigen::Vector3d(1.0, 2.0, 3.0).normalized()).toRotationMatrix();
    general(0, 3) = -4.5;
    general(1, 3) = 0.25;
    general(2, 3) = 6.0;

    PointMatrix src2;
    std::vector<RefPoint> ref2;
    fill_n(src2, ref2, 6);
    PointMatrix copied2 = src2.copyAndTransform(general);
    PointMatrix moved2(src2);
    moved2.transformPoints(general);
    expect_transformed(copied2, ref2, general);
    expect_transformed(moved2, ref2, general);
    check_values(src2, ref2);

    PointMatrix empty;
    Transf identity = Transf::Identity();
    PointMatrix empty_copy = empty.copyAndTransform(identity);
    CHECK(empty_copy.num_point == 0);
    empty.transformPoints(identity);
    CHECK(empty.num_point == 0);
}

static void test_eigen() {
    PointMatrix plane;
    plane.addPoint(Point(0.0, 0.0, 0.0));
    plane.addPoint(Point(1.0, 0.0, 0.0));
    plane.addPoint(Point(0.0, 1.0, 0.0));
    plane.addPoint(Point(1.0, 1.0, 0.0));
    plane.addPoint(Point(0.25, 0.75, 0.0));
    plane.addPoint(Point(0.5, 0.1, 0.0));
    plane.eigenDecomposition();
    CHECK(std::abs(plane.min_value) < 1e-9);
    CHECK(plane.min_vector.norm() > 0.5);
    double align = std::abs(plane.min_vector.normalized().dot(Eigen::Vector3d::UnitZ()));
    CHECK(align > 0.999);
    CHECK(close_to(plane.centroid(2), 0.0));
    double max_other = 0.0;
    for (int i = 0; i < 3; ++i) {
        double ev = plane.eigenvalMat(i, i);
        if (std::abs(ev - plane.min_value) > 1e-8 && ev > max_other) {
            max_other = ev;
        }
    }
    CHECK(max_other > 1e-3);

    // Fixed seed so the noisy plane does not change between runs.
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> xy(-3.0, 3.0);
    std::uniform_real_distribution<double> nz(-0.002, 0.002);
    PointMatrix noisy;
    for (int i = 0; i < 300; ++i) {
        noisy.addPoint(Point(xy(rng), xy(rng), nz(rng)));
    }
    noisy.eigenDecomposition();
    CHECK(noisy.min_value < 1e-3);
    CHECK(noisy.min_value > -1e-6);
    CHECK(noisy.min_vector.norm() > 0.5);
    double noisy_align = std::abs(noisy.min_vector.normalized().dot(Eigen::Vector3d::UnitZ()));
    CHECK(noisy_align > 0.99);
    double noisy_other = 0.0;
    for (int i = 0; i < 3; ++i) {
        double ev = noisy.eigenvalMat(i, i);
        if (ev > noisy_other) {
            noisy_other = ev;
        }
    }
    CHECK(noisy_other > 0.5);
}

static void test_eigen_nonfinite() {
    // One NaN coordinate makes the covariance non-finite. EigenSolver then
    // returns NoConvergence and leaves its matrices uninitialized.
    const double nan = std::numeric_limits<double>::quiet_NaN();
    PointMatrix bad;
    bad.addPoint(Point(1.0, 2.0, 3.0));
    bad.addPoint(Point(0.0, -1.0, 4.0));
    bad.addPoint(Point(nan, 0.5, -2.0));
    bad.eigenDecomposition();
    CHECK(bad.num_point == 3);
    CHECK(bad.point(0, 0) == 1.0);
    CHECK(bad.min_value == -1.0);
    CHECK(bad.min_vector.isZero());
    CHECK(bad.eigenvalMat.isZero());
    CHECK(bad.eigenvectorMat.isZero());
    CHECK(bad.eig_sorted.empty());

    // Inf can make EigenSolver report NumericalIssue instead. Same cleared state.
    PointMatrix infinite;
    infinite.addPoint(Point(0.0, 0.0, 0.0));
    infinite.addPoint(Point(std::numeric_limits<double>::infinity(), 1.0, -1.0));
    infinite.eigenDecomposition();
    CHECK(infinite.min_value == -1.0);
    CHECK(infinite.min_vector.isZero());
    CHECK(infinite.eigenvalMat.isZero());
    CHECK(infinite.eigenvectorMat.isZero());
    CHECK(infinite.eig_sorted.empty());
}

static void test_print_variance() {
    PointMatrix defaults;
    for (int i = 0; i < 5; ++i) {
        defaults.addPoint(Point(i * 1.0, 0.0, 0.0));
    }
    std::stringstream out;
    {
        StreamRedirect redirect(std::cout, out);
        defaults.printVariance();
    }
    const std::string text = out.str();
    CHECK(text.find("num_point: 5") != std::string::npos);
    CHECK(text.find("max 100") != std::string::npos);
    CHECK(text.find("min 100") != std::string::npos);
    CHECK(text.find("0%0%0%0%0%0%0%0%0%100%") != std::string::npos);

    PointMatrix empty;
    std::stringstream out_empty;
    {
        StreamRedirect redirect(std::cout, out_empty);
        empty.printVariance();
    }
    const std::string empty_text = out_empty.str();
    CHECK(empty_text.find("num_point: 0") != std::string::npos);
    CHECK(empty_text.find("0%0%0%0%0%0%0%0%0%0%") != std::string::npos);

    PointMatrix sized_empty(8);
    std::stringstream out_sized;
    {
        StreamRedirect redirect(std::cout, out_sized);
        sized_empty.printVariance();
    }
    CHECK(out_sized.str().find("num_point: 0") != std::string::npos);

    // Negative variance and variance >= 1 must stay inside the 10 buckets.
    PointMatrix mixed;
    mixed.addPointWithVariance(PointWithVariance(0, 0, 0, -3.0));
    mixed.addPointWithVariance(PointWithVariance(1, 0, 0, 0.15));
    mixed.addPointWithVariance(PointWithVariance(2, 0, 0, 100.0));
    mixed.addPointWithVariance(PointWithVariance(3, 0, 0, 1.0));
    mixed.addPointWithVariance(PointWithVariance(4, 0, 0, 5.0));
    std::stringstream out_mixed;
    {
        StreamRedirect redirect(std::cout, out_mixed);
        mixed.printVariance();
    }
    CHECK(out_mixed.str().find("20%20%0%0%0%0%0%0%0%60%") != std::string::npos);
}

static void test_random(int iterations) {
    std::mt19937 rng(1);
    std::uniform_real_distribution<double> coord(-50.0, 50.0);
    std::uniform_real_distribution<double> var_dist(-0.2, 1.7);
    std::uniform_int_distribution<int> op_dist(0, 2);
    std::uniform_int_distribution<int> batch_dist(0, 80);

    PointMatrix acc;
    std::vector<RefPoint> ref;
    for (int n = 0; n < iterations; ++n) {
        int op = op_dist(rng);
        if (op == 0) {
            Point p(coord(rng), coord(rng), coord(rng));
            push_plain(acc, ref, p);
        } else if (op == 1) {
            RefPoint r;
            r.x = coord(rng);
            r.y = coord(rng);
            r.z = coord(rng);
            r.v = var_dist(rng);
            push_with_variance(acc, ref, r);
        } else {
            int batch = batch_dist(rng);
            PointMatrix extra;
            for (int i = 0; i < batch; ++i) {
                RefPoint r;
                r.x = coord(rng);
                r.y = coord(rng);
                r.z = coord(rng);
                r.v = var_dist(rng);
                PointWithVariance pv;
                pv << r.x, r.y, r.z, r.v;
                extra.addPointWithVariance(pv);
                ref.push_back(r);
            }
            acc += extra;
        }
    }
    check_values(acc, ref);
    check_free_capacity(acc);
}

int main(int argc, char** argv) {
    int iterations = 200;
    if (argc > 1) {
        char* end = 0;
        long parsed = std::strtol(argv[1], &end, 10);
        if (end == argv[1] || *end != '\0' || parsed < 1 || parsed > 10000000L) {
            std::cerr << "usage: test_point_matrix [iterations]\n";
            return 2;
        }
        iterations = static_cast<int>(parsed);
    }

    test_constructors();
    test_add_growth();
    test_plus_equal();
    test_copy();
    test_assign_matrix();
    test_get_point();
    test_transform();
    test_eigen();
    test_eigen_nonfinite();
    test_print_variance();
    test_random(iterations);

    if (g_failures != 0) {
        std::cerr << g_failures << " checks failed\n";
        return 1;
    }
    std::cout << "ok " << iterations << "\n";
    return 0;
}
