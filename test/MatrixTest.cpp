#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "Matrix.h"

TEST_CASE("Construct Matrix") {
    using namespace graphlib;
    SECTION("Default constructor") {
        Matrix<int> mtx;
        auto [i, j] = mtx.size();
        REQUIRE(i == 0);
        REQUIRE(j == 0);
        REQUIRE(mtx.empty());
    }
    SECTION("Constructor with x and y") {
        Matrix<int> mtx(6, 7);
        auto [i, j] = mtx.size();
        REQUIRE(i == 6);
        REQUIRE(j == 7);
        REQUIRE(mtx.empty());
    }
    SECTION("Constructor with x, y and value") {
        Matrix<int> mtx(6, 7, 1);
        auto [i, j] = mtx.size();
        REQUIRE(i == 6);
        REQUIRE(j == 7);
        REQUIRE_FALSE(mtx.empty());
        for (auto val : mtx)
            REQUIRE(val == 1);
    }
    SECTION("Copy constructor") {
        Matrix<int> mtx(6, 7, 1);
        Matrix<int> mtx_cp(mtx);
        REQUIRE(mtx == mtx_cp);
        for (auto val : mtx)
            REQUIRE(val == 1);
    }
    SECTION("Move constructor") {
        Matrix<int> mtx(6, 7, 1);
        Matrix<int> mtx_mv(std::move(mtx));
        auto [i, j] = mtx_mv.size();
        REQUIRE(i == 6);
        REQUIRE(j == 7);
        for (auto val : mtx)
            REQUIRE(val == 1);
    }
    SECTION("Copy assignment") {
        Matrix<int> mtx_a(6, 7, 1);
        Matrix<int> mtx_b;
        mtx_b = mtx_a;
        REQUIRE(mtx_a == mtx_b);
        auto [i, j] = mtx_b.size();
        REQUIRE(i == 6);
        REQUIRE(j == 7);
        REQUIRE_FALSE(mtx_b.empty());
    }
    SECTION("Move assignment") {
        Matrix<int> mtx_a(6, 7, 1);
        Matrix<int> mtx_b;
        mtx_b = std::move(mtx_a);
        auto [i, j] = mtx_b.size();
        REQUIRE(i == 6);
        REQUIRE(j == 7);
        REQUIRE_FALSE(mtx_b.empty());
    }
}

TEST_CASE("size method") {
    using namespace graphlib;
    Matrix<int> mtx_a;
    auto [i, j] = mtx_a.size();
    REQUIRE(i == 0);
    REQUIRE(j == 0);

    Matrix<int> mtx_b(3, 4);
    auto [k, l] = mtx_b.size();
    REQUIRE(k == 3);
    REQUIRE(l == 4);
}

TEST_CASE("empty") {
    using namespace graphlib;
    Matrix<int> mtx_a(3, 3);
    Matrix<int> mtx_b(3, 3, 1);
    REQUIRE(mtx_a.empty());
    REQUIRE_FALSE(mtx_b.empty());
}

TEST_CASE("access method") {
    using namespace graphlib;
    Matrix<int> mtx_a(3, 3);
    mtx_a[2][2] = 1;
    auto ref_a = mtx_a.access(2, 2);
    REQUIRE(ref_a == 1);

    const Matrix<int> &mtx_b = const_cast<Matrix<int> &>(mtx_a);
    auto ref_b = mtx_b.access(2, 2);
    REQUIRE(ref_b == 1);
}

TEST_CASE("operator[]") {
    using namespace graphlib;
    Matrix<int> mtx_a(3, 3);
    REQUIRE(mtx_a[2][2] == 0);
    mtx_a[2][2] = 1;
    REQUIRE(mtx_a.access(2, 2) == 1);

    const Matrix<int> &mtx_b = const_cast<Matrix<int> &>(mtx_a);
    REQUIRE(mtx_b[2][2] == 1);
}

TEST_CASE("operator==/operator!=") {
    using namespace graphlib;
    Matrix<int> mtx_a(3, 3);
    Matrix<int> mtx_b(mtx_a);
    Matrix<int> mtx_c(3, 4);
    Matrix<int> mtx_d(3, 3);
    mtx_d[2][2] = 1;
    REQUIRE(mtx_a == mtx_b);
    REQUIRE(mtx_a != mtx_c);
    REQUIRE(mtx_a != mtx_d);
}
