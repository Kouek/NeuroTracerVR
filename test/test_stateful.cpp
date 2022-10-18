#include <cassert>
#include <iostream>

#include <array>

#include <stateful/stateful.hpp>

void testStatic() {
    // f0: {0,1} -> {2,3,4}
    // f1: {0,2} -> {5,6}
    // f2: {5} -> {6}
    // Topo sort:
    // {0,1}, {2,3,4}, {5}, {6}
    // Function sort:
    // f0, f1, f2

    std::array<int, 7> vars{}; // all 0
    kouek::StatefulSystem sys;
    sys.Register(
        std::tie(vars[0], vars[1]),
        [&]() {
            vars[2] = vars[0] + 1;
            vars[3] = vars[1] + 1;
            vars[4] = vars[0] + vars[1] + 1;
        },
        std::tie(vars[2], vars[3], vars[4]));
    sys.Register(
        std::tie(vars[5]), [&]() { vars[6] = vars[5] + vars[6]; },
        std::tie(vars[6]));
    sys.Register(
        std::tie(vars[0], vars[2]),
        [&]() {
            vars[5] = vars[0] + 2;
            vars[6] = vars[2] + 2;
        },
        std::tie(vars[5], vars[6]));
    sys.Register(
        [&]() {
            vars[0] = 0;
            vars[1] = 1;
        },
        std::tie(vars[0], vars[1]));

    sys.Update();

    std::cout << __FUNCTION__ << ':' << std::endl;
    decltype(vars) answer{0, 1, 1, 2, 2, 2, 5};
    for (const auto val : vars)
        std::cout << val << ' ';
    assert(vars == answer);
    std::cout << std::endl;
}

void testDynamic() {
    // stage0:
    // f0: {0,1} -> {2,3,4}
    // f1: {1,4} -> {5,6}
    // Topo sort:
    // {0,1}, {2,3,4}, {5,6}
    // Function sort:
    // f0, f1

    // stage1:
    // f2: {0} -> {8}
    // Topo sort:
    // {0,1}, {2,3,4,8}, {5,6}
    // Function sort:
    // {f0, f2}, f1

    std::array<int, 8> vars{}; // all 0
    kouek::StatefulSystem sys;
    sys.Register(
        [&]() {
            vars[0] = 0;
            vars[1] = 1;
        },
        std::tie(vars[0], vars[1]));
    sys.Register(
        std::tie(vars[0], vars[1]),
        [&]() {
            vars[2] = vars[0] * 2;
            vars[3] = vars[1] * 2;
            vars[4] = (vars[0] + vars[1]) * 2;
        },
        std::tie(vars[2], vars[3], vars[4]));
    sys.Register(
        std::tie(vars[1], vars[4]),
        [&]() {
            vars[5] = vars[1] + 2;
            vars[6] = vars[4] + 2;
        },
        std::tie(vars[5], vars[6]));

    sys.Update();

    sys.Register(
        std::tie(vars[0]), [&]() { vars[7] = vars[0] + 8; }, std::tie(vars[7]));

    sys.Update();

    std::cout << __FUNCTION__ << ':' << std::endl;
    decltype(vars) answer{0, 1, 0, 2, 2, 3, 4, 8};
    for (const auto val : vars)
        std::cout << val << ' ';
    assert(vars == answer);
    std::cout << std::endl;
}

static void testDuplicate() {
    // f0: {0,1} -> {3,4}
    // f1: {1,2} -> {3,4}
    // f2: {4} -> {5}
    // Topo sort:
    // {0,1,2}, {3,4}, {5}
    // Function sort:
    // {f0, f1}, f2

    std::array<int, 6> vars{}; // all 0
    kouek::StatefulSystem sys;
    sys.Register(
        std::tie(vars[0], vars[1]), [&]() { vars[3] = vars[0] + vars[1] + 1; },
        std::tie(vars[3], vars[4]));
    sys.Register(
        std::tie(vars[1], vars[2]), [&]() { vars[4] = vars[1] + vars[2] + 1; },
        std::tie(vars[3], vars[4]));
    sys.Register(
        std::tie(vars[4]), [&]() { vars[5] = 2 * vars[4]; }, std::tie(vars[5]));
    sys.Register(
        [&]() {
            vars[0] = 0;
            vars[1] = 1;
            vars[2] = 2;
        },
        std::tie(vars[0], vars[1], vars[2]));

    sys.Update();

    std::cout << __FUNCTION__ << ':' << std::endl;
    decltype(vars) answer{0, 1, 2, 2, 4, 8};
    for (const auto val : vars)
        std::cout << val << ' ';
    assert(vars == answer);
    std::cout << std::endl;
}

int main() {
    testStatic();
    testDynamic();
    testDuplicate();
    return 0;
}
