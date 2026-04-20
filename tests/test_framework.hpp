#pragma once

#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

namespace mad::tests {

struct TestCase {
    std::string suite;
    std::string name;
    std::function<void()> fn;
};

class TestRegistry {
public:
    static TestRegistry& Instance();
    void Add(TestCase test_case);
    const std::vector<TestCase>& tests() const { return m_tests; }

private:
    std::vector<TestCase> m_tests;
};

class TestRegistrar {
public:
    TestRegistrar(const char* suite, const char* name, std::function<void()> fn);
};

[[noreturn]] void Fail(const std::string& message);

} // namespace mad::tests

#define MAD_TEST(SUITE, NAME) \
    static void SUITE##_##NAME(); \
    static ::mad::tests::TestRegistrar SUITE##_##NAME##_registrar(#SUITE, #NAME, SUITE##_##NAME); \
    static void SUITE##_##NAME()

#define MAD_REQUIRE(CONDITION) \
    do { \
        if (!(CONDITION)) { \
            ::mad::tests::Fail(std::string("require failed: ") + #CONDITION + " @ " + __FILE__ + ":" + std::to_string(__LINE__)); \
        } \
    } while (false)

#define MAD_REQUIRE_NEAR(A, B, EPS) \
    do { \
        const auto mad_value_a = (A); \
        const auto mad_value_b = (B); \
        const auto mad_eps = (EPS); \
        if ((mad_value_a < mad_value_b ? mad_value_b - mad_value_a : mad_value_a - mad_value_b) > mad_eps) { \
            ::mad::tests::Fail(std::string("require near failed: ") + #A + " vs " + #B + " @ " + __FILE__ + ":" + std::to_string(__LINE__)); \
        } \
    } while (false)
