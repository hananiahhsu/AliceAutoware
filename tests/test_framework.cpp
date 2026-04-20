#include "test_framework.hpp"

namespace mad::tests {

TestRegistry& TestRegistry::Instance() {
    static TestRegistry registry;
    return registry;
}

void TestRegistry::Add(TestCase test_case) {
    m_tests.push_back(std::move(test_case));
}

TestRegistrar::TestRegistrar(const char* suite, const char* name, std::function<void()> fn) {
    TestRegistry::Instance().Add({suite, name, std::move(fn)});
}

[[noreturn]] void Fail(const std::string& message) {
    throw std::runtime_error(message);
}

} // namespace mad::tests
