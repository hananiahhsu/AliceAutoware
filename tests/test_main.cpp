#include "test_framework.hpp"

#include <exception>
#include <iostream>

int main() {
    std::size_t passed = 0;
    std::size_t failed = 0;

    for (const auto& test : mad::tests::TestRegistry::Instance().tests()) {
        try {
            test.fn();
            ++passed;
            std::cout << "[PASS] " << test.suite << "." << test.name << "\n";
        } catch (const std::exception& ex) {
            ++failed;
            std::cerr << "[FAIL] " << test.suite << "." << test.name << " : " << ex.what() << "\n";
        }
    }

    std::cout << "[MAD TESTS] passed=" << passed << " failed=" << failed << "\n";
    return failed == 0 ? 0 : 1;
}
