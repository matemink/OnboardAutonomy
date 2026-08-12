#include "onboard_autonomy/bootstrap/Program.hpp"

#include <exception>
#include <iostream>

int main(const int argc, char** argv) {
    try {
        return onboard_autonomy::bootstrap::run_program(argc, argv);
    } catch (const std::exception& error) {
        std::cerr << "OnboardAutonomy error: " << error.what() << '\n';
        return 1;
    }
}
