#pragma once

#include "onboard_autonomy/presentation/cli/CommandLine.hpp"

#include <filesystem>
#include <memory>

namespace onboard_autonomy::application {
class CompanionApplication;
}

namespace onboard_autonomy::application::ports {
class Transport;
}

namespace onboard_autonomy::presentation {
class BoardTypeResolver;
}

namespace onboard_autonomy::bootstrap {

// Owns the runtime adapters and wires them into CompanionApplication.
class RuntimeAssembly {
public:
    RuntimeAssembly(
        const presentation::cli::CommandLineOptions& options,
        const std::filesystem::path& executable
    );
    ~RuntimeAssembly();

    RuntimeAssembly(const RuntimeAssembly&) = delete;
    RuntimeAssembly& operator=(const RuntimeAssembly&) = delete;
    RuntimeAssembly(RuntimeAssembly&&) = delete;
    RuntimeAssembly& operator=(RuntimeAssembly&&) = delete;

    [[nodiscard]] application::CompanionApplication& application();
    [[nodiscard]] application::ports::Transport& transport();
    [[nodiscard]] const presentation::BoardTypeResolver*
    board_type_resolver() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace onboard_autonomy::bootstrap
