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

// Owns the companion-computer adapters and their application instance.
class CompanionSystem {
  public:
    CompanionSystem(const presentation::cli::CommandLineOptions& options,
        const std::filesystem::path& executable);
    ~CompanionSystem();

    CompanionSystem(const CompanionSystem&) = delete;
    CompanionSystem& operator=(const CompanionSystem&) = delete;
    CompanionSystem(CompanionSystem&&) = delete;
    CompanionSystem& operator=(CompanionSystem&&) = delete;

    [[nodiscard]] application::CompanionApplication& application();
    [[nodiscard]] application::ports::Transport& transport();
    [[nodiscard]] const presentation::BoardTypeResolver*
    board_type_resolver() const;

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace onboard_autonomy::bootstrap
