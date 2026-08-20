#pragma once

#include "onboard_autonomy/application/ports/RuntimeSnapshotSink.hpp"

#include <filesystem>
#include <iosfwd>
#include <memory>

namespace onboard_autonomy::diagnostics::logging {

// Writes backward-compatible snapshots plus transition events as JSON Lines.
class JsonDiagnosticSink final
    : public application::ports::RuntimeSnapshotSink {
  public:
    explicit JsonDiagnosticSink(std::ostream& output);
    explicit JsonDiagnosticSink(const std::filesystem::path& output_file);
    ~JsonDiagnosticSink() override;

    JsonDiagnosticSink(const JsonDiagnosticSink&) = delete;
    JsonDiagnosticSink& operator=(const JsonDiagnosticSink&) = delete;

    void consume(const application::AppSnapshot& snapshot,
        std::chrono::system_clock::time_point recorded_at) override;

  private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace onboard_autonomy::diagnostics::logging
