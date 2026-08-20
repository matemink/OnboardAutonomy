#include "TestCases.hpp"

#include "onboard_autonomy/hardware/transport/TransportFactory.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>

#ifndef _WIN32
#include <fcntl.h>
#include <unistd.h>
#endif

namespace {

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void quiet_udp_reads_return_immediately() {
    auto transport =
        onboard_autonomy::hardware::transport::make_udp_transport("127.0.0.1",
            0);
    std::array<std::uint8_t, 512> buffer{};

    const auto started_at = std::chrono::steady_clock::now();
    for (int attempt = 0; attempt < 10; ++attempt) {
        require(transport->read(buffer) == 0,
            "quiet UDP transport must report no available bytes");
    }
    const auto elapsed = std::chrono::steady_clock::now() - started_at;

    require(elapsed < std::chrono::milliseconds(500),
        "quiet UDP reads must not inherit a receive timeout");
}

#ifndef _WIN32

class PseudoTerminal {
  public:
    PseudoTerminal() {
        master_ = posix_openpt(O_RDWR | O_NOCTTY | O_CLOEXEC | O_NONBLOCK);
        require(master_ >= 0, "failed to create pseudo-terminal");
        require(grantpt(master_) == 0, "failed to grant pseudo-terminal");
        require(unlockpt(master_) == 0, "failed to unlock pseudo-terminal");
        const char* slave = ptsname(master_);
        require(slave != nullptr, "failed to resolve pseudo-terminal path");
        slave_ = slave;
    }

    ~PseudoTerminal() {
        if (master_ >= 0) {
            close(master_);
        }
    }

    PseudoTerminal(const PseudoTerminal&) = delete;
    PseudoTerminal& operator=(const PseudoTerminal&) = delete;

    [[nodiscard]] const std::string& slave() const { return slave_; }

    [[nodiscard]] int master() const { return master_; }

    void disconnect() {
        close(master_);
        master_ = -1;
    }

  private:
    int master_{-1};
    std::string slave_;
};

void serial_transport_reopens_a_stable_device_path() {
    const auto temporary =
        std::filesystem::temp_directory_path() /
        ("onboard-autonomy-serial-" + std::to_string(getpid()));
    std::filesystem::remove(temporary);

    PseudoTerminal first;
    std::filesystem::create_symlink(first.slave(), temporary);
    auto transport =
        onboard_autonomy::hardware::transport::make_serial_transport(
            temporary.string(),
            115200,
            std::chrono::milliseconds(5));
    std::array<std::uint8_t, 16> buffer{};
    const std::array<std::uint8_t, 3> first_message{1, 2, 3};
    require(
        ::write(first.master(), first_message.data(), first_message.size()) ==
            static_cast<ssize_t>(first_message.size()),
        "failed to feed the first serial session");
    require(transport->read(buffer) == first_message.size(),
        "serial transport did not read the first session");

    first.disconnect();
    require(transport->read(buffer) == 0, "serial hangup must be non-fatal");

    PseudoTerminal second;
    std::filesystem::remove(temporary);
    std::filesystem::create_symlink(second.slave(), temporary);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    const std::array<std::uint8_t, 4> second_message{4, 5, 6, 7};
    require(transport->write(second_message) == second_message.size(),
        "serial transport did not reopen the stable device path");
    require(::read(second.master(), buffer.data(), buffer.size()) ==
                static_cast<ssize_t>(second_message.size()),
        "reconnected serial bytes did not reach the new device");

    transport.reset();
    std::filesystem::remove(temporary);
}

#endif

} // namespace

void run_transport_tests() {
    quiet_udp_reads_return_immediately();
#ifndef _WIN32
    serial_transport_reopens_a_stable_device_path();
#endif
}
