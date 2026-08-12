#include "onboard_autonomy/adapters/transport/TransportFactory.hpp"

#include "../PosixError.hpp"

#include <cerrno>
#include <chrono>
#include <stdexcept>
#include <string>

#ifndef _WIN32
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace onboard_autonomy::adapters::transport {
namespace {

#ifndef _WIN32

speed_t to_posix_baud(const std::uint32_t baud_rate) {
    switch (baud_rate) {
        case 57600:
            return B57600;
        case 115200:
            return B115200;
#ifdef B460800
        case 460800:
            return B460800;
#endif
#ifdef B921600
        case 921600:
            return B921600;
#endif
        default:
            throw std::invalid_argument("Unsupported serial baud rate");
    }
}

int open_serial_device(
    const std::string& device,
    const std::uint32_t baud_rate
) {
    const speed_t baud = to_posix_baud(baud_rate);
    const int file_descriptor = open(
        device.c_str(),
        O_RDWR | O_NOCTTY | O_CLOEXEC | O_NONBLOCK
    );
    if (file_descriptor < 0) {
        throw std::runtime_error(
            "Unable to open serial device " + device + ": " +
            posix_error_message(errno)
        );
    }

    termios options{};
    if (tcgetattr(file_descriptor, &options) != 0) {
        const std::string error = posix_error_message(errno);
        close(file_descriptor);
        throw std::runtime_error(
            "Unable to read serial settings for " + device + ": " +
            error
        );
    }

    cfmakeraw(&options);
    cfsetispeed(&options, baud);
    cfsetospeed(&options, baud);
    options.c_cflag |= CLOCAL | CREAD;
    const auto clear_control_flag = [&options](const tcflag_t flag) {
        options.c_cflag &= ~flag;
    };
    clear_control_flag(static_cast<tcflag_t>(CSTOPB));
    clear_control_flag(static_cast<tcflag_t>(CRTSCTS));
    clear_control_flag(static_cast<tcflag_t>(PARENB));
    clear_control_flag(static_cast<tcflag_t>(CSIZE));
    options.c_cflag |= CS8;
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    if (tcsetattr(file_descriptor, TCSANOW, &options) != 0) {
        const std::string error = posix_error_message(errno);
        close(file_descriptor);
        throw std::runtime_error(
            "Unable to configure serial device " + device + ": " +
            error
        );
    }
    return file_descriptor;
}

class PosixSerialTransport final : public application::ports::Transport {
public:
    PosixSerialTransport(
        std::string device,
        const std::uint32_t baud_rate,
        const std::chrono::milliseconds reconnect_interval
    )
        : device_(std::move(device)),
          baud_rate_(baud_rate),
          reconnect_interval_(reconnect_interval) {
        if (reconnect_interval_ < std::chrono::milliseconds::zero()) {
            throw std::invalid_argument(
                "Serial reconnect interval cannot be negative"
            );
        }
        file_descriptor_ = open_serial_device(device_, baud_rate_);
    }

    ~PosixSerialTransport() override {
        if (file_descriptor_ >= 0) {
            close(file_descriptor_);
        }
    }

    std::size_t read(std::span<std::uint8_t> destination) override {
        if (!ensure_connected()) {
            return 0;
        }

        // A quiet VMIN=0 read and a disconnected tty can both return zero;
        // poll exposes device hangup without blocking the application loop.
        pollfd descriptor{
            .fd = file_descriptor_,
            .events = POLLIN,
            .revents = 0,
        };
        const int poll_result = ::poll(&descriptor, 1, 0);
        if (poll_result < 0) {
            if (errno == EINTR) {
                return 0;
            }
            disconnect();
            return 0;
        }
        if (poll_result == 0) {
            return 0;
        }
        if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            disconnect();
            return 0;
        }
        if ((descriptor.revents & POLLIN) == 0) {
            return 0;
        }

        const auto received = ::read(
            file_descriptor_,
            destination.data(),
            destination.size()
        );
        if (received < 0) {
            if (errno == EAGAIN || errno == EINTR) {
                return 0;
            }
            disconnect();
            return 0;
        }
        return static_cast<std::size_t>(received);
    }

    std::size_t write(std::span<const std::uint8_t> source) override {
        if (!ensure_connected()) {
            return 0;
        }
        const auto sent = ::write(
            file_descriptor_,
            source.data(),
            source.size()
        );
        if (sent < 0) {
            if (errno == EAGAIN || errno == EINTR) {
                return 0;
            }
            disconnect();
            return 0;
        }
        return static_cast<std::size_t>(sent);
    }

    [[nodiscard]] std::string description() const override {
        return "serial://" + device_ + "?baud=" +
               std::to_string(baud_rate_);
    }

private:
    bool ensure_connected() {
        if (file_descriptor_ >= 0) {
            return true;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now < next_reconnect_attempt_) {
            return false;
        }
        try {
            file_descriptor_ = open_serial_device(device_, baud_rate_);
            return true;
        } catch (const std::runtime_error&) {
            next_reconnect_attempt_ = now + reconnect_interval_;
            return false;
        }
    }

    void disconnect() {
        if (file_descriptor_ >= 0) {
            close(file_descriptor_);
            file_descriptor_ = -1;
        }
        next_reconnect_attempt_ =
            std::chrono::steady_clock::now() + reconnect_interval_;
    }

    std::string device_;
    std::uint32_t baud_rate_;
    std::chrono::milliseconds reconnect_interval_;
    std::chrono::steady_clock::time_point next_reconnect_attempt_{};
    int file_descriptor_{-1};
};

#endif

}  // namespace

std::unique_ptr<application::ports::Transport> make_serial_transport(
    const std::string& device,
    const std::uint32_t baud_rate,
    const std::chrono::milliseconds reconnect_interval
) {
#ifdef _WIN32
    static_cast<void>(device);
    static_cast<void>(baud_rate);
    static_cast<void>(reconnect_interval);
    throw std::runtime_error(
        "Serial transport is currently supported on Linux only"
    );
#else
    return std::make_unique<PosixSerialTransport>(
        device,
        baud_rate,
        reconnect_interval
    );
#endif
}

}  // namespace onboard_autonomy::adapters::transport
