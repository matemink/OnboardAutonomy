#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

namespace onboard_autonomy::mission::ports {

class Transport {
  public:
    virtual ~Transport() = default;

    // Reads only bytes already available and returns 0 immediately when
    // the transport is quiet. MAVLink framing belongs to the decoder, so
    // one read may contain a partial frame or several complete frames.
    virtual std::size_t read(std::span<std::uint8_t> destination) = 0;
    virtual std::size_t write(std::span<const std::uint8_t> source) = 0;
    [[nodiscard]] virtual std::string description() const = 0;
};

} // namespace onboard_autonomy::mission::ports
