#include <array>
#include <csignal>
#include <fstream>
#include <iostream>
#include <string_view>

#include <sys/prctl.h>
#include <unistd.h>

int main(const int argc, char* argv[]) {
    // Do not leave a fixture behind if CTest terminates a hung regression test.
    if (::prctl(PR_SET_PDEATHSIG, SIGKILL) != 0 || ::getppid() == 1) {
        return 1;
    }
    std::signal(SIGINT, SIG_IGN);
    std::signal(SIGTERM, SIG_IGN);
    for (int index = 1; index + 1 < argc; ++index) {
        if (std::string_view{argv[index]} == "--metadata") {
            std::ofstream metadata{argv[index + 1]};
            metadata << "\"FrameWallClock\": 1785440325818936064\n";
        }
    }
    // One packed 8x2 I420 frame proves the child installed its signal handlers.
    const std::array<char, 24> frame{};
    std::cout.write(frame.data(), static_cast<std::streamsize>(frame.size()));
    std::cout.flush();
    for (;;) {
        ::pause();
    }
}
