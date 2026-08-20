#include "TestCases.hpp"

#include "onboard_autonomy/operator/ui/screen/BoardTypeCatalog.hpp"
#include "onboard_autonomy/mission/AppSnapshot.hpp"
#include "onboard_autonomy/operator/ui/screen/ConsoleView.hpp"

#include <filesystem>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

void require(const bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void parser_preserves_aliases_and_prefers_reserved_names() {
    std::istringstream input{"TARGET_HW_FIRST 9\n"
                             "TARGET_HW_SECOND 9 # same numeric ID\n"
                             "Reserved \"Friendly Board\" 9\n"
                             "Reserved Unquoted Friendly Board 56\n"};

    const auto catalog =
        onboard_autonomy::operator_interface::ui::BoardTypeCatalog::from_stream(
            input);

    const auto ambiguous = catalog.resolve(9);
    require(ambiguous.has_value() &&
                ambiguous->preferred_name == "Friendly Board" &&
                ambiguous->aliases.size() == 3,
        "reserved display name must win without losing aliases");

    const auto unquoted = catalog.resolve(56);
    require(unquoted.has_value() &&
                unquoted->preferred_name == "Unquoted Friendly Board",
        "unquoted multi-word reserved names must be parsed");
    require(!catalog.resolve(65000).has_value(),
        "unknown board IDs must remain unknown");
}

void pinned_ardupilot_table_is_complete() {
    const auto path = std::filesystem::path{ONBOARD_AUTONOMY_SOURCE_DIR} /
                      "third_party" / "ardupilot" / "board_types.txt";
    const auto catalog =
        onboard_autonomy::operator_interface::ui::BoardTypeCatalog::from_file(
            path);

    require(catalog.alias_count() == 369,
        "the pinned ArduPilot table must keep all 369 rows");
    require(catalog.board_type_count() == 358,
        "the pinned table must expose all 358 unique board IDs");

    const auto pixhawk6c = catalog.resolve(56);
    require(pixhawk6c.has_value() &&
                pixhawk6c->preferred_name == "PX4 [BL] FMU v6C.x",
        "board type 56 must come from the table, not a hardcode");

    const auto ambiguous = catalog.resolve(9);
    require(ambiguous.has_value() && ambiguous->aliases.size() >= 3,
        "ambiguous official board IDs must retain every alias");
}

void ambiguous_board_is_honest_in_the_console() {
    onboard_autonomy::mission::AppSnapshot snapshot;
    snapshot.vehicle.connected = true;
    snapshot.vehicle.autopilot_metadata =
        onboard_autonomy::mission::AutopilotMetadata{
            .board_version = 9U << 16U,
        };

    std::istringstream table{"TARGET_HW_PX4_FMU_V2 9\n"
                             "TARGET_HW_PX4_FMU_V3 9\n"
                             "TARGET_HW_CUBE_F4 9\n"};
    const auto catalog =
        onboard_autonomy::operator_interface::ui::BoardTypeCatalog::from_stream(
            table);
    const auto output =
        onboard_autonomy::operator_interface::ui::render_console(snapshot,
            "fake://transport",
            false,
            &catalog);

    require(
        output.find("TARGET_HW_PX4_FMU_V2 / ID 9 / SILICON 0 / 3 ALIASES") !=
            std::string::npos,
        "ambiguous IDs must not be displayed as an exact model");
}

} // namespace

void run_board_type_catalog_tests() {
    parser_preserves_aliases_and_prefers_reserved_names();
    pinned_ardupilot_table_is_complete();
    ambiguous_board_is_honest_in_the_console();
}
