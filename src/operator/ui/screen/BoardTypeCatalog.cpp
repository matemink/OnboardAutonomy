#include "onboard_autonomy/operator/ui/screen/BoardTypeCatalog.hpp"

#include <algorithm>
#include <charconv>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

namespace onboard_autonomy::operator_interface::ui {
namespace {

std::string_view trim(const std::string_view value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string_view::npos) {
        return {};
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

std::pair<std::string, bool> normalized_name(std::string_view source_name) {
    source_name = trim(source_name);
    bool is_reserved = false;
    constexpr std::string_view reserved_prefix{"Reserved"};
    if (source_name.starts_with(reserved_prefix)) {
        source_name = trim(source_name.substr(reserved_prefix.size()));
        is_reserved = true;
    }

    if (source_name.size() >= 2 && source_name.front() == '"' &&
        source_name.back() == '"') {
        source_name = source_name.substr(1, source_name.size() - 2);
    }

    return {std::string(trim(source_name)), is_reserved};
}

struct ParsedLine {
    std::uint16_t board_type;
    std::string name;
    bool is_reserved;
};

std::optional<ParsedLine> parse_line(std::string line,
    const std::size_t line_number) {
    if (const auto comment = line.find('#'); comment != std::string::npos) {
        line.erase(comment);
    }

    const std::string_view content = trim(line);
    if (content.empty()) {
        return std::nullopt;
    }

    const auto separator = content.find_last_of(" \t");
    if (separator == std::string_view::npos) {
        throw std::runtime_error("ArduPilot board table line " +
                                 std::to_string(line_number) +
                                 " has no numeric board type");
    }

    const std::string_view id_text = trim(content.substr(separator + 1));
    const std::string_view name_text = trim(content.substr(0, separator));

    std::uint32_t parsed_id{0};
    const auto [end, error] = std::from_chars(id_text.data(),
        id_text.data() + id_text.size(),
        parsed_id);
    if (error != std::errc{} || end != id_text.data() + id_text.size() ||
        parsed_id > std::numeric_limits<std::uint16_t>::max()) {
        throw std::runtime_error("ArduPilot board table line " +
                                 std::to_string(line_number) +
                                 " has an invalid board type");
    }

    auto [name, is_reserved] = normalized_name(name_text);
    if (name.empty()) {
        throw std::runtime_error("ArduPilot board table line " +
                                 std::to_string(line_number) +
                                 " has an empty board name");
    }

    return ParsedLine{
        .board_type = static_cast<std::uint16_t>(parsed_id),
        .name = std::move(name),
        .is_reserved = is_reserved,
    };
}

} // namespace

BoardTypeCatalog BoardTypeCatalog::from_file(
    const std::filesystem::path& path) {
    std::ifstream input{path};
    if (!input) {
        throw std::runtime_error(
            "cannot open ArduPilot board table: " + path.string());
    }
    return from_stream(input);
}

BoardTypeCatalog BoardTypeCatalog::from_stream(std::istream& input) {
    BoardTypeCatalog catalog;
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        const auto parsed = parse_line(std::move(line), line_number);
        if (!parsed.has_value()) {
            continue;
        }
        catalog.add(parsed->board_type, parsed->name, parsed->is_reserved);
    }
    return catalog;
}

std::optional<operator_interface::ui::BoardTypeMatch> BoardTypeCatalog::resolve(
    const std::uint16_t board_type) const {
    const auto found = entries_.find(board_type);
    if (found == entries_.end()) {
        return std::nullopt;
    }
    return operator_interface::ui::BoardTypeMatch{
        .preferred_name = found->second.preferred_name,
        .aliases = found->second.aliases,
    };
}

std::size_t BoardTypeCatalog::board_type_count() const {
    return entries_.size();
}

std::size_t BoardTypeCatalog::alias_count() const { return alias_count_; }

void BoardTypeCatalog::add(const std::uint16_t board_type,
    std::string name,
    const bool is_reserved) {
    auto& entry = entries_[board_type];
    const bool duplicate =
        std::find(entry.aliases.begin(), entry.aliases.end(), name) !=
        entry.aliases.end();
    if (!duplicate) {
        entry.aliases.push_back(name);
        ++alias_count_;
    }

    if (entry.preferred_name.empty() ||
        (is_reserved && !entry.preferred_is_reserved)) {
        entry.preferred_name = std::move(name);
        entry.preferred_is_reserved = is_reserved;
    }
}

} // namespace onboard_autonomy::operator_interface::ui
