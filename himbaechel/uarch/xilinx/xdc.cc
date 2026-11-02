/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2019-2023  gatecat <gatecat@ds0.me>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include <boost/algorithm/string.hpp>
#include <fstream>
#include <limits>
#include <regex>

#include "extra_data.h"
#include "himbaechel_api.h"
#include "log.h"
#include "nextpnr.h"
#include "util.h"

#include "xilinx.h"

#define HIMBAECHEL_CONSTIDS "uarch/xilinx/constids.inc"
#include "himbaechel_constids.h"

NEXTPNR_NAMESPACE_BEGIN

void XilinxImpl::parse_xdc(const std::string &filename)
{
    auto in = open_ifstream_and_log_error(filename, "XDC file");
    log_info("Parsing XDC file...\n");
    std::string line;
    std::string linebuf;
    int lineno = 0;
    unsigned num_errors = 0;
    int missing_targets = 0;

    auto isempty = [](const std::string &str) {
        return std::all_of(str.begin(), str.end(), [](char c) { return std::isspace(c); });
    };
    auto strip_quotes = [](const std::string &str) {
        if (str.at(0) == '"') {
            NPNR_ASSERT(str.back() == '"');
            return str.substr(1, str.size() - 2);
        } else if (str.at(0) == '{') {
            NPNR_ASSERT(str.back() == '}');
            return str.substr(1, str.size() - 2);
        } else {
            return str;
        }
    };
    auto split_to_args = [](const std::string &str, bool group_brackets) {
        std::vector<std::string> split_args;
        std::string buffer;
        auto flush = [&]() {
            if (!buffer.empty())
                split_args.push_back(buffer);
            buffer.clear();
        };
        int brcount = 0;
        for (char c : str) {
            if ((c == '[' || c == '{') && group_brackets) {
                ++brcount;
            }
            if ((c == ']' || c == '}') && group_brackets) {
                --brcount;
                buffer += c;
                if (brcount == 0)
                    flush();
                continue;
            }
            if (std::isspace(c)) {
                if (brcount == 0) {
                    flush();
                    continue;
                }
            }
            buffer += c;
        }
        flush();
        return split_args;
    };

    // Vivado calls a one-bit vector port "a[0]", but the JSON frontend
    // collapses a width-1, offset-0 vector to the bare name "a", because
    // yosys' JSON carries nothing that tells `wire [0:0] a` apart from
    // `wire a`.  An XDC written against Vivado names therefore misses such
    // a port.  Strip a trailing "[0]" so the lookup can retry.
    //  (Port of nextpnr-xilinx b257be4d.)
    auto debus_zero = [](const std::string &str) {
        bool ends_with_bit_zero = str.size() > 3 && str.compare(str.size() - 3, 3, "[0]") == 0;
        if (ends_with_bit_zero)
            return str.substr(0, str.size() - 3);
        return std::string();
    };
    auto get_cells = [&](std::string str) {
        std::vector<CellInfo *> tgt_cells;
        if (str.empty() || str.front() != '[')
            log_error("failed to parse target (on line %d)\n", lineno);
        str = str.substr(1, str.size() - 2);
        auto split = split_to_args(str, false);
        if (split.size() < 1)
            log_error("failed to parse target (on line %d)\n", lineno);
        // get_ports names a top-level port, whose pad cell carries the property;
        // get_cells names a cell outright, which is how a design pins a hard
        // block -- an MMCM, a clock buffer, a transceiver -- to the site that
        // is known to work for it.
        bool is_supported_selector = split.front() == "get_ports" || split.front() == "get_cells";
        if (!is_supported_selector)
            log_error("targets other than 'get_ports' or 'get_cells' are not supported (on line %d)\n", lineno);
        if (split.size() < 2)
            log_error("failed to parse target (on line %d)\n", lineno);
        std::string target_name;
        if (split.front() == "get_cells") {
            int cursor = 1;
            while (cursor < int(split.size()) && !split.at(cursor).empty() && split.at(cursor).at(0) == '-') {
                if (split.at(cursor) == "-hier") {
                    ++cursor;
                    continue;
                }
                log_nonfatal_error("unsupported get_cells option '%s' (on line %d)\n", split.at(cursor).c_str(), lineno);
                num_errors++;
                return tgt_cells;
            }
            if (cursor >= int(split.size())) {
                log_nonfatal_error("failed to parse get_cells target (on line %d)\n", lineno);
                num_errors++;
                return tgt_cells;
            }
            bool single_name_follows_options = (cursor + 1) == int(split.size());
            if (!single_name_follows_options) {
                log_nonfatal_error("unsupported get_cells selector form '%s' (on line %d)\n", str.c_str(), lineno);
                num_errors++;
                return tgt_cells;
            }
            target_name = strip_quotes(split.at(cursor));
        } else {
            target_name = strip_quotes(split.at(1));
        }
        IdString cellname = ctx->id(target_name);
        if (!ctx->cells.count(cellname)) {
            std::string base = debus_zero(target_name);
            bool debussed_name_exists = !base.empty() && ctx->cells.count(ctx->id(base));
            if (debussed_name_exists)
                cellname = ctx->id(base);
        }
        if (ctx->cells.count(cellname))
            tgt_cells.push_back(ctx->cells.at(cellname).get());
        else {
            // A board-level XDC legitimately constrains every pin of the
            // board while a design uses a subset, so a miss is not an error:
            // stay silent like other arches do, and itemise only in verbose
            // mode.  (Port of nextpnr-xilinx 3da43687/555d326c.)
            missing_targets++;
            if (ctx->verbose)
                log_info("%s: no cell named '%s' (on line %d) - this target is ignored\n",
                         split.front().c_str(), cellname.c_str(ctx), lineno);
        }
        return tgt_cells;
    };

    auto get_nets = [&](std::string str) {
        std::vector<NetInfo *> tgt_nets;
        if (str.empty())
            return tgt_nets;
        if (str.front() != '[' || str.back() != ']')
            log_error("failed to parse target '%s' (on line %d)\n", str.c_str(), lineno);
        str = str.substr(1, str.size() - 2);
        auto split = split_to_args(str, false);
        if (split.size() < 1)
            log_error("failed to parse target (on line %d)\n", lineno);
        if (split.front() != "get_ports" && split.front() != "get_nets")
            log_error("targets other than 'get_ports' or 'get_nets' are not supported (on line %d)\n", lineno);
        if (split.size() < 2)
            log_error("failed to parse target (on line %d)\n", lineno);
        str = strip_quotes(split.at(1));
        if (str.empty())
            return tgt_nets;
        IdString netname = ctx->id(str);
        NetInfo *maybe_net = ctx->getNetByAlias(netname);
        if (maybe_net == nullptr) {
            // Retry with a trailing "[0]" stripped (see debus_zero above).
            std::string base = debus_zero(str);
            if (!base.empty())
                maybe_net = ctx->getNetByAlias(ctx->id(base));
        }
        if (maybe_net != nullptr) {
            tgt_nets.push_back(maybe_net);
            return tgt_nets;
        }
        // Also test the lowercase variant, for better interoperability with synthesis tools
        boost::algorithm::to_lower(str);
        netname = ctx->id(str);
        maybe_net = ctx->getNetByAlias(netname);
        if (maybe_net != nullptr)
            tgt_nets.push_back(maybe_net);
        else {
            missing_targets++;
            if (ctx->verbose)
                log_info("%s: no net or port named '%s' (on line %d) - this target is ignored\n",
                         split.front().c_str(), netname.c_str(ctx), lineno);
        }
        return tgt_nets;
    };

    while (std::getline(in, line)) {
        ++lineno;
        // Trim comments, from # until end of the line
        size_t cstart = line.find('#');
        if (cstart != std::string::npos)
            line = line.substr(0, cstart);
        if (isempty(line))
            continue;

        std::vector<std::string> arguments = split_to_args(line, true);
        if (arguments.empty())
            continue;
        std::string &cmd = arguments.front();

        if (cmd == "set_property") {
            std::vector<std::pair<std::string, std::string>> arg_pairs;
            if (arguments.size() < 4) {
                log_nonfatal_error("expected at least four arguments to 'set_property' (on line %d)\n", lineno);
                num_errors++;
                goto nextline;
            } else if (arguments.at(1) == "-dict") {
                std::vector<std::string> dict_args = split_to_args(strip_quotes(arguments.at(2)), false);
                if ((dict_args.size() % 2) != 0) {
                    log_nonfatal_error("expected an even number of argument for dictionary (on line %d)\n", lineno);
                    num_errors++;
                    goto nextline;
                }
                arg_pairs.reserve(dict_args.size() / 2);
                for (int cursor = 0; cursor + 1 < int(dict_args.size()); cursor += 2) {
                    arg_pairs.emplace_back(std::move(dict_args.at(cursor)), std::move(dict_args.at(cursor + 1)));
                }
            } else {
                arg_pairs.emplace_back(std::move(arguments.at(1)), std::move(arguments.at(2)));
            }
            // Warning : ug835 has lowercase example, so probably supporting lowercase too is needed
            if (arg_pairs.size() == 1 && arg_pairs.front().first == "INTERNAL_VREF") { // get_iobanks not supported
                log_warning("INTERNAL_VREF isn't supported, ignoring (on line %d)\n", lineno);
                continue;
            }
            if (arguments.at(3).size() > 2 && arguments.at(3) == "[current_design]") {
                log_warning("[current_design] isn't supported, ignoring (on line %d)\n", lineno);
                continue;
            }
            // All remaining arguments are supposed to designate cells
            std::vector<CellInfo *> dest;
            for (int cursor = 3; cursor < int(arguments.size()); cursor++) {
                std::vector<CellInfo *> dest_loc = get_cells(arguments.at(cursor));
                // misses are reported (verbose-only) inside get_cells now
                dest.insert(dest.end(), dest_loc.begin(), dest_loc.end());
            }
            for (auto c : dest) {
                for (const auto &pair : arg_pairs) {
                    IdString id_prop = ctx->id(pair.first);
                    if (ctx->debug)
                        log_info("applying property '%s' = '%s' to cell '%s' (on line %d)\n", pair.first.c_str(),
                                 pair.second.c_str(), c->name.c_str(ctx), lineno);
                    if (c->attrs.find(id_prop) != c->attrs.end()) {
                        log_nonfatal_error("found multiple properties '%s' for cell '%s' (on line %d)\n",
                                           pair.first.c_str(), c->name.c_str(ctx), lineno);
                        num_errors++;
                    }
                    c->attrs[id_prop] = std::string(pair.second);
                    if (pair.first == "PACKAGE_PIN")
                        c->attrs[id_LOC] = std::string(pair.second);
                }
            }
        } else if (cmd == "create_clock") {
            double period = 0;
            bool got_period = false;
            int cursor = 1;
            for (cursor = 1; cursor < int(arguments.size()); cursor++) {
                std::string opt = arguments.at(cursor);
                if (opt == "-add") {
                    log_warning("ignoring unsupported XDC option '%s' (on line %d)\n", opt.c_str(), lineno);
                } else if (opt == "-name" || opt == "-waveform") {
                    log_warning("ignoring unsupported XDC option '%s' (on line %d)\n", opt.c_str(), lineno);
                    cursor++;
                } else if (opt == "-period") {
                    cursor++;
                    period = std::stod(arguments.at(cursor));
                    got_period = true;
                } else
                    break;
            }
            if (!got_period) {
                log_nonfatal_error("found create_clock without period (on line %d)\n", lineno);
                num_errors++;
                goto nextline;
            }
            // All remaining arguments are supposed to designate ports/nets
            std::vector<NetInfo *> dest;
            if (cursor >= int(arguments.size())) {
                // virtual clock (no target ports/nets): not supported; skip it
                // instead of silently constraining nothing
                log_warning("ignoring virtual clock (unsupported, on line %d)\n", lineno);
                goto nextline;
            }
            for (; cursor < (int)arguments.size(); cursor++) {
                std::vector<NetInfo *> dest_loc = get_nets(arguments.at(cursor));
                if (dest_loc.empty())
                    log_warning("create_clock: target %s matched nothing, so the %.3f ns constraint "
                                "was NOT applied (on line %d). The clock domain keeps the default "
                                "target and will be reported as meeting timing at that default.\n",
                                arguments.at(cursor).c_str(), period, lineno);
                dest.insert(dest.end(), dest_loc.begin(), dest_loc.end());
            }
            for (auto n : dest) {
                if (ctx->debug)
                    log_info("applying clock period constraint on net '%s' (on line %d)\n", n->name.c_str(ctx), lineno);
                if (n->clkconstr.get() != nullptr) {
                    log_nonfatal_error("found multiple clock constraints on net '%s' (on line %d)\n",
                                       n->name.c_str(ctx), lineno);
                    num_errors++;
                }
                n->clkconstr = std::unique_ptr<ClockConstraint>(new ClockConstraint);
                n->clkconstr->period = DelayPair(ctx->getDelayFromNS(period));
                n->clkconstr->high = DelayPair(ctx->getDelayFromNS(period / 2));
                n->clkconstr->low = DelayPair(ctx->getDelayFromNS(period / 2));
            }
        } else if (cmd == "set_multicycle_path") {
            // set_multicycle_path [-setup|-hold] [-end] <N> -to <sel>: the -to cells carry N (NEXTPNR_MCP_SETUP)
            // for the timing analysis. <sel> is a list of cell names, or [get_cells ...] with names or
            // -filter {NAME =~ <glob>} or {NAME == <name>}. get_pins and get_nets are not taken: the tag relaxes
            // every input of a cell, and Vivado does not apply a net in -to.
            //  (Port of nextpnr-xilinx 813bb715.)
            auto glob_match = [](const std::string &name, const std::string &pat) {
                // simple '*' wildcard match
                size_t n = 0, p = 0, star = std::string::npos, mark = 0;
                while (n < name.size()) {
                    bool pattern_remaining = p < pat.size();
                    bool pattern_char_matches = pattern_remaining && (pat[p] == name[n] || pat[p] == '?');
                    bool pattern_at_star = pattern_remaining && pat[p] == '*';
                    bool can_backtrack_to_star = star != std::string::npos;
                    if (pattern_char_matches) {
                        ++n;
                        ++p;
                    } else if (pattern_at_star) {
                        star = p++;
                        mark = n;
                    } else if (can_backtrack_to_star) {
                        p = star + 1;
                        n = ++mark;
                    } else
                        return false;
                }
                while (p < pat.size() && pat[p] == '*')
                    ++p;
                return p == pat.size();
            };
            // The cells a name or glob selects; an exact name (NAME ==) is never a glob
            auto add_matches = [&](const std::string &pattern, const bool is_exact, std::vector<CellInfo *> &cells) {
                const bool is_glob = !is_exact && pattern.find_first_of("*?") != std::string::npos;
                if (!is_glob) {
                    IdString name = ctx->id(pattern);
                    if (!ctx->cells.count(name) && !debus_zero(pattern).empty())
                        name = ctx->id(debus_zero(pattern));
                    if (ctx->cells.count(name))
                        cells.push_back(ctx->cells.at(name).get());
                    return;
                }
                for (auto &cell : ctx->cells) {
                    // the JSON frontend's constant drivers, e.g. "g[0].q.CE$const$VCC$12", are not paths
                    const bool is_constant_driver = cell.second->type.in(id_GND, id_VCC);
                    if (!is_constant_driver && glob_match(cell.first.str(ctx), pattern))
                        cells.push_back(cell.second.get());
                }
            };
            // Tcl words: {...} and "..." group, and brackets stay in a name such as g[3].q
            auto tcl_words = [](const std::string &str) {
                std::vector<std::string> words;
                std::string word;
                int depth = 0;
                bool quoted = false;
                for (char c : str) {
                    if (c == '"' && depth == 0)
                        quoted = !quoted;
                    else if (c == '{' && !quoted)
                        depth++;
                    else if (c == '}' && !quoted)
                        depth--;
                    const bool separates_words = std::isspace(c) && depth == 0 && !quoted;
                    if (!separates_words)
                        word += c;
                    else if (!word.empty()) {
                        words.push_back(word);
                        word.clear();
                    }
                }
                if (!word.empty())
                    words.push_back(word);
                return words;
            };
            // The cells of a -to selector; false, with a warning, for a form this reader does not take
            auto select_cells = [&](const std::string &sel, std::vector<CellInfo *> &cells) {
                const bool is_command = sel.size() >= 2 && sel.front() == '[' && sel.back() == ']';
                std::vector<std::string> words =
                        tcl_words(is_command ? sel.substr(1, sel.size() - 2) : strip_quotes(sel));
                std::string kind = "get_cells";
                int cursor = 0;
                if (is_command) {
                    kind = words.empty() ? std::string() : words.front();
                    const bool is_supported_command = kind == "get_cells";
                    if (!is_supported_command) {
                        log_warning("ignoring set_multicycle_path with unsupported selector '%s' (on line %d)\n",
                                    sel.c_str(), lineno);
                        return false;
                    }
                    cursor = 1;
                }
                // -filter narrows the named objects, as in Vivado; without names it selects from all of them
                std::vector<CellInfo *> named, filtered;
                bool has_names = false, has_filter = false;
                for (; cursor < int(words.size()); cursor++) {
                    const std::string &w = words.at(cursor);
                    const bool names_are_flat = w == "-hier" || w == "-hierarchical" || w == "-quiet";
                    if (names_are_flat)
                        continue;
                    const bool is_filter = w == "-filter" && cursor + 1 < int(words.size());
                    if (is_filter) {
                        // only NAME =~ <glob> and NAME == <name>
                        std::vector<std::string> expr = tcl_words(strip_quotes(words.at(++cursor)));
                        const bool is_name_filter =
                                expr.size() == 3 && expr.at(0) == "NAME" && (expr.at(1) == "=~" || expr.at(1) == "==");
                        if (!is_name_filter) {
                            log_warning("ignoring set_multicycle_path with unsupported -filter '%s', only "
                                        "NAME =~ <glob> or NAME == <name> (on line %d)\n",
                                        words.at(cursor).c_str(), lineno);
                            return false;
                        }
                        // NAME == compares the whole name, as in Vivado: a '*' there is not a wildcard
                        const bool is_exact = expr.at(1) == "==";
                        add_matches(strip_quotes(expr.at(2)), is_exact, filtered);
                        has_filter = true;
                        continue;
                    }
                    const bool is_other_option = w.front() == '-';
                    if (is_other_option) {
                        log_warning("ignoring set_multicycle_path with unsupported %s option '%s' (on line %d)\n",
                                    kind.c_str(), w.c_str(), lineno);
                        return false;
                    }
                    for (const std::string &name : tcl_words(strip_quotes(w)))
                        add_matches(name, false, named);
                    has_names = true;
                }
                if (has_names && has_filter) {
                    pool<IdString> passes_filter;
                    for (CellInfo *cell : filtered)
                        passes_filter.insert(cell->name);
                    for (CellInfo *cell : named)
                        if (passes_filter.count(cell->name))
                            cells.push_back(cell);
                } else {
                    cells.insert(cells.end(), named.begin(), named.end());
                    cells.insert(cells.end(), filtered.begin(), filtered.end());
                }
                return true;
            };
            int cycles = 0;
            bool is_hold = false;
            std::string to_sel;
            for (int c = 1; c < int(arguments.size()); c++) {
                const std::string &a = arguments.at(c);
                const bool has_value = c + 1 < int(arguments.size());
                // an N that does not fit an int is not taken (Vivado refuses 2^32 and more)
                const bool is_cycle_count = !a.empty() && a.size() <= 10 &&
                                            std::all_of(a.begin(), a.end(), ::isdigit) &&
                                            std::strtoll(a.c_str(), nullptr, 10) <= std::numeric_limits<int>::max();
                // -end: the capture edge moves N-1 capture periods. -start counts launch periods, which the
                // timing hook does not know, so it is left to the unsupported-argument warning below.
                const bool has_no_effect_here = a == "-setup" || a == "-end";
                if (has_no_effect_here)
                    continue;
                if (a == "-hold")
                    is_hold = true;
                else if (a == "-from") {
                    log_warning("ignoring unsupported XDC option '-from' in set_multicycle_path (on line %d)\n", lineno);
                    goto nextline;
                } else if (a == "-to" && has_value)
                    to_sel = arguments.at(++c);
                else if (is_cycle_count)
                    cycles = std::stoi(a);
                else {
                    log_warning("ignoring set_multicycle_path with unsupported argument '%s' (on line %d)\n", a.c_str(),
                                lineno);
                    goto nextline;
                }
            }
            if (is_hold) {
                log_info("set_multicycle_path -hold: hold stays checked on the launch edge (on line %d)\n", lineno);
                goto nextline;
            }
            const bool is_incomplete = cycles < 1 || to_sel.empty();
            if (is_incomplete) {
                log_warning("ignoring set_multicycle_path without N of 1 or more, or without -to (on line %d)\n",
                            lineno);
                goto nextline;
            }
            std::vector<CellInfo *> to_cells;
            if (!select_cells(to_sel, to_cells))
                goto nextline;
            pool<IdString> tagged;
            for (CellInfo *cell : to_cells)
                if (tagged.insert(cell->name).second)
                    cell->attrs[id_NEXTPNR_MCP_SETUP] = std::to_string(cycles);
            if (tagged.empty())
                log_warning("set_multicycle_path: the -to list matches no cells (on line %d)\n", lineno);
            else
                log_info("set_multicycle_path: setup multicycle %d tagged on %d cells (on line %d)\n", cycles,
                         int(tagged.size()), lineno);
        } else {
            log_warning("ignoring unsupported XDC command '%s' (on line %d)\n", cmd.c_str(), lineno);
        }

    nextline:; // Phony statement to have something legal after the label
    }
    if (!isempty(linebuf)) {
        log_nonfatal_error("unexpected end of XDC file\n");
        num_errors++;
    }
    bool report_missing_targets = missing_targets > 0 && ctx->verbose;
    if (report_missing_targets)
        log_info("%d XDC constraint target(s) reference ports or nets that are not in this design and were "
                 "ignored (a board-level XDC normally constrains more pins than a design uses)\n",
                 missing_targets);
    if (num_errors > 0) {
        log_error("Stopping the program after %u errors found in XDC file\n", num_errors);
    }
}

NEXTPNR_NAMESPACE_END
