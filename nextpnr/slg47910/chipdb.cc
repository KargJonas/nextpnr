/*
 *  nextpnr Viaduct uarch for the Renesas SLG47910 (ForgeFPGA).
 *  ChipDB JSON loader — parses the chipdb.json produced by tools/gen_chipdb.py.
 */

#include "slg47910.h"

#include <cstdlib>
#include <fstream>
#include <sstream>

NEXTPNR_NAMESPACE_BEGIN

namespace {

// Minimal JSON pull-parser tuned to the flat chipdb schema.
struct J
{
    const char *p;
    void skip_ws() { while (*p == ' ' || *p == '\n' || *p == '\r' || *p == '\t') ++p; }
    void expect(char c) { skip_ws(); if (*p != c) log_error("chipdb: expected '%c'\n", c); ++p; }
    bool accept(char c) { skip_ws(); if (*p == c) { ++p; return true; } return false; }
    std::string str() {
        skip_ws(); if (*p != '"') log_error("chipdb: expected string\n"); ++p;
        std::string s; while (*p && *p != '"') { s += *p; ++p; } if (*p == '"') ++p; return s;
    }
    int64_t num() {
        skip_ws(); char *end; int64_t v = std::strtoll(p, &end, 10); p = end; return v;
    }
    void skip_value() {
        skip_ws();
        if (*p == '{') { int d=0; do { if(*p=='{')++d; else if(*p=='}')--d; ++p; } while(d>0&&*p); }
        else if (*p == '[') { int d=0; do { if(*p=='[')++d; else if(*p==']')--d; ++p; } while(d>0&&*p); }
        else if (*p == '"') { ++p; while(*p&&*p!='"')++p; if(*p=='"')++p; }
        else while(*p&&*p!=','&&*p!='}'&&*p!=']')++p;
    }
};

} // namespace

void ChipDB::load(const std::string &path)
{
    std::ifstream f(path);
    if (!f) log_error("cannot open chipdb '%s'\n", path.c_str());
    std::stringstream ss; ss << f.rdbuf();
    std::string data = ss.str();
    J j{data.c_str()};

    j.expect('{');
    while (true) {
        j.skip_ws(); if (j.accept('}')) break;
        std::string key = j.str(); j.expect(':');
        if (key == "version") {
            int64_t v = j.num();
            if (v != 1) log_error("chipdb: unsupported version %lld\n", (long long)v);
        } else if (key == "grid") {
            j.expect('{');
            while (!j.accept('}')) {
                std::string gk = j.str(); j.expect(':');
                if (gk == "width")       grid_w = (int)j.num();
                else if (gk == "height") grid_h = (int)j.num();
                else j.skip_value();
                j.accept(',');
            }
        } else if (key == "sites") {
            j.expect('[');
            while (!j.accept(']')) {
                Site s; s.z = 0; s.sx = 0; s.sy = 0;
                j.expect('{');
                while (!j.accept('}')) {
                    std::string sk = j.str(); j.expect(':');
                    if (sk == "id")        s.id = (int)j.num();
                    else if (sk == "type") s.type = j.str();
                    else if (sk == "x")    s.x = (int)j.num();
                    else if (sk == "y")    s.y = (int)j.num();
                    else if (sk == "z")    s.z = (int)j.num();
                    else if (sk == "sx")   s.sx = (int)j.num();
                    else if (sk == "sy")   s.sy = (int)j.num();
                    else if (sk == "pins") {
                        j.expect('[');
                        while (!j.accept(']')) {
                            Pin pn; pn.dir = 'I';
                            j.expect('{');
                            while (!j.accept('}')) {
                                std::string pk = j.str(); j.expect(':');
                                if (pk == "name")      pn.name = j.str();
                                else if (pk == "node") pn.node = (int)j.num();
                                else if (pk == "dir")  { std::string d = j.str(); pn.dir = d[0]; }
                                else j.skip_value();
                                j.accept(',');
                            }
                            s.pins.push_back(pn);
                            j.accept(',');
                        }
                    } else j.skip_value();
                    j.accept(',');
                }
                sites.push_back(std::move(s));
                j.accept(',');
            }
        } else if (key == "wires") {
            j.expect('[');
            while (!j.accept(']')) {
                Wire w; j.expect('{');
                while (!j.accept('}')) {
                    std::string wk = j.str(); j.expect(':');
                    if (wk == "id")        w.id = (int)j.num();
                    else if (wk == "name") w.name = j.str();
                    else if (wk == "x")    w.x = (int)j.num();
                    else if (wk == "y")    w.y = (int)j.num();
                    else j.skip_value();
                    j.accept(',');
                }
                wires.push_back(std::move(w));
                j.accept(',');
            }
        } else if (key == "pips") {
            j.expect('[');
            while (!j.accept(']')) {
                Pip p; j.expect('{');
                while (!j.accept('}')) {
                    std::string pk = j.str(); j.expect(':');
                    if (pk == "src")        p.src = (int)j.num();
                    else if (pk == "dst")   p.dst = (int)j.num();
                    else if (pk == "cost")  p.cost = (int)j.num();
                    else if (pk == "type")  p.type = j.str();
                    else j.skip_value();
                    j.accept(',');
                }
                pips.push_back(p);
                j.accept(',');
            }
        } else {
            j.skip_value();
        }
        j.accept(',');
    }
}

NEXTPNR_NAMESPACE_END
