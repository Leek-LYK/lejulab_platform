/*
 * The Lean Mean C++ Option Parser
 *
 * Copyright (C) 2012-2017 Matthias S. Benkmann
 *
 * The "Software" in the following 2 paragraphs refers to this file containing
 * the code to The Lean Mean C++ Option Parser.
 * The "Software" does NOT refer to any other files which you
 * may have received alongside this file (e.g. as part of a larger project that
 * incorporates The Lean Mean C++ Option Parser).
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software, to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to permit
 * persons to whom the Software is furnished to do so, subject to the following
 * conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef OPTIONPARSER_H_
#define OPTIONPARSER_H_

#include <cstring>
#include <cstdio>

namespace option {

struct Option;

struct Arg {
    enum ArgType { NONE_VAL = 0, OPTIONAL_VAL = 1 };

    static int None(const struct Option&, bool) { return NONE_VAL; }
    static int Optional(const struct Option& opt, bool msg);
};

struct Descriptor {
    unsigned index;
    int type;
    const char* shortopt;
    const char* longopt;
    int (*check_arg)(const struct Option&, bool);
    const char* help;
};

struct Option {
    const Descriptor* desc;
    const char* name;
    const char* arg;
    Option* next_;
    Option* prev_;

    Option() : desc(nullptr), name(nullptr), arg(nullptr), next_(nullptr), prev_(nullptr) {}

    Option(const Descriptor* d, const char* n, const char* a)
        : desc(d), name(n), arg(a), next_(nullptr), prev_(nullptr) {}

    operator bool() const { return desc != nullptr; }

    Option* next() { return next_; }
    Option* prev() { return prev_; }

    int type() const { return desc ? desc->type : 0; }
    int index() const { return desc ? static_cast<int>(desc->index) : -1; }
    int count() const {
        int c = 0;
        const Option* p = this;
        while (p) { ++c; p = p->prev_; }
        return c - 1;
    }
};

// Define Arg::Optional after Option is complete
inline int Arg::Optional(const struct Option& opt, bool) {
    if (opt.arg) return NONE_VAL;
    return OPTIONAL_VAL;
}

struct Stats {
    unsigned options_max;
    unsigned buffer_max;

    Stats() : options_max(0), buffer_max(0) {}

    Stats(const Descriptor usage[], int argc, char** argv)
        : options_max(0), buffer_max(0)
    {
        add(usage, argc, argv);
    }

    void add(const Descriptor usage[], int argc, char**) {
        unsigned count = 0;
        for (const Descriptor* d = usage; d->shortopt || d->longopt; ++d) {
            ++count;
        }
        options_max = count + 1;
        buffer_max = static_cast<unsigned>(argc) + count + 1;
    }
};

class Parser {
public:
    Parser() : err_(false) {}

    Parser(const Descriptor usage[], int argc, char** argv,
           Option* options, Option* buffer)
        : err_(false)
    {
        parse(usage, argc, argv, options, buffer);
    }

    bool error() const { return err_; }

    void parse(const Descriptor usage[], int argc, char** argv,
               Option* options, Option* buffer)
    {
        // Initialize options array
        unsigned count = 0;
        for (const Descriptor* d = usage; d->shortopt || d->longopt; ++d) {
            ++count;
        }
        for (unsigned i = 0; i <= count; ++i) {
            options[i] = Option();
        }

        int bufIdx = 0;

        for (int i = 0; i < argc; ++i) {
            const char* arg = argv[i];
            if (!arg) continue;

            if (arg[0] == '-') {
                bool isLong = (arg[1] == '-');
                const char* optStart = arg + (isLong ? 2 : 1);

                // Find the option
                for (const Descriptor* d = usage; d->shortopt || d->longopt; ++d) {
                    bool match = false;
                    const char* optArg = nullptr;

                    if (isLong && d->longopt) {
                        size_t len = std::strlen(d->longopt);
                        if (std::strncmp(optStart, d->longopt, len) == 0) {
                            if (optStart[len] == '=') {
                                optArg = optStart + len + 1;
                                match = true;
                            } else if (optStart[len] == '\0') {
                                match = true;
                            }
                        }
                    } else if (!isLong && d->shortopt) {
                        if (optStart[0] == d->shortopt[0]) {
                            if (optStart[1] != '\0') {
                                optArg = optStart + 1;
                            }
                            match = true;
                        }
                    }

                    if (match) {
                        // Check if next arg is the value
                        if (!optArg && i + 1 < argc && argv[i + 1] && argv[i + 1][0] != '-') {
                            if (d->check_arg) {
                                Option testOpt(d, arg, nullptr);
                                if (d->check_arg(testOpt, false) != Arg::NONE_VAL) {
                                    optArg = argv[++i];
                                }
                            }
                        }

                        buffer[bufIdx] = Option(d, arg, optArg);

                        // Link to options array
                        unsigned idx = d->index;
                        if (!options[idx].desc) {
                            options[idx] = buffer[bufIdx];
                        } else {
                            Option* last = &options[idx];
                            while (last->next_) last = last->next_;
                            last->next_ = &buffer[bufIdx];
                            buffer[bufIdx].prev_ = last;
                        }
                        ++bufIdx;
                        break;
                    }
                }
            }
        }
    }

private:
    bool err_;
};

inline void printUsage(std::FILE* out, const Descriptor usage[]) {
    for (const Descriptor* d = usage; d->help; ++d) {
        std::fprintf(out, "%s\n", d->help);
    }
}

inline void printUsage(std::ostream& out, const Descriptor usage[]) {
    for (const Descriptor* d = usage; d->help; ++d) {
        out << d->help << "\n";
    }
}

} // namespace option

#include <ostream>

#endif // OPTIONPARSER_H_
