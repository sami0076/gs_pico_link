#include "motor_cmd.hpp"

#include <stddef.h>

namespace {

// A command has at most 5 fields (MOVE). One spare so that a line with too
// many fields can be noticed rather than silently truncated.
constexpr size_t MAX_FIELDS = 6;

struct Field {
    const char *p;
    size_t n;
};

// Cut the line at every comma. Empty fields are kept, so ",,," is four empty
// fields and gets rejected later rather than looking like a short command.
size_t split(const char *line, Field *out)
{
    size_t count = 0;
    const char *start = line;

    for (const char *p = line;; p++) {
        if (*p == ',' || *p == '\0') {
            if (count < MAX_FIELDS) {
                out[count].p = start;
                out[count].n = static_cast<size_t>(p - start);
            }
            count++;
            if (*p == '\0') {
                break;
            }
            start = p + 1;
        }
    }
    return count;
}

char upper(char c)
{
    return (c >= 'a' && c <= 'z') ? static_cast<char>(c - 'a' + 'A') : c;
}

// Compare a field against a fixed word, ignoring case.
bool is_word(const Field &f, const char *word)
{
    size_t i = 0;
    for (; i < f.n; i++) {
        if (word[i] == '\0' || upper(f.p[i]) != word[i]) {
            return false;
        }
    }
    return word[i] == '\0';
}

// Read a whole number. Strict on purpose: an optional + or -, then at least
// one digit, then the end of the field. Anything else is a mistake worth
// reporting rather than guessing at - "45.5" from a confused sender should
// not quietly become 45.
//
// The running total is int64 so that a long run of digits cannot wrap round
// into a plausible-looking small number before the range check sees it.
bool parse_int(const Field &f, int64_t lo, int64_t hi, int64_t *out)
{
    if (f.n == 0) {
        return false;
    }

    size_t i      = 0;
    bool negative = false;
    if (f.p[0] == '+' || f.p[0] == '-') {
        negative = (f.p[0] == '-');
        i        = 1;
        if (f.n == 1) {
            return false;  // a sign on its own
        }
    }

    int64_t value = 0;
    for (; i < f.n; i++) {
        const char c = f.p[i];
        if (c < '0' || c > '9') {
            return false;
        }
        value = value * 10 + (c - '0');
        if (value > 4000000000LL) {
            return false;  // far past anything useful; stop before it wraps
        }
    }

    if (negative) {
        value = -value;
    }
    if (value < lo || value > hi) {
        return false;
    }
    *out = value;
    return true;
}

// Fill in err and leave the verb unset, so a caller that forgets to check
// err still does nothing rather than something wrong.
Command fail(ParseError e)
{
    Command c;
    c.err = e;
    return c;
}

bool read_axis(const Field &f, bool allow_all, AxisId *out)
{
    if (is_word(f, "AZ")) {
        *out = AxisId::Az;
        return true;
    }
    if (is_word(f, "EL")) {
        *out = AxisId::El;
        return true;
    }
    if (allow_all && is_word(f, "ALL")) {
        *out = AxisId::All;
        return true;
    }
    return false;
}

}  // namespace

Command parse_command(const char *line)
{
    Field f[MAX_FIELDS] = {};
    const size_t n      = split(line, f);

    if (n > MAX_FIELDS) {
        return fail(ParseError::Syntax);
    }

    // Work out which command this is first, so an unknown word is reported as
    // such rather than as a field-count problem.
    Verb verb          = Verb::None;
    size_t want_fields = 0;

    if (is_word(f[0], "MOVE")) {
        verb        = Verb::Move;
        want_fields = 5;
    } else if (is_word(f[0], "STOP")) {
        verb        = Verb::Stop;
        want_fields = 2;
    } else if (is_word(f[0], "ESTOP")) {
        verb        = Verb::Estop;
        want_fields = 2;
    } else if (is_word(f[0], "ZERO")) {
        verb        = Verb::Zero;
        want_fields = 3;  // or 4, handled below
    } else if (is_word(f[0], "POS")) {
        verb        = Verb::Pos;
        want_fields = 2;
    } else if (is_word(f[0], "PIN")) {
        verb        = Verb::Pin;
        want_fields = 4;
    } else if (is_word(f[0], "LIMITS")) {
        verb        = Verb::Limits;
        want_fields = 2;
    } else if (is_word(f[0], "PING")) {
        verb        = Verb::Ping;
        want_fields = 2;
    } else {
        return fail(ParseError::UnknownVerb);
    }

    const bool zero_with_angle = (verb == Verb::Zero && n == 4);
    if (n != want_fields && !zero_with_angle) {
        return fail(ParseError::Syntax);
    }

    // Every command carries an id.
    int64_t id = 0;
    if (!parse_int(f[1], 0, 65535, &id)) {
        return fail(ParseError::Syntax);
    }

    Command c;
    c.verb = verb;
    c.id   = static_cast<uint16_t>(id);

    switch (verb) {
    case Verb::Move: {
        if (!read_axis(f[2], false, &c.axis)) {
            return fail(ParseError::BadAxis);
        }
        // The range here is only a guard against nonsense; whether the speed
        // and angle are actually acceptable is plan_move's decision.
        int64_t mdps = 0;
        int64_t mdeg = 0;
        if (!parse_int(f[3], 0, 1000000, &mdps) ||
            !parse_int(f[4], -1000000, 1000000, &mdeg)) {
            return fail(ParseError::Syntax);
        }
        c.mdps = static_cast<int32_t>(mdps);
        c.mdeg = static_cast<int32_t>(mdeg);
        break;
    }

    case Verb::Zero: {
        if (!read_axis(f[2], true, &c.axis)) {
            return fail(ParseError::BadAxis);
        }
        if (zero_with_angle) {
            int64_t mdeg = 0;
            if (!parse_int(f[3], -1000000, 1000000, &mdeg)) {
                return fail(ParseError::Syntax);
            }
            c.mdeg     = static_cast<int32_t>(mdeg);
            c.has_mdeg = true;
        }
        break;
    }

    case Verb::Pin: {
        // 0-29 is every GPIO the chip has. Whether a particular pin is safe
        // to poke is a hardware question, answered where the pin map lives.
        int64_t gpio  = 0;
        int64_t level = 0;
        if (!parse_int(f[2], 0, 29, &gpio) || !parse_int(f[3], 0, 1, &level)) {
            return fail(ParseError::BadGpio);
        }
        c.gpio  = static_cast<uint8_t>(gpio);
        c.level = static_cast<uint8_t>(level);
        break;
    }

    default:
        break;  // STOP, ESTOP, POS, LIMITS, PING need nothing more
    }

    return c;
}
