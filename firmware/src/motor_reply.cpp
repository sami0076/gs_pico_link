#include "motor_reply.hpp"

#include <stdio.h>

namespace {

// snprintf tells us how long the line would have been. If that does not fit,
// report nothing written rather than handing back a truncated line.
size_t finish(char *buf, size_t n, int written)
{
    if (written < 0 || static_cast<size_t>(written) >= n) {
        if (n > 0) {
            buf[0] = '\0';
        }
        return 0;
    }
    return static_cast<size_t>(written);
}

// Longest token kept from text the laptop sent. Enough to recognise what was
// wrong without letting a long line push the real fields off the end.
constexpr size_t MAX_TOKEN = 16;

}  // namespace

const char *verb_name(Verb v)
{
    switch (v) {
    case Verb::Move:   return "MOVE";
    case Verb::Stop:   return "STOP";
    case Verb::Estop:  return "ESTOP";
    case Verb::Zero:   return "ZERO";
    case Verb::Pos:    return "POS";
    case Verb::Pin:    return "PIN";
    case Verb::Limits: return "LIMITS";
    case Verb::Ping:   return "PING";
    case Verb::None:   break;
    }
    return "NONE";
}

const char *axis_name(AxisId a)
{
    switch (a) {
    case AxisId::Az:  return "AZ";
    case AxisId::El:  return "EL";
    case AxisId::All: return "ALL";
    case AxisId::None: break;
    }
    return "NONE";
}

const char *err_name(ErrCode e)
{
    switch (e) {
    case ErrCode::Syntax:    return "syntax";
    case ErrCode::BadVerb:   return "verb";
    case ErrCode::BadAxis:   return "axis";
    case ErrCode::BadGpio:   return "gpio";
    case ErrCode::BadSpeed:  return "speed";
    case ErrCode::Limit:     return "limit";
    case ErrCode::NoZero:    return "nozero";
    case ErrCode::Busy:      return "busy";
    case ErrCode::ZeroSteps: return "zerosteps";
    case ErrCode::TooLong:   return "toolong";
    }
    return "unknown";
}

size_t sanitise_token(char *dst, size_t dstn, const char *src)
{
    if (dstn == 0) {
        return 0;
    }
    if (src == nullptr) {
        dst[0] = '\0';
        return 0;
    }

    size_t out   = 0;
    const size_t room = (dstn - 1 < MAX_TOKEN) ? (dstn - 1) : MAX_TOKEN;

    for (size_t i = 0; src[i] != '\0' && out < room; i++) {
        const unsigned char c = static_cast<unsigned char>(src[i]);

        // A comma would split the line into an extra field, and a newline
        // would end the record early - the laptop would then read the rest as
        // a different reply. Anything outside plain printable ASCII goes the
        // same way.
        if (c < 0x20 || c > 0x7E || c == ',') {
            dst[out++] = '?';
        } else {
            dst[out++] = static_cast<char>(c);
        }
    }

    dst[out] = '\0';
    return out;
}

size_t format_ack(char *buf, size_t n, uint16_t id, Verb verb)
{
    return finish(buf, n,
                  snprintf(buf, n, "ACK,%u,%s\n", static_cast<unsigned>(id),
                           verb_name(verb)));
}

size_t format_plan(char *buf, size_t n, uint16_t id, AxisId axis,
                   const Move &m)
{
    // The shape is reported so an odd row in the test data explains itself:
    // TRI means the move was too short to reach the commanded speed.
    return finish(
        buf, n,
        snprintf(buf, n, "PLAN,%u,%s,%ld,%ld,%ld,%ld,%s,%lu\n",
                 static_cast<unsigned>(id), axis_name(axis),
                 static_cast<long>(m.steps), static_cast<long>(m.from_mdeg),
                 static_cast<long>(m.to_mdeg),
                 static_cast<long>(m.speed_mdps), m.triangular ? "TRI" : "TRAP",
                 static_cast<unsigned long>(m.est_us)));
}

size_t format_done(char *buf, size_t n, uint16_t id, AxisId axis, bool ok,
                   int32_t steps_done, int32_t from_mdeg, int32_t to_mdeg,
                   uint32_t elapsed_us)
{
    return finish(
        buf, n,
        snprintf(buf, n, "DONE,%u,%s,%s,%ld,%ld,%ld,%lu\n",
                 static_cast<unsigned>(id), axis_name(axis),
                 ok ? "OK" : "ABORT", static_cast<long>(steps_done),
                 static_cast<long>(from_mdeg), static_cast<long>(to_mdeg),
                 static_cast<unsigned long>(elapsed_us)));
}

size_t format_pos(char *buf, size_t n, uint16_t id, int32_t az_mdeg,
                  int32_t el_mdeg, int32_t az_steps, int32_t el_steps,
                  bool zeroed)
{
    // Steps are sent as well as degrees because steps are what the firmware
    // actually counts; the degrees are derived from them.
    return finish(buf, n,
                  snprintf(buf, n, "POS,%u,%ld,%ld,%ld,%ld,%s\n",
                           static_cast<unsigned>(id),
                           static_cast<long>(az_mdeg),
                           static_cast<long>(el_mdeg),
                           static_cast<long>(az_steps),
                           static_cast<long>(el_steps),
                           zeroed ? "ZEROED" : "UNZEROED"));
}

size_t format_pin(char *buf, size_t n, uint16_t id, uint8_t gpio,
                  uint8_t level)
{
    return finish(buf, n,
                  snprintf(buf, n, "PIN,%u,%u,%u\n", static_cast<unsigned>(id),
                           static_cast<unsigned>(gpio),
                           static_cast<unsigned>(level)));
}

size_t format_lim(char *buf, size_t n, uint16_t id)
{
    // Read straight from the config, so the laptop never has to guess what
    // this particular build was flashed with.
    return finish(buf, n,
                  snprintf(buf, n, "LIM,%u,%ld,%ld,%ld,%ld,%ld,%ld\n",
                           static_cast<unsigned>(id),
                           static_cast<long>(AZ_SPEC.min_mdeg),
                           static_cast<long>(AZ_SPEC.max_mdeg),
                           static_cast<long>(EL_SPEC.min_mdeg),
                           static_cast<long>(EL_SPEC.max_mdeg),
                           static_cast<long>(SPEED_MIN_MDPS),
                           static_cast<long>(SPEED_MAX_MDPS)));
}

size_t format_pong(char *buf, size_t n, uint16_t id, const char *target,
                   const char *version, uint32_t uptime_ms)
{
    // Uptime is here so the laptop can spot a Pico that rebooted: if it goes
    // backwards the position was lost, and moving again would be unsafe.
    char t[MAX_TOKEN + 1];
    char v[MAX_TOKEN + 1];
    sanitise_token(t, sizeof(t), target);
    sanitise_token(v, sizeof(v), version);

    return finish(buf, n,
                  snprintf(buf, n, "PONG,%u,%s,%s,%lu\n",
                           static_cast<unsigned>(id), t, v,
                           static_cast<unsigned long>(uptime_ms)));
}

size_t format_mstat(char *buf, size_t n, uint32_t uptime_s, int32_t az_mdeg,
                    int32_t el_mdeg, bool moving)
{
    // No id: this one is not a reply to anything, it just arrives.
    return finish(buf, n,
                  snprintf(buf, n, "MSTAT,%lu,%ld,%ld,%s\n",
                           static_cast<unsigned long>(uptime_s),
                           static_cast<long>(az_mdeg),
                           static_cast<long>(el_mdeg),
                           moving ? "MOVING" : "IDLE"));
}

size_t format_err(char *buf, size_t n, uint16_t id, ErrCode code,
                  const char *detail)
{
    char d[MAX_TOKEN + 1];
    sanitise_token(d, sizeof(d), detail);

    return finish(buf, n,
                  snprintf(buf, n, "ERR,%u,%s,%s\n", static_cast<unsigned>(id),
                           err_name(code), d));
}

size_t format_err_limit(char *buf, size_t n, uint16_t id, AxisId axis,
                        int32_t target_mdeg, int32_t min_mdeg,
                        int32_t max_mdeg)
{
    return finish(buf, n,
                  snprintf(buf, n, "ERR,%u,limit,%s,%ld,%ld,%ld\n",
                           static_cast<unsigned>(id), axis_name(axis),
                           static_cast<long>(target_mdeg),
                           static_cast<long>(min_mdeg),
                           static_cast<long>(max_mdeg)));
}
