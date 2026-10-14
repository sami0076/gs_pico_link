// Collecting bytes from the USB port into one complete command line.
//
// PURE C++. No Pico headers, so this is covered by firmware/host_tests.
//
// Bytes arrive from the host a few at a time, split at arbitrary points, and
// a command is only useful once the whole line has turned up. This class does
// that one job: feed it characters, and it tells you when a line is ready.
//
// It uses a fixed buffer and never allocates (spec line 169), because it runs
// inside the loop that is also pulsing the motors.
//
// A line that is too long is reported once and then thrown away, and reading
// picks up cleanly at the next newline. Without that, one bad burst of bytes
// would wedge the parser and the Pico would stop answering.
#pragma once

#include <stddef.h>

// Longest command accepted. The longest one this protocol can produce is
// about 30 characters ("MOVE,65535,AZ,90000,-360000"), so this is roomy.
constexpr size_t MAX_LINE_LEN = 95;

class LineReader {
public:
    enum class Result {
        None,     // nothing finished yet - keep feeding
        Line,     // a complete line is ready, read it with line()
        TooLong,  // the line overran the buffer and was discarded
    };

    // Hand over one received character.
    Result feed(char c);

    // The finished line, without its newline. Only meaningful right after
    // feed() returns Line, and only until the next call to feed().
    const char *line() const { return buf_; }

    // True when some of a line has arrived but its newline has not. Used to
    // keep status messages from printing over something being typed.
    bool partial() const { return len_ > 0; }

    // Throw away a partly-received line. Used at startup.
    void reset();

private:
    char buf_[MAX_LINE_LEN + 1] = {};  // +1 for the terminating NUL
    size_t len_                 = 0;
    bool overflow_              = false;
};
