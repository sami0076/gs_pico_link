// Tests for the line reader.
//
// The interesting cases are all about bytes arriving in awkward ways: split
// across reads, with the wrong newline, or far too many of them.
#include <string.h>

#include "check.hpp"
#include "line_reader.hpp"

// Feed a whole string and return what the last character produced.
// Checks along the way that nothing finished early.
static LineReader::Result feed_all(LineReader &r, const char *s)
{
    LineReader::Result last = LineReader::Result::None;
    for (const char *p = s; *p != '\0'; p++) {
        last = r.feed(*p);
    }
    return last;
}

static void test_a_plain_line()
{
    LineReader r;
    CHECK(feed_all(r, "POS\n") == LineReader::Result::Line);
    CHECK(strcmp(r.line(), "POS") == 0);
}

// Windows sends \r\n. The \r ends the line and the \n must not then produce
// a second, empty one.
static void test_carriage_return_newline()
{
    LineReader r;
    CHECK(r.feed('P') == LineReader::Result::None);
    CHECK(r.feed('O') == LineReader::Result::None);
    CHECK(r.feed('S') == LineReader::Result::None);
    CHECK(r.feed('\r') == LineReader::Result::Line);
    CHECK(strcmp(r.line(), "POS") == 0);
    CHECK(r.feed('\n') == LineReader::Result::None);
}

static void test_bare_carriage_return()
{
    LineReader r;
    CHECK(feed_all(r, "PING\r") == LineReader::Result::Line);
    CHECK(strcmp(r.line(), "PING") == 0);
}

// Bytes turn up from USB in whatever sized lumps the host happened to send.
static void test_a_line_split_across_reads()
{
    LineReader r;
    CHECK(feed_all(r, "MOV") == LineReader::Result::None);
    CHECK(feed_all(r, "E,1,AZ,") == LineReader::Result::None);
    CHECK(feed_all(r, "5000,45000") == LineReader::Result::None);
    CHECK(r.feed('\n') == LineReader::Result::Line);
    CHECK(strcmp(r.line(), "MOVE,1,AZ,5000,45000") == 0);
}

// Pressing Enter on an empty prompt must not look like a command.
static void test_blank_lines_are_ignored()
{
    LineReader r;
    CHECK(r.feed('\n') == LineReader::Result::None);
    CHECK(r.feed('\n') == LineReader::Result::None);
    CHECK(feed_all(r, "\r\n\r\n") == LineReader::Result::None);

    // A real line still works afterwards.
    CHECK(feed_all(r, "POS\n") == LineReader::Result::Line);
    CHECK(strcmp(r.line(), "POS") == 0);
}

static void test_several_lines_in_a_row()
{
    LineReader r;
    CHECK(feed_all(r, "ONE\n") == LineReader::Result::Line);
    CHECK(strcmp(r.line(), "ONE") == 0);
    CHECK(feed_all(r, "TWO\n") == LineReader::Result::Line);
    CHECK(strcmp(r.line(), "TWO") == 0);
    // With \r\n the line completes on the \r, so feed it separately - the
    // trailing \n lands on an empty buffer and correctly reports nothing.
    CHECK(feed_all(r, "THREE\r") == LineReader::Result::Line);
    CHECK(strcmp(r.line(), "THREE") == 0);
    CHECK(r.feed('\n') == LineReader::Result::None);
}

// Exactly full is fine; one more is not.
static void test_the_length_limit()
{
    LineReader r;
    char full[MAX_LINE_LEN + 1];
    memset(full, 'x', MAX_LINE_LEN);
    full[MAX_LINE_LEN] = '\0';

    CHECK(feed_all(r, full) == LineReader::Result::None);
    CHECK(r.feed('\n') == LineReader::Result::Line);
    CHECK(strlen(r.line()) == MAX_LINE_LEN);

    // One character over.
    LineReader r2;
    CHECK(feed_all(r2, full) == LineReader::Result::None);
    CHECK(r2.feed('x') == LineReader::Result::None);
    CHECK(r2.feed('\n') == LineReader::Result::TooLong);
}

// The important part: after a too-long line, the next one must still work.
// Otherwise one bad burst stops the Pico answering for good.
static void test_it_recovers_after_a_long_line()
{
    LineReader r;
    for (int i = 0; i < 500; i++) {
        r.feed('y');
    }
    CHECK(r.feed('\n') == LineReader::Result::TooLong);

    CHECK(feed_all(r, "POS\n") == LineReader::Result::Line);
    CHECK(strcmp(r.line(), "POS") == 0);
}

// Over-length is reported once per line, not once per surplus character.
static void test_too_long_is_reported_once()
{
    LineReader r;
    int reports = 0;
    for (int i = 0; i < 300; i++) {
        if (r.feed('z') == LineReader::Result::TooLong) {
            reports++;
        }
    }
    CHECK_EQ(reports, 0);
    CHECK(r.feed('\n') == LineReader::Result::TooLong);
}

// A long run of rubbish, including control bytes and 0xFF, must leave the
// reader usable. Bytes like these turn up when a serial port is opened at the
// wrong moment or something else was talking on the line.
static void test_a_burst_of_garbage()
{
    LineReader r;
    for (int i = 0; i < 10000; i++) {
        const char c = static_cast<char>(i % 256);
        if (c == '\n' || c == '\r') {
            continue;  // keep it one enormous line
        }
        r.feed(c);
    }
    CHECK(r.feed('\n') == LineReader::Result::TooLong);
    CHECK(feed_all(r, "PING\n") == LineReader::Result::Line);
    CHECK(strcmp(r.line(), "PING") == 0);
}

// reset() throws away a half-typed line, which is what startup wants.
static void test_reset()
{
    LineReader r;
    feed_all(r, "HALF");
    r.reset();
    CHECK(feed_all(r, "POS\n") == LineReader::Result::Line);
    CHECK(strcmp(r.line(), "POS") == 0);
}

int main()
{
    test_a_plain_line();
    test_carriage_return_newline();
    test_bare_carriage_return();
    test_a_line_split_across_reads();
    test_blank_lines_are_ignored();
    test_several_lines_in_a_row();
    test_the_length_limit();
    test_it_recovers_after_a_long_line();
    test_too_long_is_reported_once();
    test_a_burst_of_garbage();
    test_reset();
    return test_summary();
}
