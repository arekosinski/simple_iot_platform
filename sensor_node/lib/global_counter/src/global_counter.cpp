#include <global_counter.h>

GlobalCounter::GlobalCounter(void) {
    reset_cycles();
    reset_msg_retransmissions();
    reset_cycles_length();
    reset_msg_success();
    reset_msg_trials();
}

void GlobalCounter::increase_cycles(const uint_fast32_t value) {
    global_cycles_count += value;
}

uint_fast32_t GlobalCounter::get_cycles() {
    return global_cycles_count;
}

void GlobalCounter::reset_cycles() {
    global_cycles_count = 0;
}

void GlobalCounter::increase_cycles_length(const int_fast32_t value) {
    global_cycles_length += value;
}

uint_fast64_t GlobalCounter::get_cycles_length() {
    return global_cycles_length;
}

void GlobalCounter::reset_cycles_length() {
    global_cycles_length = 0;
}

void GlobalCounter::increase_msg_retransmissions(const int_fast32_t value) {
    global_msg_retransmission_count += value;
}

uint_fast32_t GlobalCounter::get_msg_retransmissions() {
    return global_msg_retransmission_count;
}

void GlobalCounter::reset_msg_retransmissions() {
    global_msg_retransmission_count = 0;
}

void GlobalCounter::increase_msg_success(const int_fast32_t value) {
    global_msg_success_count += value;
}

uint_fast32_t GlobalCounter::get_msg_success() {
    return global_msg_success_count;
}

void GlobalCounter::reset_msg_success() {
    global_msg_success_count = 0;
}


void GlobalCounter::increase_msg_trials(const int_fast32_t value) {
    global_msg_trials_count += value;
}

uint_fast32_t GlobalCounter::get_msg_trials() {
    return global_msg_trials_count;
}

void GlobalCounter::reset_msg_trials() {
    global_msg_trials_count = 0;
}