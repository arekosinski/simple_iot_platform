
#ifndef GLOBAL_COUNTER_H
#define GLOBAL_COUNTER_H

#include <Arduino.h>

class GlobalCounter {
    public:
        GlobalCounter();
        void increase_msg_retransmissions(const int_fast32_t value = 1);
        uint_fast32_t get_msg_retransmissions();
        void reset_msg_retransmissions();

        void increase_cycles(const uint_fast32_t value = 1);
        uint_fast32_t get_cycles();
        void reset_cycles();

        void increase_cycles_length(const int_fast32_t value = 1);
        uint_fast64_t get_cycles_length();
        void reset_cycles_length();

        void increase_msg_success(const int_fast32_t value = 1);
        uint_fast32_t get_msg_success();
        void reset_msg_success();

        void increase_msg_trials(const int_fast32_t value = 1);
        uint_fast32_t get_msg_trials();
        void reset_msg_trials();


    private:
        uint_fast32_t global_cycles_count;
        uint_fast64_t global_cycles_length;
        uint_fast32_t global_msg_retransmission_count;
        uint_fast32_t global_msg_success_count;
        uint_fast32_t global_msg_trials_count;
};

#endif