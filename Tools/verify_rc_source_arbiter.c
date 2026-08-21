#include "algorithms/rc_source_arbiter.h"

#include <assert.h>
#include <stdio.h>

int main(void)
{
    rc_source_arbiter_t arbiter;
    rc_virtual_candidate_t candidate = {
        .valid = true,
        .received_ms = 10U,
        .source_sequence = 1U,
        .heartbeat = 1U,
        .channel = {0, 0, 0, 0, 0},
    };
    uint16_t physical[RC_INPUT_CHANNEL_COUNT] = {
        1500U, 1500U, 1500U, 1300U, 1800U, 1800U, 1000U, 1000U,
    };
    uint16_t output[RC_INPUT_CHANNEL_COUNT];

    rc_source_arbiter_init(&arbiter, 0U);
    rc_source_arbiter_update(&arbiter, physical, true, true, 0U, true,
                             &candidate, 10U, output);
    assert(arbiter.authorization_seen_low);

    physical[RC_SOURCE_AUTH_CHANNEL] = 1800U;
    candidate.received_ms = 20U;
    rc_source_arbiter_update(&arbiter, physical, true, true, 0U, true,
                             &candidate, 20U, output);
    assert(arbiter.active_source == RC_SOURCE_VIRTUAL);
    assert(output[1] == 1500U);
    assert(output[3] == 1300U);

    candidate.channel[1] = 300;
    physical[3] = 1450U;
    candidate.received_ms = 70U;
    rc_source_arbiter_update(&arbiter, physical, true, true, 0U, true,
                             &candidate, 70U, output);
    assert(arbiter.active_source == RC_SOURCE_VIRTUAL);
    assert(output[1] == 1530U);
    assert(output[3] == 1450U);

    physical[0] = 1700U;
    rc_source_arbiter_update(&arbiter, physical, true, true, 0U, true,
                             &candidate, 80U, output);
    assert(arbiter.active_source == RC_SOURCE_PHYSICAL);
    assert(arbiter.last_exit_reason == RC_SOURCE_EXIT_PHYSICAL_TAKEOVER);
    assert(output[0] == 1700U && output[3] == 1450U);

    puts("rc source arbiter self-test ok");
    return 0;
}
