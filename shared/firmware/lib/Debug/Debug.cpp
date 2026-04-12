#include "Debug.h"
#include "PID.h"

uint32_t Debug::s_mask = 0;

void     Debug::begin(uint32_t initial_mask) { s_mask = initial_mask; }
void     Debug::setMask(uint32_t mask)       { s_mask = mask; }
uint32_t Debug::getMask()                    { return s_mask; }
void     Debug::enable(uint32_t topics)      { s_mask |= topics; }
void     Debug::disable(uint32_t topics)     { s_mask &= ~topics; }
bool     Debug::enabled(uint32_t topic)      { return (s_mask & topic) != 0; }

void Debug::pidState(uint32_t topic, const char* label, const PID& pid) {
#ifndef ENABLE_COMMS
    if (!enabled(topic)) return;
    Serial.printf("  [%s] err=%7.2f  P=%7.3f I=%7.3f D=%7.3f  out=%6.3f\n",
                  label,
                  pid.error(), pid.pTerm(), pid.iTerm(), pid.dTerm(),
                  pid.output());
#else
    (void)topic; (void)label; (void)pid;
#endif
}
