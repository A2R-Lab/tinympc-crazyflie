#pragma once
typedef int logVarId_t;
static inline logVarId_t logGetVarId(const char* g, const char* n) { (void)g; (void)n; return 0; }
static inline float logGetFloat(logVarId_t id) { (void)id; return 0.0f; }
#define LOG_GROUP_START(g)
#define LOG_ADD(t, n, p)
#define LOG_GROUP_STOP(g)
