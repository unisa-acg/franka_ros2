#ifndef FRANKA_TRACING_TP_H
#define FRANKA_TRACING_TP_H

#ifdef FRANKA_WITH_LTTNG_UST

#undef TRACEPOINT_PROVIDER
#define TRACEPOINT_PROVIDER franka_timing

#undef TRACEPOINT_INCLUDE
#define TRACEPOINT_INCLUDE "./tp.h"

#if !defined(FRANKA_TRACING_TP_H_LTTNG) || defined(TRACEPOINT_HEADER_MULTI_READ)
#define FRANKA_TRACING_TP_H_LTTNG

#include <stdint.h>

#include <lttng/tracepoint.h>

TRACEPOINT_EVENT(
    franka_timing,
    read_entry,
    TP_ARGS(),
    TP_FIELDS()
)

TRACEPOINT_EVENT(
    franka_timing,
    read_exit,
    TP_ARGS(uint64_t, robot_ticks),
    TP_FIELDS(
        ctf_integer(uint64_t, robot_ticks, robot_ticks)
    )
)

TRACEPOINT_EVENT(
    franka_timing,
    write_entry,
    TP_ARGS(),
    TP_FIELDS()
)

TRACEPOINT_EVENT(
    franka_timing,
    write_exit,
    TP_ARGS(),
    TP_FIELDS()
)

#endif  // FRANKA_TRACING_TP_H_LTTNG

#include <lttng/tracepoint-event.h>

#else  // !FRANKA_WITH_LTTNG_UST

#ifndef tracepoint
#define tracepoint(...) \
  do {                  \
  } while (0)
#endif

#endif  // FRANKA_WITH_LTTNG_UST

#endif  // FRANKA_TRACING_TP_H
