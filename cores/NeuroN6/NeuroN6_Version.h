/**
 ******************************************************************************
 * @file    NeuroN6_Version.h
 * @brief   The version of this core, for sketches that need a minimum one.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#ifndef NEURON6_VERSION_H
#define NEURON6_VERSION_H

/* Matches version= in platform.txt; tools/check_arduino_spec.py checks it. */
#define NEURON6_CORE_VERSION_MAJOR  0
#define NEURON6_CORE_VERSION_MINOR  1
#define NEURON6_CORE_VERSION_PATCH  3
#define NEURON6_CORE_VERSION_STRING "0.1.3"

#define NEURON6_CORE_VERSION \
  ((NEURON6_CORE_VERSION_MAJOR << 16) | (NEURON6_CORE_VERSION_MINOR << 8) | NEURON6_CORE_VERSION_PATCH)

/* True when this core is a.b.c or later. Test it under #ifndef
 * NEURON6_CORE_VERSION_AT_LEAST, which catches a build for another board:
 * TUTORIAL.md, "Sketch structure". */
#define NEURON6_CORE_VERSION_AT_LEAST(a, b, c) \
  (NEURON6_CORE_VERSION >= (((a) << 16) | ((b) << 8) | (c)))

#endif /* NEURON6_VERSION_H */
