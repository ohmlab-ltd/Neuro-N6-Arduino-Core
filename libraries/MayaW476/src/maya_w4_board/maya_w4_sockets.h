/**
 ******************************************************************************
 * @file    maya_w4_sockets.h
 * @brief   lwIP's socket API without its BSD name macros.
 *
 * Copyright (c) 2026 Ohm Lab Ltd.
 * SPDX-License-Identifier: MIT
 ******************************************************************************
 */
#ifndef MAYA_W4_SOCKETS_H
#define MAYA_W4_SOCKETS_H

#include "lwip/sockets.h"
#include "lwip/inet.h"

#undef accept
#undef bind
#undef shutdown
#undef closesocket
#undef connect
#undef getsockname
#undef getpeername
#undef setsockopt
#undef getsockopt
#undef listen
#undef recv
#undef recvmsg
#undef recvfrom
#undef send
#undef sendmsg
#undef sendto
#undef socket
#undef select
#undef poll
#undef ioctlsocket
#undef inet_ntop
#undef inet_pton
#undef read
#undef readv
#undef write
#undef writev
#undef close
#undef fcntl
#undef ioctl

#endif /* MAYA_W4_SOCKETS_H */
