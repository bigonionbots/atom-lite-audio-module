#pragma once

/**
 * @file Link.h
 * @brief Module end of the host link (protocol/AudioLinkProtocol.h), over USB and the Grove UART.
 *
 * Two transports listen at once:
 * - USB: UART0, wired to the ATOM Lite's USB-to-UART bridge. A host with USB host support (e.g. an
 *   ESP32-S3) plugs into the ATOM's USB-C port; that also powers the module.
 * - UART: Serial1 on the Grove port (default RX G32, TX G26, see Config.h), for any board with two
 *   free GPIOs. The Grove port's 5V and GND power the module.
 *
 * The protocol state is shared: whichever port a host is heard on first becomes the active one,
 * and all replies go there. Another port takes over with a PING, or with any frame after the
 * active one has been silent for LINK_TAKEOVER_MS. Until a host is heard, HELLO, STATUS and LOG go
 * out on both.
 *
 * Reliable host frames are accepted strictly in sequence. PING is the exception: it is accepted
 * with any sequence number and resynchronises the expected one, so a host that reconnects (or a
 * module that rebooted and sent HELLO) never deadlocks on stale sequence numbers.
 */

#include <AudioLinkProtocol.h>

#include <cstdint>

namespace hostlink {

bool begin();

/** Sends a LOG frame to the host. Any task. */
void log(const char* format, ...) __attribute__((format(printf, 1, 2)));

/** Sends one BT_DEVICE frame. Any task (called from the Bluetooth task while scanning). */
void sendBtDevice(const uint8_t address[6], int8_t rssi, const char* name);

/** The transport the current host talks on; Unknown until one is heard. */
audiolink::Transport activeTransport();

}  // namespace hostlink
