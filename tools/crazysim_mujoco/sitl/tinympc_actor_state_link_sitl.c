#include "tinympc_actor_state_link.h"

#include "crc32.h"

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define TINYMPC_ACTOR_STATE_DEFAULT_PORT 19961

typedef struct __attribute__((packed)) {
  uint8_t magic[8];
  uint16_t version;
  uint16_t payload_floats;
  uint32_t sequence;
  uint32_t firmware_tick_ms;
  float state[TINYMPC_ACTOR_STATE_FLOATS];
  uint32_t checksum;
} TinyMpcActorStatePacket;

_Static_assert(sizeof(TinyMpcActorStatePacket) == 120u,
               "actor-state packet layout changed");

static const uint8_t packet_magic[8] = {'T', 'M', 'A', 'S', '2', '4', 'V', '1'};
static int actor_state_socket = -1;
static struct sockaddr_in actor_state_destination;
static uint32_t actor_state_sequence;
static bool actor_state_init_attempted;

static void initializeActorStateLink(void) {
  if (actor_state_init_attempted) {
    return;
  }
  actor_state_init_attempted = true;
  const char *port_text = getenv("TINYMPC_ACTOR_STATE_PORT");
  const long port = port_text != NULL ? strtol(port_text, NULL, 10)
                                      : TINYMPC_ACTOR_STATE_DEFAULT_PORT;
  if (port < 1 || port > 65535) {
    fprintf(stderr, "Invalid TINYMPC_ACTOR_STATE_PORT=%s\n",
            port_text != NULL ? port_text : "");
    return;
  }
  actor_state_socket = socket(AF_INET, SOCK_DGRAM, 0);
  if (actor_state_socket < 0) {
    perror("TinyMPC actor-state socket");
    return;
  }
  const int flags = fcntl(actor_state_socket, F_GETFL, 0);
  if (flags < 0 || fcntl(actor_state_socket, F_SETFL, flags | O_NONBLOCK) < 0) {
    perror("TinyMPC actor-state nonblocking socket");
    close(actor_state_socket);
    actor_state_socket = -1;
    return;
  }
  memset(&actor_state_destination, 0, sizeof(actor_state_destination));
  actor_state_destination.sin_family = AF_INET;
  actor_state_destination.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  actor_state_destination.sin_port = htons((uint16_t)port);
  printf("TinyMPC actor state publishing to udp://127.0.0.1:%ld\n", port);
}

void tinyMpcActorStateLinkPublish(
    const float state[TINYMPC_ACTOR_STATE_FLOATS], uint32_t firmware_tick_ms) {
  initializeActorStateLink();
  if (actor_state_socket < 0 || state == NULL) {
    return;
  }
  for (uint32_t index = 0u; index < TINYMPC_ACTOR_STATE_FLOATS; ++index) {
    if (!isfinite(state[index])) {
      return;
    }
  }
  TinyMpcActorStatePacket packet;
  memset(&packet, 0, sizeof(packet));
  memcpy(packet.magic, packet_magic, sizeof(packet.magic));
  packet.version = 1u;
  packet.payload_floats = TINYMPC_ACTOR_STATE_FLOATS;
  packet.sequence = ++actor_state_sequence;
  packet.firmware_tick_ms = firmware_tick_ms;
  memcpy(packet.state, state, sizeof(packet.state));
  packet.checksum = crc32CalculateBuffer(
      &packet, sizeof(packet) - sizeof(packet.checksum));
  const ssize_t sent = sendto(
      actor_state_socket, &packet, sizeof(packet), 0,
      (const struct sockaddr *)&actor_state_destination,
      sizeof(actor_state_destination));
  if (sent < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
    static bool reported;
    if (!reported) {
      perror("TinyMPC actor-state send");
      reported = true;
    }
  }
}
