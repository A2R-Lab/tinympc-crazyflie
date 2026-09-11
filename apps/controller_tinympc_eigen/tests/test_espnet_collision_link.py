"""Run the production byte-stream parser and coherent snapshots on the host."""
from pathlib import Path
import subprocess
import tempfile
import unittest


class EspnetCollisionLinkTest(unittest.TestCase):
    def test_receiver(self):
        root = Path(__file__).resolve().parents[3]
        src = root / "apps/controller_tinympc_eigen/src"
        source = (src / "espnet_collision_link.c").read_text().split(
            "void espnetCollisionLinkInit(void)", 1)[0]
        excluded = {f'#include "{h}"' for h in (
            "FreeRTOS.h", "task.h", "log.h", "system.h", "uart1.h")}
        source = "\n".join(line for line in source.splitlines() if line not in excluded)
        harness = r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <setjmp.h>
#include <pthread.h>
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
#define taskENTER_CRITICAL() assert(pthread_mutex_lock(&mutex) == 0)
#define taskEXIT_CRITICAL() assert(pthread_mutex_unlock(&mutex) == 0)
#define portTICK_PERIOD_MS 1
#define systemWaitStart() ((void)0)
static uint32_t now;
static uint32_t xTaskGetTickCount(void) { return now; }
typedef struct { uint32_t (*acquireUInt32)(uint32_t, void *); void *data; } logByFunction_t;
static uint16_t stream[4096];
static size_t cursor, stream_size;
static jmp_buf exhausted;
static bool uart1GetDataWithDefaultTimeout(uint8_t *out) {
  if (cursor == stream_size) longjmp(exhausted, 1);
  const uint16_t value = stream[cursor++];
  if (value == 256) return false;
  *out = value;
  return true;
}
''' + source + r'''
static void put16(uint8_t *p, unsigned value) { p[0] = value; p[1] = value >> 8; }
static void put32(uint8_t *p, uint32_t value) {
  for (unsigned i = 0; i < 4; ++i) p[i] = value >> (8 * i);
}
static void checksum(uint8_t *p) { put32(p + 18, crc32CalculateBuffer(p, 18)); }
static void packet(uint8_t *p, unsigned sequence, unsigned center) {
  memset(p, 0, 22); memcpy(p, header, 4);
  put32(p + 4, sequence + 100); put16(p + 8, sequence);
  put16(p + 10, 1); put16(p + 12, center); put16(p + 14, 32768);
  p[16] = 1; checksum(p);
}
static void append(const uint8_t *p, size_t size) {
  assert(stream_size + size < 4096);
  for (size_t i = 0; i < size; ++i) stream[stream_size++] = p[i];
}
static void receive(void) {
  cursor = 0;
  if (!setjmp(exhausted)) collisionRxTask(NULL);
  stream_size = 0;
}
static void *writer(void *unused) {
  (void)unused;
  for (unsigned i = 0; i < 100000; ++i) {
    uint8_t p[22]; packet(p, 1 + i % 30000, 1 + i % 30000);
    acceptPacket(p);
  }
  return NULL;
}
int main(void) {
  (void)initialized; (void)age_log; (void)gate_age_log; (void)dg_age_log;
  (void)perception_map_age_log;
  assert(crc32CalculateBuffer("123456789", 9) == 0xcbf43926u);
  EspnetCollisionObservation out;
  assert(!espnetCollisionLinkGetLatest(NULL));
  assert(!espnetCollisionLinkGetLatest(&out));
  assert(!out.valid && out.sample == 0 && out.received_age_ms == UINT32_MAX);
  uint8_t p[22]; packet(p, 1, 24576); now = 100;
  /* Noise and truncated candidate header must resynchronize byte-by-byte. */
  append(p, 13); append(p, 22); receive();
  assert(espnetCollisionLinkGetLatest(&out));
  assert(out.valid && out.sample == 1 && out.sequence == 1);
  assert(out.probability[0] == 1.0f / 32768.0f);
  assert(out.probability[1] == 0.75f && out.probability[2] == 1.0f);
  assert(out.source_timestamp_ms == 101 && out.received_age_ms == 0);
  now = 130; append(p, 22); receive();
  assert(espnetCollisionLinkGetLatest(&out) && out.sample == 1);
  assert(out.received_age_ms == 30); /* Duplicate cannot prolong validity. */
  for (unsigned bad = 0; bad < 10; ++bad) {
    packet(p, 2, 24577);
    if (bad == 0) put16(p + 8, 0);
    if (bad >= 1 && bad <= 3) put16(p + 8 + 2 * bad, 32769);
    if (bad == 4) p[16] = 2;
    if (bad == 5) p[17] = 1;
    if (bad >= 6 && bad <= 8) {
      memset(p + 10, 0, 6); p[16] = 0; put16(p + 10 + 2 * (bad - 6), 1);
    }
    checksum(p); if (bad == 9) p[21] ^= 1;
    append(p, 22); receive();
  }
  assert(invalid_packets == 9);
  assert(espnetCollisionLinkGetLatest(&out) && out.sample == 1);
  assert(out.probability[1] == 0.75f && out.received_age_ms == 30);
  packet(p, 2, 24577);
  append(p, 10); stream[stream_size++] = 256; append(p + 10, 12); receive();
  assert(espnetCollisionLinkGetLatest(&out) && out.sample == 1);
  append(p, 22); receive();
  assert(espnetCollisionLinkGetLatest(&out) && out.probability[1] > 0.75f);
  packet(p, 3, 0); memset(p + 10, 0, 6); checksum(p); append(p, 22); receive();
  assert(espnetCollisionLinkGetLatest(&out) && out.valid && out.probability[1] == 0);
  put16(p + 8, 4); p[16] = 0; checksum(p); append(p, 22); receive();
  assert(espnetCollisionLinkGetLatest(&out) && !out.valid && out.sample == 4);
  now = UINT32_MAX - 4; packet(p, 65535, 1); append(p, 22); receive();
  now = 5; assert(espnetCollisionLinkGetLatest(&out) && out.received_age_ms == 10);
  packet(p, 1, 1); append(p, 22); receive();
  assert(espnetCollisionLinkGetLatest(&out) && out.sequence == 1 && out.sample == 6);
  /* Real snapshot + writer code contend under the same task-exclusion rule. */
  writer(NULL); pthread_t thread;
  assert(pthread_create(&thread, NULL, writer, NULL) == 0);
  for (unsigned i = 0; i < 100000; ++i) {
    assert(espnetCollisionLinkGetLatest(&out) && out.valid);
    assert(out.source_timestamp_ms == out.sequence + 100u);
    assert(out.probability[1] == out.sequence / 32768.0f);
    assert(out.received_age_ms == 0);
  }
  assert(pthread_join(thread, NULL) == 0);
  EspnetGateObservation gate;
  assert(!espnetGateLinkGetLatest(NULL));
  assert(!espnetGateLinkGetLatest(&gate) && gate.received_age_ms == UINT32_MAX);
  uint8_t g[52] = {0}; memcpy(g, gate_header, 4);
  put32(g + 4, 1234); put16(g + 8, 1); g[10] = 1; g[11] = 1; g[12] = 5;
  for (unsigned i = 0; i < 17; ++i) put16(g + 14 + 2*i, i * 1000);
  put32(g + 48, crc32CalculateBuffer(g, 48));
  /* Both packet types can coexist, with arbitrary preceding noise. */
  now = 200; packet(p, 45000, 24000);
  append(g, 17); append(p, 22); append(g, 52); receive();
  assert(espnetGateLinkGetLatest(&gate) && gate.sample == 1 && gate.valid);
  assert(gate.corner_edge_mask == 5 && gate.source_timestamp_ms == 1234);
  assert(gate.corner_x[0] == 5000.0f/32768 && gate.corner_y[3] == 15000.0f/32768);
  assert(gate.corner_confidence[3] == 16000.0f/32768);
  assert(espnetCollisionLinkGetLatest(&out) && out.sequence == 45000);
  now = 240; append(g, 52); receive();
  assert(espnetGateLinkGetLatest(&gate) && gate.sample == 1 && gate.received_age_ms == 40);
  uint8_t badgate[52];
  for (unsigned bad = 0; bad < 24; ++bad) {
    memcpy(badgate, g, 52); put16(badgate + 8, 2);
    if (bad == 0) put16(badgate + 8, 0);
    if (bad == 1) badgate[10] = 2;
    if (bad == 2) badgate[11] = 2;
    if (bad == 3) badgate[12] = 16;
    if (bad == 4) badgate[13] = 1;
    if (bad >= 5 && bad < 22) put16(badgate + 14 + 2*(bad-5), 32769);
    if (bad == 22) badgate[11] = 0;
    put32(badgate + 48, crc32CalculateBuffer(badgate, 48));
    if (bad == 23) badgate[51] ^= 1;
    append(badgate, 52); receive();
  }
  assert(gate_invalid_packets == 23 && gate_crc_errors >= 1);
  assert(espnetGateLinkGetLatest(&gate) && gate.sample == 1 && gate.received_age_ms == 40);
  /* Invalid inference is accepted explicitly; it cannot leave a valid gate cached. */
  put16(g + 8, 2); memset(g + 11, 0, 37); put32(g + 48, crc32CalculateBuffer(g, 48));
  append(g, 52); receive();
  assert(espnetGateLinkGetLatest(&gate) && !gate.valid && gate.sample == 2);
  put16(g + 8, 65535); put32(g + 48, crc32CalculateBuffer(g, 48));
  append(g, 20); stream[stream_size++] = 256; append(g + 20, 32); receive();
  assert(espnetGateLinkGetLatest(&gate) && gate.sample == 2);
  now = UINT32_MAX - 4; append(g, 52); receive();
  now = 5; assert(espnetGateLinkGetLatest(&gate) && gate.received_age_ms == 10);
  put16(g + 8, 1); put32(g + 48, crc32CalculateBuffer(g, 48));
  append(g, 52); receive();
  assert(espnetGateLinkGetLatest(&gate) && gate.sample == 4 && gate.sequence == 1);
  /* NanoCockpit dense perception-map v2 coexists on the same byte stream. */
  PerceptionMapObservation map;
  assert(!perceptionMapLinkGetLatest(NULL));
  assert(!perceptionMapLinkGetLatest(&map) && map.received_age_ms == UINT32_MAX);
  uint8_t m[PERCEPTION_MAP_PACKET_SIZE] = {0};
  memcpy(m, perception_map_header, 4);
  put32(m + 4, 900000); put32(m + 8, 321); put16(m + 12, 7);
  m[14] = PERCEPTION_MAP_WIRE_VERSION;
  m[15] = PERCEPTION_MAP_WIDTH; m[16] = PERCEPTION_MAP_HEIGHT;
  /* cell (3,4): collision=1, inverse range=10, uncertainty=2, gate=15 */
  const unsigned cell = 4 * PERCEPTION_MAP_WIDTH + 3;
  m[18 + 2 * cell] = 0xa1; m[18 + 2 * cell + 1] = 0xf2;
  put32(m + 218, crc32CalculateBuffer(m, 218));
  now = 500; append(m, 91); append(m + 91, sizeof(m) - 91); receive();
  assert(perceptionMapLinkGetLatest(&map) && map.sample == 1);
  assert(map.sequence == 7 && map.gap8_timestamp_us == 900000);
  assert(map.stm32_timestamp_echo == 321 && map.received_age_ms == 0);
  assert(perceptionMapValue(&map, 3, 4, PERCEPTION_MAP_COLLISION) == 1);
  assert(perceptionMapValue(&map, 3, 4, PERCEPTION_MAP_INVERSE_RANGE) == 10);
  assert(perceptionMapValue(&map, 3, 4, PERCEPTION_MAP_UNCERTAINTY) == 2);
  assert(perceptionMapValue(&map, 3, 4, PERCEPTION_MAP_GATE_OPENING) == 15);
  assert(perceptionMapValue(&map, 10, 0, PERCEPTION_MAP_COLLISION) == 0);
  now = 525; append(m, sizeof(m)); receive();
  assert(perceptionMapLinkGetLatest(&map) && map.sample == 1);
  assert(map.received_age_ms == 25); /* Duplicate does not refresh age. */
  m[14] = 3; put32(m + 218, crc32CalculateBuffer(m, 218));
  append(m, sizeof(m)); receive(); assert(perception_map_invalid_packets == 1);
  m[14] = PERCEPTION_MAP_WIRE_VERSION; m[221] ^= 1;
  append(m, sizeof(m)); receive(); assert(perception_map_crc_errors == 1);
  return 0;
}
'''
        with tempfile.TemporaryDirectory(prefix="espnet-rx-") as directory:
            temporary = Path(directory)
            (temporary / "static_mem.h").write_text("#define NO_DMA_CCM_SAFE_ZERO_INIT\n")
            include = root / "crazyflie-firmware/src/utils/interface"
            crc = temporary / "crc.o"
            subprocess.run([
                "cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-Wno-sign-compare",
                "-I", directory, "-I", str(include), "-c",
                str(root / "crazyflie-firmware/src/utils/src/crc32.c"), "-o", str(crc),
            ], check=True)
            binary = temporary / "receiver"
            subprocess.run([
                "cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-O2", "-pthread",
                "-I", str(src), "-I", str(include), "-x", "c", "-", "-x", "none",
                str(crc), "-o", str(binary),
            ], input=harness, text=True, check=True)
            subprocess.run([str(binary)], check=True, timeout=5)


if __name__ == "__main__":
    unittest.main()
