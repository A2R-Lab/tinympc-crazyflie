#pragma once

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define TINYMPC_SITL_MMAP_MAGIC 0x544d444du
#define TINYMPC_SITL_MMAP_VERSION 1u
#define TINYMPC_SITL_MMAP_HEADER_SIZE 4096u
#define TINYMPC_SITL_MMAP_SLOT_SIZE 3872u
#define TINYMPC_SITL_MMAP_RECORD_SIZE 3848u
#define TINYMPC_SITL_MMAP_CAPACITY 16384u
#define TINYMPC_SITL_MMAP_FILE_SIZE 63442944u
#define TINYMPC_SITL_MMAP_NEXT_SEQUENCE_OFFSET 64u
#define TINYMPC_SITL_MMAP_SLOTS_OFFSET 4096u
#define TINYMPC_SITL_MMAP_COMMIT_OFFSET 0u
#define TINYMPC_SITL_MMAP_RESERVATION_OFFSET 8u
#define TINYMPC_SITL_MMAP_PAYLOAD_OFFSET 16u
#define TINYMPC_SITL_MMAP_CHECKSUM_OFFSET 3864u
#define TINYMPC_SITL_MMAP_RESERVED_OFFSET 3868u
#define TINYMPC_SITL_MMAP_FNV_OFFSET_BASIS 2166136261u
#define TINYMPC_SITL_MMAP_FNV_PRIME 16777619u

typedef struct __attribute__((aligned(64))) {
  uint32_t magic;
  uint16_t version;
  uint16_t header_size;
  uint32_t slot_size;
  uint32_t record_size;
  uint32_t capacity;
  uint32_t firmware_tick_us;
  uint32_t payload_magic;
  uint16_t payload_version;
  uint16_t reserved0;
  uint8_t reserved_to_sequence[32];
  uint64_t next_sequence;
  uint64_t overflow_count;
  uint64_t committed_count;
  uint8_t reserved[TINYMPC_SITL_MMAP_HEADER_SIZE - 88u];
} TinyMpcSitlMmapDiagHeader;

typedef struct {
  uint64_t commit_sequence;
  uint64_t reservation_sequence;
  uint8_t payload[TINYMPC_SITL_MMAP_RECORD_SIZE];
  uint32_t checksum;
  uint32_t reserved;
} TinyMpcSitlMmapDiagSlot;

#if defined(__cplusplus)
static_assert(sizeof(TinyMpcSitlMmapDiagHeader) ==
                  TINYMPC_SITL_MMAP_HEADER_SIZE,
              "SITL mmap diagnostic header layout changed");
static_assert(alignof(TinyMpcSitlMmapDiagHeader) == 64u,
              "SITL mmap diagnostic header alignment changed");
static_assert(offsetof(TinyMpcSitlMmapDiagHeader, next_sequence) ==
                  TINYMPC_SITL_MMAP_NEXT_SEQUENCE_OFFSET,
              "SITL mmap next_sequence offset changed");
static_assert(sizeof(TinyMpcSitlMmapDiagSlot) == TINYMPC_SITL_MMAP_SLOT_SIZE,
              "SITL mmap diagnostic slot layout changed");
static_assert(offsetof(TinyMpcSitlMmapDiagSlot, commit_sequence) ==
                  TINYMPC_SITL_MMAP_COMMIT_OFFSET,
              "SITL mmap commit offset changed");
static_assert(offsetof(TinyMpcSitlMmapDiagSlot, reservation_sequence) ==
                  TINYMPC_SITL_MMAP_RESERVATION_OFFSET,
              "SITL mmap reservation offset changed");
static_assert(offsetof(TinyMpcSitlMmapDiagSlot, payload) ==
                  TINYMPC_SITL_MMAP_PAYLOAD_OFFSET,
              "SITL mmap payload offset changed");
static_assert(offsetof(TinyMpcSitlMmapDiagSlot, checksum) ==
                  TINYMPC_SITL_MMAP_CHECKSUM_OFFSET,
              "SITL mmap checksum offset changed");
static_assert(offsetof(TinyMpcSitlMmapDiagSlot, reserved) ==
                  TINYMPC_SITL_MMAP_RESERVED_OFFSET,
              "SITL mmap reserved offset changed");
static_assert(TINYMPC_SITL_MMAP_FILE_SIZE ==
                  TINYMPC_SITL_MMAP_HEADER_SIZE +
                      TINYMPC_SITL_MMAP_CAPACITY *
                          TINYMPC_SITL_MMAP_SLOT_SIZE,
              "SITL mmap file-size formula changed");
#else
_Static_assert(sizeof(TinyMpcSitlMmapDiagHeader) ==
                   TINYMPC_SITL_MMAP_HEADER_SIZE,
               "SITL mmap diagnostic header layout changed");
_Static_assert(offsetof(TinyMpcSitlMmapDiagHeader, next_sequence) ==
                   TINYMPC_SITL_MMAP_NEXT_SEQUENCE_OFFSET,
               "SITL mmap next_sequence offset changed");
_Static_assert(sizeof(TinyMpcSitlMmapDiagSlot) == TINYMPC_SITL_MMAP_SLOT_SIZE,
               "SITL mmap diagnostic slot layout changed");
#endif

static inline uint32_t tinyMpcSitlMmapDiagChecksum(
    uint64_t reservation_sequence,
    const uint8_t payload[TINYMPC_SITL_MMAP_RECORD_SIZE]) {
  uint32_t checksum = TINYMPC_SITL_MMAP_FNV_OFFSET_BASIS;
  const uint8_t *sequence_bytes =
      (const uint8_t *)&reservation_sequence;
  for (size_t index = 0u; index < sizeof(reservation_sequence); ++index) {
    checksum ^= sequence_bytes[index];
    checksum *= TINYMPC_SITL_MMAP_FNV_PRIME;
  }
  for (size_t index = 0u; index < TINYMPC_SITL_MMAP_RECORD_SIZE; ++index) {
    checksum ^= payload[index];
    checksum *= TINYMPC_SITL_MMAP_FNV_PRIME;
  }
  return checksum;
}

static inline uint32_t tinyMpcSitlMmapDiagChecksumRaw(
    const void *bytes, size_t count) {
  uint32_t checksum = TINYMPC_SITL_MMAP_FNV_OFFSET_BASIS;
  const uint8_t *data = (const uint8_t *)bytes;
  for (size_t index = 0u; index < count; ++index) {
    checksum ^= data[index];
    checksum *= TINYMPC_SITL_MMAP_FNV_PRIME;
  }
  return checksum;
}

static inline void tinyMpcSitlMmapDiagInitialize(
    TinyMpcSitlMmapDiagHeader *header, uint32_t firmware_tick_us,
    uint32_t payload_magic, uint16_t payload_version) {
  memset(header, 0, TINYMPC_SITL_MMAP_FILE_SIZE);
  header->magic = TINYMPC_SITL_MMAP_MAGIC;
  header->version = TINYMPC_SITL_MMAP_VERSION;
  header->header_size = TINYMPC_SITL_MMAP_HEADER_SIZE;
  header->slot_size = TINYMPC_SITL_MMAP_SLOT_SIZE;
  header->record_size = TINYMPC_SITL_MMAP_RECORD_SIZE;
  header->capacity = TINYMPC_SITL_MMAP_CAPACITY;
  header->firmware_tick_us = firmware_tick_us;
  header->payload_magic = payload_magic;
  header->payload_version = payload_version;
  __atomic_store_n(&header->next_sequence, 1u, __ATOMIC_RELAXED);
}

static inline TinyMpcSitlMmapDiagSlot *tinyMpcSitlMmapDiagReserve(
    TinyMpcSitlMmapDiagHeader *header, uint64_t *reservation_sequence) {
  const uint64_t sequence = __atomic_fetch_add(
      &header->next_sequence, 1u, __ATOMIC_RELAXED);
  *reservation_sequence = sequence;
  if (sequence == 0u || sequence > TINYMPC_SITL_MMAP_CAPACITY) {
    __atomic_add_fetch(&header->overflow_count, 1u, __ATOMIC_RELAXED);
    return NULL;
  }
  uint8_t *base = (uint8_t *)header + TINYMPC_SITL_MMAP_HEADER_SIZE;
  return (TinyMpcSitlMmapDiagSlot *)(
      base + (sequence - 1u) * TINYMPC_SITL_MMAP_SLOT_SIZE);
}

static inline void tinyMpcSitlMmapDiagCommit(
    TinyMpcSitlMmapDiagHeader *header, TinyMpcSitlMmapDiagSlot *slot,
    uint64_t reservation_sequence,
    const uint8_t payload[TINYMPC_SITL_MMAP_RECORD_SIZE]) {
  slot->reservation_sequence = reservation_sequence;
  memcpy(slot->payload, payload, TINYMPC_SITL_MMAP_RECORD_SIZE);
  slot->checksum = tinyMpcSitlMmapDiagChecksumRaw(
      &slot->reservation_sequence,
      sizeof(slot->reservation_sequence) + sizeof(slot->payload));
  slot->reserved = 0u;
  __atomic_store_n(
      &slot->commit_sequence, reservation_sequence, __ATOMIC_RELEASE);
  __atomic_add_fetch(&header->committed_count, 1u, __ATOMIC_RELAXED);
}

static inline int tinyMpcSitlMmapDiagSlotValid(
    const TinyMpcSitlMmapDiagSlot *slot) {
  const uint64_t commit = __atomic_load_n(
      &slot->commit_sequence, __ATOMIC_ACQUIRE);
  if (commit == 0u || commit != slot->reservation_sequence) {
    return 0;
  }
  return slot->checksum == tinyMpcSitlMmapDiagChecksumRaw(
      &slot->reservation_sequence,
      sizeof(slot->reservation_sequence) + sizeof(slot->payload));
}
