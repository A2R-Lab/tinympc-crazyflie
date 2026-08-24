#include "tinympc_sitl_mmap_diag.h"

#include <assert.h>
#include <stdlib.h>

#include <algorithm>
#include <array>
#include <thread>
#include <vector>

static TinyMpcSitlMmapDiagHeader *allocateMapping(void) {
  void *memory = aligned_alloc(64u, TINYMPC_SITL_MMAP_FILE_SIZE);
  assert(memory != nullptr);
  return static_cast<TinyMpcSitlMmapDiagHeader *>(memory);
}

static std::array<uint8_t, TINYMPC_SITL_MMAP_RECORD_SIZE> payloadFor(
    uint32_t producer, uint32_t index) {
  std::array<uint8_t, TINYMPC_SITL_MMAP_RECORD_SIZE> payload{};
  memcpy(payload.data(), &producer, sizeof(producer));
  memcpy(payload.data() + sizeof(producer), &index, sizeof(index));
  for (size_t byte = 2u * sizeof(uint32_t); byte < payload.size(); ++byte) {
    payload[byte] = static_cast<uint8_t>(producer * 31u + index * 7u + byte);
  }
  return payload;
}

static void testConcurrentProducers(TinyMpcSitlMmapDiagHeader *mapping) {
  tinyMpcSitlMmapDiagInitialize(mapping, 1250u, 0x544d5043u, 2u);
  assert(mapping->magic == TINYMPC_SITL_MMAP_MAGIC);
  assert(mapping->version == TINYMPC_SITL_MMAP_VERSION);
  assert(mapping->header_size == TINYMPC_SITL_MMAP_HEADER_SIZE);
  assert(mapping->slot_size == TINYMPC_SITL_MMAP_SLOT_SIZE);
  assert(mapping->record_size == TINYMPC_SITL_MMAP_RECORD_SIZE);
  assert(mapping->capacity == TINYMPC_SITL_MMAP_CAPACITY);
  assert(mapping->firmware_tick_us == 1250u);
  assert(mapping->payload_magic == 0x544d5043u);
  assert(mapping->payload_version == 2u);
  constexpr uint32_t per_producer = 1000u;
  std::array<std::vector<uint64_t>, 2> sequences;
  auto producer = [&](uint32_t producer_id) {
    sequences[producer_id].reserve(per_producer);
    for (uint32_t index = 0u; index < per_producer; ++index) {
      uint64_t sequence = 0u;
      TinyMpcSitlMmapDiagSlot *slot = tinyMpcSitlMmapDiagReserve(
          mapping, &sequence);
      assert(slot != nullptr);
      const auto payload = payloadFor(producer_id, index);
      tinyMpcSitlMmapDiagCommit(
          mapping, slot, sequence, payload.data());
      sequences[producer_id].push_back(sequence);
    }
  };
  std::thread first(producer, 0u);
  std::thread second(producer, 1u);
  first.join();
  second.join();

  std::vector<uint64_t> all;
  all.insert(all.end(), sequences[0].begin(), sequences[0].end());
  all.insert(all.end(), sequences[1].begin(), sequences[1].end());
  std::sort(all.begin(), all.end());
  assert(all.size() == 2u * per_producer);
  for (size_t index = 0u; index < all.size(); ++index) {
    assert(all[index] == index + 1u);
  }
  assert(__atomic_load_n(&mapping->next_sequence, __ATOMIC_RELAXED) ==
         2u * per_producer + 1u);
  assert(__atomic_load_n(&mapping->committed_count, __ATOMIC_RELAXED) ==
         2u * per_producer);
  auto *slots = reinterpret_cast<TinyMpcSitlMmapDiagSlot *>(
      reinterpret_cast<uint8_t *>(mapping) + TINYMPC_SITL_MMAP_HEADER_SIZE);
  for (size_t index = 0u; index < all.size(); ++index) {
    assert(tinyMpcSitlMmapDiagSlotValid(&slots[index]));
  }
}

static void testOutOfOrderAndTornSlots(
    TinyMpcSitlMmapDiagHeader *mapping) {
  tinyMpcSitlMmapDiagInitialize(mapping, 1250u, 0x544d5043u, 2u);
  uint64_t first_sequence = 0u;
  uint64_t second_sequence = 0u;
  TinyMpcSitlMmapDiagSlot *first = tinyMpcSitlMmapDiagReserve(
      mapping, &first_sequence);
  TinyMpcSitlMmapDiagSlot *second = tinyMpcSitlMmapDiagReserve(
      mapping, &second_sequence);
  assert(first != nullptr && second != nullptr);
  const auto first_payload = payloadFor(7u, 1u);
  const auto second_payload = payloadFor(7u, 2u);
  tinyMpcSitlMmapDiagCommit(
      mapping, second, second_sequence, second_payload.data());
  assert(second->checksum == tinyMpcSitlMmapDiagChecksumRaw(
      &second->reservation_sequence,
      sizeof(second->reservation_sequence) + sizeof(second->payload)));
  assert(!tinyMpcSitlMmapDiagSlotValid(first));
  assert(tinyMpcSitlMmapDiagSlotValid(second));
  tinyMpcSitlMmapDiagCommit(
      mapping, first, first_sequence, first_payload.data());
  assert(tinyMpcSitlMmapDiagSlotValid(first));

  uint64_t torn_sequence = 0u;
  TinyMpcSitlMmapDiagSlot *torn = tinyMpcSitlMmapDiagReserve(
      mapping, &torn_sequence);
  assert(torn != nullptr);
  torn->reservation_sequence = torn_sequence;
  memcpy(torn->payload, first_payload.data(), first_payload.size() / 2u);
  torn->checksum = tinyMpcSitlMmapDiagChecksum(
      torn_sequence, torn->payload);
  assert(!tinyMpcSitlMmapDiagSlotValid(torn));
  tinyMpcSitlMmapDiagCommit(
      mapping, torn, torn_sequence, first_payload.data());
  assert(tinyMpcSitlMmapDiagSlotValid(torn));
  torn->payload[17] ^= 0x80u;
  assert(!tinyMpcSitlMmapDiagSlotValid(torn));

  uint64_t killed_sequence = 0u;
  TinyMpcSitlMmapDiagSlot *killed = tinyMpcSitlMmapDiagReserve(
      mapping, &killed_sequence);
  assert(killed != nullptr);
  killed->reservation_sequence = killed_sequence;
  memcpy(killed->payload, second_payload.data(), second_payload.size());
  __atomic_store_n(
      &killed->commit_sequence, killed_sequence, __ATOMIC_RELEASE);
  assert(!tinyMpcSitlMmapDiagSlotValid(killed));
}

static void testCapacityOverflow(TinyMpcSitlMmapDiagHeader *mapping) {
  tinyMpcSitlMmapDiagInitialize(mapping, 1250u, 0x544d5043u, 2u);
  __atomic_store_n(
      &mapping->next_sequence,
      static_cast<uint64_t>(TINYMPC_SITL_MMAP_CAPACITY),
      __ATOMIC_RELAXED);
  uint64_t last_sequence = 0u;
  TinyMpcSitlMmapDiagSlot *last = tinyMpcSitlMmapDiagReserve(
      mapping, &last_sequence);
  assert(last != nullptr);
  assert(last_sequence == TINYMPC_SITL_MMAP_CAPACITY);
  uint64_t overflow_sequence = 0u;
  assert(tinyMpcSitlMmapDiagReserve(mapping, &overflow_sequence) == nullptr);
  assert(overflow_sequence == TINYMPC_SITL_MMAP_CAPACITY + 1u);
  assert(__atomic_load_n(&mapping->overflow_count, __ATOMIC_RELAXED) == 1u);
  assert(__atomic_load_n(&mapping->next_sequence, __ATOMIC_RELAXED) ==
         TINYMPC_SITL_MMAP_CAPACITY + 2u);
}

int main(void) {
  static_assert(__atomic_always_lock_free(8, 0),
                "host test requires lock-free 64-bit atomics");
  TinyMpcSitlMmapDiagHeader *mapping = allocateMapping();
  testConcurrentProducers(mapping);
  testOutOfOrderAndTornSlots(mapping);
  testCapacityOverflow(mapping);
  free(mapping);
  return 0;
}
