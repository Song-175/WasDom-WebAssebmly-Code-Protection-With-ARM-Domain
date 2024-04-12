// Copyright 2022 the V8 project authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef V8_COMMON_CODE_MEMORY_ACCESS_INL_H_
#define V8_COMMON_CODE_MEMORY_ACCESS_INL_H_

#include "src/common/code-memory-access.h"
#include "src/flags/flags.h"
#include "src/objects/instruction-stream-inl.h"
#include "src/objects/instruction-stream.h"
#include "src/objects/slots-inl.h"
#include "src/objects/tagged.h"
#if V8_HAS_PKU_JIT_WRITE_PROTECT
#include "src/base/platform/memory-protection-key.h"
#endif
#if V8_HAS_PTHREAD_JIT_WRITE_PROTECT
#include "src/base/platform/platform.h"
#endif

//added by song 20240108
#include <stdio.h>  
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>

//added by song 20230108
#define WASDOM_MMAP_ADDR 0x40C00000U            // We use fixed mmapped addresss
#define WASDOM_MMAP_SIZE 0x100000              // 1MB
#define WASDOM_DOMAIN_NUM   4
#define WASDOM_DOMAIN_OFFSET    2*WASDOM_DOMAIN_NUM
#define WASDOM_DOMAIN_MASK  ~(0x3<<(WASDOM_DOMAIN_OFFSET))

#define WASDOM_DACR_NOACCESS  0
#define WASDOM_DACR_CLIENT    1       // during execute
#define WASDOM_DACR_MANAGER   3       // during jit compile

#define WASDOM_CMD_WR_DOMAIN    _IOW('a', '0', int32_t*)
#define WASDOM_CMD_RD_DOMAIN    _IOR('a', '1', int32_t*)
#define WASDOM_CMD_WR_DACR      _IOW('a', '2', int32_t*)
#define WASDOM_CMD_RD_DACR      _IOR('a', '3', int32_t*)
#define WASDOM_CMD_WR_TABLE     _IOW('a', '4', int32_t*)
#define WASDOM_CMD_RD_TABLE     _IOR('a', '5', int32_t*)
#define WASDOM_CMD_FIND_DOMAIN  _IOW('a', '6', int32_t*)


namespace v8 {
namespace internal {



RwxMemoryWriteScope::RwxMemoryWriteScope(const char* comment) {
  if (!v8_flags.jitless) {
    SetWritable();
  }
}

RwxMemoryWriteScope::~RwxMemoryWriteScope() {
  if (!v8_flags.jitless) {
    SetExecutable();
  }
}

WritableJitAllocation::~WritableJitAllocation() = default;

WritableJitAllocation::WritableJitAllocation(
    Address addr, size_t size, ThreadIsolation::JitAllocationType type,
    JitAllocationSource source)
    : address_(addr),
      // The order of these is important. We need to create the write scope
      // before we lookup the Jit page, since the latter will take a mutex in
      // protected memory.
      write_scope_("WritableJitAllocation"),
      page_ref_(ThreadIsolation::LookupJitPage(addr, size)),
      allocation_(source == JitAllocationSource::kRegister
                      ? page_ref_->RegisterAllocation(addr, size, type)
                      : page_ref_->LookupAllocation(addr, size, type)) {}

WritableJitAllocation::WritableJitAllocation(
    Address addr, size_t size, ThreadIsolation::JitAllocationType type)
    : address_(addr), allocation_(size, type) {}

// static
WritableJitAllocation WritableJitAllocation::ForNonExecutableMemory(
    Address addr, size_t size, ThreadIsolation::JitAllocationType type) {
  return WritableJitAllocation(addr, size, type);
}

// static
WritableJitAllocation WritableJitAllocation::ForInstructionStream(
    Tagged<InstructionStream> istream) {
  return WritableJitAllocation(
      istream->address(), istream->Size(),
      ThreadIsolation::JitAllocationType::kInstructionStream,
      JitAllocationSource::kLookup);
}

WritableJumpTablePair::WritableJumpTablePair(Address jump_table_address,
                                             size_t jump_table_size,
                                             Address far_jump_table_address,
                                             size_t far_jump_table_size)
    : write_scope_("WritableJumpTablePair"),
      // Always split the pages since we are not guaranteed that the jump table
      // and far jump table are on the same JitPage.
      jump_table_pages_(ThreadIsolation::SplitJitPages(
          far_jump_table_address, far_jump_table_size, jump_table_address,
          jump_table_size)),
      jump_table_(jump_table_pages_.second.LookupAllocation(
          jump_table_address, jump_table_size,
          ThreadIsolation::JitAllocationType::kWasmJumpTable)),
      far_jump_table_(jump_table_pages_.first.LookupAllocation(
          far_jump_table_address, far_jump_table_size,
          ThreadIsolation::JitAllocationType::kWasmFarJumpTable)) {}

template <typename T, size_t offset>
void WritableJitAllocation::WriteHeaderSlot(T value) {
  // This assert is no strict requirement, it just guards against
  // non-implemented functionality.
  static_assert(!is_taggable_v<T>);

  if constexpr (offset == HeapObject::kMapOffset) {
    TaggedField<T, offset>::Relaxed_Store_Map_Word(
        HeapObject::FromAddress(address_), value);
  } else {
    WriteMaybeUnalignedValue<T>(address_ + offset, value);
  }
}

template <typename T, size_t offset>
void WritableJitAllocation::WriteHeaderSlot(Tagged<T> value, ReleaseStoreTag) {
  // These asserts are no strict requirements, they just guard against
  // non-implemented functionality.
  static_assert(offset != HeapObject::kMapOffset);

  TaggedField<T, offset>::Release_Store(HeapObject::FromAddress(address_),
                                        value);
}

template <typename T, size_t offset>
void WritableJitAllocation::WriteHeaderSlot(Tagged<T> value, RelaxedStoreTag) {
  if constexpr (offset == HeapObject::kMapOffset) {
    TaggedField<T, offset>::Relaxed_Store_Map_Word(
        HeapObject::FromAddress(address_), value);
  } else {
    TaggedField<T, offset>::Relaxed_Store(HeapObject::FromAddress(address_),
                                          value);
  }
}

template <typename T, size_t offset>
void WritableJitAllocation::WriteProtectedPointerHeaderSlot(Tagged<T> value,
                                                            RelaxedStoreTag) {
  static_assert(offset != HeapObject::kMapOffset);
  TaggedField<T, offset, TrustedSpaceCompressionScheme>::Relaxed_Store(
      HeapObject::FromAddress(address_), value);
}

template <typename T>
V8_INLINE void WritableJitAllocation::WriteHeaderSlot(Address address, T value,
                                                      RelaxedStoreTag tag) {
  CHECK_EQ(allocation_.Type(),
           ThreadIsolation::JitAllocationType::kInstructionStream);
  size_t offset = address - address_;
  Tagged<T> tagged(value);
  switch (offset) {
    case InstructionStream::kCodeOffset:
      WriteProtectedPointerHeaderSlot<T, InstructionStream::kCodeOffset>(tagged,
                                                                         tag);
      break;
    case InstructionStream::kRelocationInfoOffset:
      WriteHeaderSlot<T, InstructionStream::kRelocationInfoOffset>(tagged, tag);
      break;
    default:
      UNREACHABLE();
  }
}

void WritableJitAllocation::CopyCode(size_t dst_offset, const uint8_t* src,
                                     size_t num_bytes) {
  CopyBytes(reinterpret_cast<uint8_t*>(address_ + dst_offset), src, num_bytes);
}

void WritableJitAllocation::CopyData(size_t dst_offset, const uint8_t* src,
                                     size_t num_bytes) {
  CopyBytes(reinterpret_cast<uint8_t*>(address_ + dst_offset), src, num_bytes);
}

void WritableJitAllocation::ClearBytes(size_t offset, size_t len) {
  memset(reinterpret_cast<void*>(address_ + offset), 0, len);
}

WritableJitPage::~WritableJitPage() = default;

WritableJitPage::WritableJitPage(Address addr, size_t size)
    : write_scope_("WritableJitPage"),
      page_ref_(ThreadIsolation::LookupJitPage(addr, size)) {}

WritableJitAllocation WritableJitPage::LookupAllocationContaining(
    Address addr) {
  auto pair = page_ref_.AllocationContaining(addr);
  return WritableJitAllocation(pair.first, pair.second.Size(),
                               pair.second.Type());
}

V8_INLINE WritableFreeSpace WritableJitPage::FreeRange(Address addr,
                                                       size_t size) {
  page_ref_.UnregisterRange(addr, size);
  return WritableFreeSpace(addr, size, true);
}

WritableFreeSpace::~WritableFreeSpace() = default;

// static
V8_INLINE WritableFreeSpace
WritableFreeSpace::ForNonExecutableMemory(base::Address addr, size_t size) {
  return WritableFreeSpace(addr, size, false);
}

V8_INLINE WritableFreeSpace::WritableFreeSpace(base::Address addr, size_t size,
                                               bool executable)
    : address_(addr), size_(static_cast<int>(size)), executable_(executable) {}

template <typename T, size_t offset>
void WritableFreeSpace::WriteHeaderSlot(Tagged<T> value,
                                        RelaxedStoreTag) const {
  Tagged<HeapObject> object = HeapObject::FromAddress(address_);
  // TODO(v8:13355): add validation before the write.
  if constexpr (offset == HeapObject::kMapOffset) {
    TaggedField<T, offset>::Relaxed_Store_Map_Word(object, value);
  } else {
    TaggedField<T, offset>::Relaxed_Store(object, value);
  }
}

template <size_t offset>
void WritableFreeSpace::ClearTagged(size_t count) const {
  base::Address start = address_ + offset;
  // TODO(v8:13355): add validation before the write.
  MemsetTagged(ObjectSlot(start), Tagged<Object>(kClearedFreeMemoryValue),
               count);
}

#if V8_HAS_PTHREAD_JIT_WRITE_PROTECT

// static
bool RwxMemoryWriteScope::IsSupported() { return true; }

// static
void RwxMemoryWriteScope::SetWritable() {
  if (code_space_write_nesting_level_ == 0) {
    base::SetJitWriteProtected(0);
  }
  code_space_write_nesting_level_++;
}

// static
void RwxMemoryWriteScope::SetExecutable() {
  code_space_write_nesting_level_--;
  if (code_space_write_nesting_level_ == 0) {
    base::SetJitWriteProtected(1);
  }
}

#elif V8_HAS_PKU_JIT_WRITE_PROTECT
// static
bool RwxMemoryWriteScope::IsSupported() {
  static_assert(base::MemoryProtectionKey::kNoMemoryProtectionKey == -1);
  DCHECK(ThreadIsolation::initialized());
  // TODO(sroettger): can we check this at initialization time instead? The
  // tests won't be able to run with/without pkey support anymore in the same
  // process.
  return v8_flags.memory_protection_keys && ThreadIsolation::pkey() >= 0;
}

// static
void RwxMemoryWriteScope::SetWritable() {
  DCHECK(ThreadIsolation::initialized());
  if (!IsSupported()) return;
  if (code_space_write_nesting_level_ == 0) {
    DCHECK_NE(
        base::MemoryProtectionKey::GetKeyPermission(ThreadIsolation::pkey()),
        base::MemoryProtectionKey::kNoRestrictions);
    base::MemoryProtectionKey::SetPermissionsForKey(
        ThreadIsolation::pkey(), base::MemoryProtectionKey::kNoRestrictions);
  }
  code_space_write_nesting_level_++;
}

// static
void RwxMemoryWriteScope::SetExecutable() {
  DCHECK(ThreadIsolation::initialized());
  if (!IsSupported()) return;
  code_space_write_nesting_level_--;
  if (code_space_write_nesting_level_ == 0) {
    DCHECK_EQ(
        base::MemoryProtectionKey::GetKeyPermission(ThreadIsolation::pkey()),
        base::MemoryProtectionKey::kNoRestrictions);
    base::MemoryProtectionKey::SetPermissionsForKey(
        ThreadIsolation::pkey(), base::MemoryProtectionKey::kDisableWrite);
  }
}

#else  // !V8_HAS_PTHREAD_JIT_WRITE_PROTECT && !V8_TRY_USE_PKU_JIT_WRITE_PROTECT

// static
bool RwxMemoryWriteScope::IsSupported() { return true; }

// static
void RwxMemoryWriteScope::SetWritable() {
    // 메모리 쓰기 가능 설정
    int fd = open("/dev/wasdom_device", O_RDWR);
  /*
  if (fd < 0) {
    perror("Error opening wasdom_device");
    return;
  }
  
  // 현재 DACR 값을 가져옴
  int32_t dacr_val;
  if (ioctl(fd, WASDOM_CMD_RD_DACR, &dacr_val) < 0) {
    perror("Error reading DACR in wasdom_driver");
    close(fd);
    return;
  }
  */
  // DACR 값이 WASDOM_DACR_MANAGER (3)이 아닐 경우에만, MANAGER(3)으로 DACR을 적용
  //if (dacr_val != WASDOM_DACR_MANAGER) {
  // DACR 값을 MANAGER로 세팅
  int32_t dacr_val = WASDOM_DACR_MANAGER;
  
  // DACR 값을 적용합니다.
  if (ioctl(fd, WASDOM_CMD_WR_DACR, &dacr_val) < 0) {
    perror("Error setting DACR in wasdom_driver");
    close(fd);
    return;
  }

  //}
  close(fd);
}

// static
void RwxMemoryWriteScope::SetExecutable() {
    // 메모리 실행 가능 설정
  int fd = open("/dev/wasdom_device", O_RDWR);
  /*
  if (fd < 0) {
    perror("Error opening wasdom_device");
    return;
  }
  // 현재 DACR 값을 가져옴
  int32_t dacr_val;
  if (ioctl(fd, WASDOM_CMD_RD_DACR, &dacr_val) < 0) {
    perror("Error reading DACR in wasdom_driver");
    close(fd);
    return;
  }
  */
  // DACR 값이 WASDOM_DACR_CLIENT (1)이 아닐 경우에만, CLIENT(1)으로 DACR을 적용
  //if (dacr_val == WASDOM_DACR_MANAGER) {
  // DACR 값을 Client로 세팅 
  int32_t dacr_val = WASDOM_DACR_MANAGER;
  
  // DACR 값을 적용합니다.
  if (ioctl(fd, WASDOM_CMD_WR_DACR, &dacr_val) < 0) {
    perror("Error setting DACR in wasdom_driver");
    close(fd);
    return;
  }
  //}
  close(fd);
}

#endif  // V8_HAS_PTHREAD_JIT_WRITE_PROTECT

}  // namespace internal
}  // namespace v8

#endif  // V8_COMMON_CODE_MEMORY_ACCESS_INL_H_
