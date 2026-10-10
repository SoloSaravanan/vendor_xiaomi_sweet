// SPDX-License-Identifier: MIT
// Copyright (c) 2026 The sweet device contributors

#include "mpbase.h"

#include <stdlib.h>
#include <string.h>

#define MPBASE_ALIGNMENT 8U
#define MPBASE_SPLIT_MIN_REMAINDER 31U
#define MPBASE_REALLOC_SHRINK_MIN_REMAINDER 32U
#define MPBASE_SENTINEL_SIZE UINT64_MAX

typedef struct {
    uint64_t size;
    uint64_t used;
} MpbaseBlock;

_Static_assert(sizeof(MpbaseBlock) == 16, "mpbase block ABI changed");
_Static_assert(sizeof(MpbaseVersionInfo) == 56, "mpbase version ABI changed");

static const char kVersion[] = "ArcSoft_Mpbase_0.1.0.4";
static const char kModuleName[] = "ArcSoft_Mpbase";
static const char kBuildDate[] = "Apr 24 2019";
static const char kCopyright[] = "Copyright 2014-2015, ArcSoft Inc.";

/* Match the binary's pre-initialization contents; Mpbase_GetVersion fills the
 * version fields and replaces the module-name pointer on each call.
 */
static MpbaseVersionInfo gVersionInfo = {
    .major = 0,
    .minor = 0,
    .patch = 0,
    .build = 0,
    .version = kModuleName,
    .build_date = kBuildDate,
    .copyright = kCopyright,
};

static MpbaseBlock *next_block(MpbaseBlock *block) {
    return (MpbaseBlock *)((uint8_t *)block + sizeof(*block) + (size_t)block->size);
}

static void *block_data(MpbaseBlock *block) {
    return (uint8_t *)block + sizeof(*block);
}

static uint32_t align_request(uint32_t size) {
    /* Keep the original 32-bit wraparound behavior. */
    return (size + UINT32_C(7)) & ~UINT32_C(7);
}

static MpbaseBlock *find_block(void *manager, const void *memory,
                               MpbaseBlock **previous_out) {
    MpbaseBlock *previous = NULL;
    MpbaseBlock *block = (MpbaseBlock *)manager;

    if (previous_out != NULL) {
        *previous_out = NULL;
    }
    if (block == NULL || memory == NULL) {
        return NULL;
    }

    while (block->size != MPBASE_SENTINEL_SIZE) {
        if (block_data(block) == memory) {
            if (previous_out != NULL) {
                *previous_out = previous;
            }
            return block;
        }
        previous = block;
        block = next_block(block);
    }

    return NULL;
}

static void merge_with_next(MpbaseBlock *block) {
    MpbaseBlock *next = next_block(block);

    if (next->size != MPBASE_SENTINEL_SIZE && next->used == 0) {
        block->size += sizeof(*next) + next->size;
    }
}

void *MMemSet(void *destination, int value, size_t size) {
    return memset(destination, (unsigned char)value, size);
}

void *MMemCpy(void *destination, const void *source, size_t size) {
    return memcpy(destination, source, size);
}

void *MMemMove(void *destination, const void *source, size_t size) {
    return memmove(destination, source, size);
}

int MMemCmp(const void *left, const void *right, size_t size) {
    return memcmp(left, right, size);
}

void *MMemMgrCreate(void *memory, size_t size) {
    uintptr_t raw_start;
    uintptr_t aligned_start;
    uintptr_t end;
    size_t arena_size;
    MpbaseBlock *first;
    MpbaseBlock *sentinel;

    if (memory == NULL) {
        return NULL;
    }

    raw_start = (uintptr_t)memory;
    if (size > (size_t)(UINTPTR_MAX - raw_start) ||
        raw_start > UINTPTR_MAX - (MPBASE_ALIGNMENT - 1U)) {
        return NULL;
    }

    aligned_start = (raw_start + (MPBASE_ALIGNMENT - 1U)) &
                    ~((uintptr_t)MPBASE_ALIGNMENT - 1U);
    end = raw_start + size;
    if (end < aligned_start) {
        return NULL;
    }

    arena_size = (size_t)((end - aligned_start) &
                          ~((uintptr_t)MPBASE_ALIGNMENT - 1U));
    if (arena_size <= 2U * sizeof(MpbaseBlock)) {
        return NULL;
    }

    first = (MpbaseBlock *)aligned_start;
    first->size = arena_size - 2U * sizeof(MpbaseBlock);
    first->used = 0;

    sentinel = (MpbaseBlock *)((uint8_t *)aligned_start + arena_size -
                               sizeof(MpbaseBlock));
    sentinel->size = MPBASE_SENTINEL_SIZE;
    sentinel->used = 0;
    return first;
}

void MMemMgrDestroy(void *manager) {
    (void)manager;
}

void *MMemAllocStatic(void *manager, uint32_t size) {
    MpbaseBlock *block = (MpbaseBlock *)manager;
    uint32_t aligned_size;

    if (block == NULL || size == 0) {
        return NULL;
    }

    aligned_size = align_request(size);
    while (block->size != MPBASE_SENTINEL_SIZE) {
        if (block->used == 0 && (uint64_t)aligned_size <= block->size) {
            uint64_t available_after_request = block->size - aligned_size;

            if (available_after_request >
                sizeof(*block) + MPBASE_SPLIT_MIN_REMAINDER) {
                uint64_t remainder = available_after_request - sizeof(*block);
                MpbaseBlock *split = (MpbaseBlock *)((uint8_t *)block_data(block) +
                                                     aligned_size);
                split->size = remainder;
                split->used = 0;
                block->size = aligned_size;
            }
            block->used = 1;
            return block_data(block);
        }
        block = next_block(block);
    }

    return NULL;
}

void *MMemAlloc(void *manager, size_t size) {
    if (manager != NULL) {
        return MMemAllocStatic(manager, (uint32_t)size);
    }
    return malloc(size);
}

void MMemFreeStatic(void *manager, void *memory) {
    MpbaseBlock *previous;
    MpbaseBlock *block = find_block(manager, memory, &previous);

    if (block == NULL) {
        return;
    }

    block->used = 0;
    merge_with_next(block);
    if (previous != NULL && previous->used == 0) {
        previous->size += sizeof(*block) + block->size;
    }
}

void MMemFree(void *manager, void *memory) {
    if (manager == NULL) {
        free(memory);
        return;
    }
    MMemFreeStatic(manager, memory);
}

void *MMemReallocStatic(void *manager, void *memory, uint32_t size) {
    MpbaseBlock *block;
    uint32_t aligned_size;

    if (manager == NULL || size == 0) {
        return NULL;
    }
    if (memory == NULL) {
        return MMemAllocStatic(manager, size);
    }

    block = find_block(manager, memory, NULL);
    if (block == NULL) {
        return NULL;
    }

    aligned_size = align_request(size);
    if (block->size >= aligned_size) {
        uint64_t available_after_request = block->size - aligned_size;

        if (available_after_request >
            sizeof(*block) + MPBASE_REALLOC_SHRINK_MIN_REMAINDER) {
            uint64_t remainder = available_after_request - sizeof(*block);
            MpbaseBlock *split = (MpbaseBlock *)((uint8_t *)block_data(block) +
                                                 aligned_size);
            split->size = remainder;
            split->used = 0;
            block->size = aligned_size;
            merge_with_next(split);
        }
        return memory;
    }

    {
        MpbaseBlock *next = next_block(block);

        /* The original implementation's fit check excludes the 16-byte
         * header, then adds that header when merging the blocks.
         */
        if (next->size != MPBASE_SENTINEL_SIZE && next->used == 0 &&
            (uint64_t)aligned_size <= block->size + next->size) {
            uint64_t combined_size = block->size + sizeof(*next) + next->size;
            uint64_t remainder = combined_size - aligned_size - sizeof(*block);

            block->size = combined_size;
            if (remainder > MPBASE_SPLIT_MIN_REMAINDER) {
                MpbaseBlock *split = (MpbaseBlock *)((uint8_t *)block_data(block) +
                                                     aligned_size);
                split->size = remainder;
                split->used = 0;
                block->size = aligned_size;
            }
            return memory;
        }
    }

    {
        void *replacement = MMemAllocStatic(manager, aligned_size);

        if (replacement == NULL) {
            return NULL;
        }
        MMemCpy(replacement, memory, (size_t)block->size);
        MMemFreeStatic(manager, memory);
        return replacement;
    }
}

void *MMemRealloc(void *manager, void *memory, size_t size) {
    if (manager != NULL) {
        return MMemReallocStatic(manager, memory, (uint32_t)size);
    }
    return realloc(memory, size);
}

size_t GetMaxAllocMemSize(void *manager) {
    MpbaseBlock *block = (MpbaseBlock *)manager;
    uint64_t maximum = 0;

    if (block == NULL) {
        return 2;
    }

    while (block->size != MPBASE_SENTINEL_SIZE) {
        if (block->used == 0 && block->size > maximum) {
            maximum = block->size;
        }
        block = next_block(block);
    }

    return (size_t)maximum;
}

MpbaseVersionInfo *Mpbase_GetVersion(void) {
    gVersionInfo.major = 0;
    gVersionInfo.minor = 1;
    gVersionInfo.patch = 0;
    gVersionInfo.build = 4;
    gVersionInfo.version = kVersion;
    return &gVersionInfo;
}
