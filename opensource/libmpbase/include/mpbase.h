/* SPDX-License-Identifier: MIT */
/* Copyright (c) 2026 The sweet device contributors */

#ifndef SWEET_LIBMPBASE_H_
#define SWEET_LIBMPBASE_H_

#include <stddef.h>
#include <stdint.h>

#if defined(__GNUC__)
#define MPBASE_EXPORT __attribute__((visibility("default")))
#else
#define MPBASE_EXPORT
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint64_t major;
    uint64_t minor;
    uint64_t patch;
    uint64_t build;
    const char *version;
    const char *build_date;
    const char *copyright;
} MpbaseVersionInfo;

MPBASE_EXPORT void *MMemSet(void *destination, int value, size_t size);
MPBASE_EXPORT void *MMemCpy(void *destination, const void *source, size_t size);
MPBASE_EXPORT void *MMemMove(void *destination, const void *source, size_t size);
MPBASE_EXPORT int MMemCmp(const void *left, const void *right, size_t size);

MPBASE_EXPORT void *MMemMgrCreate(void *memory, size_t size);
MPBASE_EXPORT void MMemMgrDestroy(void *manager);

MPBASE_EXPORT void *MMemAllocStatic(void *manager, uint32_t size);
MPBASE_EXPORT void *MMemAlloc(void *manager, size_t size);
MPBASE_EXPORT void MMemFreeStatic(void *manager, void *memory);
MPBASE_EXPORT void MMemFree(void *manager, void *memory);
MPBASE_EXPORT void *MMemReallocStatic(void *manager, void *memory, uint32_t size);
MPBASE_EXPORT void *MMemRealloc(void *manager, void *memory, size_t size);
MPBASE_EXPORT size_t GetMaxAllocMemSize(void *manager);

MPBASE_EXPORT MpbaseVersionInfo *Mpbase_GetVersion(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // SWEET_LIBMPBASE_H_
