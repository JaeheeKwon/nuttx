/****************************************************************************
 * arch/sim/src/sim/sim_heap.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <string.h>
#include <stdbool.h>

#include <nuttx/arch.h>
#include <nuttx/atomic.h>
#include <nuttx/fs/procfs.h>
#include <nuttx/mm/mm.h>
#include <nuttx/sched_note.h>

#include "sim_internal.h"

#ifdef CONFIG_MM_CUSTOMIZE_MANAGER

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This describes one heap (possibly with multiple regions) */

struct mm_delaynode_s
{
  struct mm_delaynode_s *flink;
};

struct mm_heap_s
{
  struct mm_delaynode_s *mm_delaylist[CONFIG_SMP_NCPUS];

#if CONFIG_MM_FREE_DELAYCOUNT_MAX > 0
  size_t mm_delaycount[CONFIG_SMP_NCPUS];
#endif

  atomic_t aordblks;
  atomic_t uordblks;
  atomic_t usmblks;

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMINFO)
  struct procfs_meminfo_entry_s mm_procfs;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void mm_delayfree(struct mm_heap_s *heap, void *mem, bool delay);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void mm_add_delaylist(struct mm_heap_s *heap, void *mem)
{
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  struct mm_delaynode_s *tmp = mem;
  irqstate_t flags;

  /* Delay the deallocation until a more appropriate time. */

  flags = up_irq_save();

  tmp->flink = heap->mm_delaylist[this_cpu()];
  heap->mm_delaylist[this_cpu()] = tmp;

#if CONFIG_MM_FREE_DELAYCOUNT_MAX > 0
  heap->mm_delaycount[this_cpu()]++;
#endif

  up_irq_restore(flags);
#endif
}

static bool free_delaylist(struct mm_heap_s *heap, bool force)
{
  bool ret = false;
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  struct mm_delaynode_s *tmp;
  irqstate_t flags;

  /* Move the delay list to local */

  flags = up_irq_save();

  tmp = heap->mm_delaylist[this_cpu()];

#if CONFIG_MM_FREE_DELAYCOUNT_MAX > 0
  if (tmp == NULL ||
      (!force &&
        heap->mm_delaycount[this_cpu()] < CONFIG_MM_FREE_DELAYCOUNT_MAX))
    {
      up_irq_restore(flags);
      return false;
    }

  heap->mm_delaycount[this_cpu()] = 0;
#endif
  heap->mm_delaylist[this_cpu()] = NULL;

  up_irq_restore(flags);

  /* Test if the delayed is empty */

  ret = tmp != NULL;

  while (tmp)
    {
      void *address;

      /* Get the first delayed deallocation */

      address = tmp;
      tmp = tmp->flink;

      /* The address should always be non-NULL since that was checked in the
       * 'while' condition above.
       */

      mm_delayfree(heap, address, false);
    }

#endif
  return ret;
}

/****************************************************************************
 * Name: mm_delayfree
 *
 * Description:
 *   Delay free memory if `delay` is true, otherwise free it immediately.
 *
 ****************************************************************************/

static void mm_delayfree(struct mm_heap_s *heap, void *mem, bool delay)
{
#if defined(CONFIG_BUILD_FLAT) || defined(__KERNEL__)
  /* Check current environment */

  if (up_interrupt_context())
    {
      /* We are in ISR, add to the delay list */

      mm_add_delaylist(heap, mem);
    }
  else
#endif

  if (nxsched_gettid() < 0 || delay)
    {
      /* nxsched_gettid() return -ESRCH, means we are in situations
       * during context switching(See nxsched_gettid's comment).
       * Then add to the delay list.
       */

      mm_add_delaylist(heap, mem);
    }
  else
    {
      int size = host_mallocsize(mem);
      atomic_fetch_sub(&heap->aordblks, 1);
      atomic_fetch_sub(&heap->uordblks, size);
      sched_note_heap(NOTE_HEAP_FREE, heap, mem, size, 0);
      host_free(mem);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mm_initialize
 *
 * Description:
 *   Initialize the selected heap data structures, providing the initial
 *   heap region.
 *
 * Input Parameters:
 *   heap      - The selected heap
 *   heapstart - Start of the initial heap region
 *   heapsize  - Size of the initial heap region
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

struct mm_heap_s *mm_initialize(const char *name,
                                void *heap_start, size_t heap_size)
{
  struct mm_heap_s *heap;

  heap = host_memalign(sizeof(void *), sizeof(*heap));
  DEBUGASSERT(heap);

  memset(heap, 0, sizeof(struct mm_heap_s));

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMINFO)
  heap->mm_procfs.name = name;
  heap->mm_procfs.heap = heap;
  procfs_register_meminfo(&heap->mm_procfs);
#endif

  sched_note_heap(NOTE_HEAP_ADD, heap, heap_start, heap_size, 0);
  return heap;
}

/****************************************************************************
 * Name: mm_uninitialize
 *
 * Description:
 *   Uninitialize the selected heap data structures
 *
 * Input Parameters:
 *   heap      - The selected heap
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void mm_uninitialize(struct mm_heap_s *heap)
{
  sched_note_heap(NOTE_HEAP_REMOVE, heap, NULL, 0, 0);

#if defined(CONFIG_FS_PROCFS) && !defined(CONFIG_FS_PROCFS_EXCLUDE_MEMINFO)
  procfs_unregister_meminfo(&heap->mm_procfs);
#endif
  mm_free_delaylist(heap);
  host_free(heap);
}

/****************************************************************************
 * Name: mm_addregion
 *
 * Description:
 *   This function adds a region of contiguous memory to the selected heap.
 *
 * Input Parameters:
 *   heap      - The selected heap
 *   heapstart - Start of the heap region
 *   heapsize  - Size of the heap region
 *
 * Returned Value:
 *   None
 *
 * Assumptions:
 *
 ****************************************************************************/

void mm_addregion(struct mm_heap_s *heap, void *heapstart,
                  size_t heapsize)
{
}

/****************************************************************************
 * Name: mm_malloc
 *
 * Description:
 *  Find the smallest chunk that satisfies the request. Take the memory from
 *  that chunk, save the remaining, smaller chunk (if any).
 *
 *  8-byte alignment of the allocated data is assured.
 *
 ****************************************************************************/

void *mm_malloc(struct mm_heap_s *heap, size_t size)
{
  return mm_realloc(heap, NULL, size);
}

/****************************************************************************
 * Name: mm_free
 *
 * Description:
 *   Returns a chunk of memory to the list of free nodes,  merging with
 *   adjacent free chunks if possible.
 *
 ****************************************************************************/

void mm_free(struct mm_heap_s *heap, void *mem)
{
  minfo("Freeing %p\n", mem);

  /* Protect against attempts to free a NULL reference */

  if (mem == NULL)
    {
      return;
    }

  mm_delayfree(heap, mem, CONFIG_MM_FREE_DELAYCOUNT_MAX > 0);
}

/****************************************************************************
 * Name: mm_free_delaylist
 *
 * Description:
 *   force freeing the delaylist of this heap.
 *
 ****************************************************************************/

void mm_free_delaylist(struct mm_heap_s *heap)
{
  if (heap)
    {
       free_delaylist(heap, true);
    }
}

/****************************************************************************
 * Name: mm_realloc
 *
 * Description:
 *   If the reallocation is for less space, then:
 *
 *     (1) the current allocation is reduced in size
 *     (2) the remainder at the end of the allocation is returned to the
 *         free list.
 *
 *  If the request is for more space and the current allocation can be
 *  extended, it will be extended by:
 *
 *     (1) Taking the additional space from the following free chunk, or
 *     (2) Taking the additional space from the preceding free chunk.
 *     (3) Or both
 *
 *  If the request is for more space but the current chunk cannot be
 *  extended, then malloc a new buffer, copy the data into the new buffer,
 *  and free the old buffer.
 *
 ****************************************************************************/

void *mm_realloc(struct mm_heap_s *heap, void *oldmem,
                 size_t size)
{
  void *mem;
  int uordblks;
  int usmblks;
  int newsize;
  int oldsize;

  free_delaylist(heap, false);

  if (size == 0)
    {
      size = 1;
    }

  oldsize = host_mallocsize(oldmem);
  atomic_fetch_sub(&heap->uordblks, oldsize);
  mem = host_realloc(oldmem, size);

  atomic_fetch_add(&heap->aordblks, oldmem == NULL && mem != NULL);
  newsize = host_mallocsize(mem ? mem : oldmem);
  atomic_fetch_add(&heap->uordblks, newsize);
  usmblks = atomic_read(&heap->usmblks);
  if (mem != NULL)
    {
      if (oldmem != NULL)
        {
          sched_note_heap(NOTE_HEAP_FREE, heap, oldmem, oldsize, 0);
        }

      sched_note_heap(NOTE_HEAP_ALLOC, heap, mem, newsize, 0);
    }

  do
    {
      uordblks = atomic_read(&heap->uordblks);
      if (uordblks <= usmblks)
        {
          break;
        }
    }
  while (atomic_try_cmpxchg(&heap->usmblks, &usmblks, uordblks));

#if CONFIG_MM_FREE_DELAYCOUNT_MAX > 0
  if (mem == NULL && free_delaylist(heap, true))
    {
      return mm_realloc(heap, oldmem, size);
    }
#endif

  return mem;
}

/****************************************************************************
 * Name: mm_calloc
 *
 * Descriptor:
 *   mm_calloc() calculates the size of the allocation and calls mm_zalloc()
 *
 ****************************************************************************/

void *mm_calloc(struct mm_heap_s *heap, size_t n, size_t elem_size)
{
  size_t size = n * elem_size;

  if (size < elem_size)
    {
      return NULL;
    }

  return mm_zalloc(heap, size);
}

/****************************************************************************
 * Name: mm_zalloc
 *
 * Description:
 *   mm_zalloc calls mm_malloc, then zeroes out the allocated chunk.
 *
 ****************************************************************************/

void *mm_zalloc(struct mm_heap_s *heap, size_t size)
{
  void *ptr;

  ptr = mm_malloc(heap, size);
  if (ptr != NULL)
    {
      memset(ptr, 0, size);
    }

  return ptr;
}

/****************************************************************************
 * Name: mm_memalign
 *
 * Description:
 *   memalign requests more than enough space from malloc, finds a region
 *   within that chunk that meets the alignment request and then frees any
 *   leading or trailing space.
 *
 *   The alignment argument must be a power of two (not checked).  8-byte
 *   alignment is guaranteed by normal malloc calls.
 *
 ****************************************************************************/

void *mm_memalign(struct mm_heap_s *heap, size_t alignment, size_t size)
{
  void *mem;
  int uordblks;
  int usmblks;

  free_delaylist(heap, false);
  mem = host_memalign(alignment, size);

  if (mem == NULL)
    {
      return NULL;
    }

  size = host_mallocsize(mem);
  sched_note_heap(NOTE_HEAP_ALLOC, heap, mem, size, 0);
  atomic_fetch_add(&heap->aordblks, 1);
  atomic_fetch_add(&heap->uordblks, size);
  usmblks = atomic_read(&heap->usmblks);

  do
    {
      uordblks = atomic_read(&heap->uordblks);
      if (uordblks <= usmblks)
        {
          break;
        }
    }
  while (atomic_try_cmpxchg(&heap->usmblks, &usmblks, uordblks));

#if CONFIG_MM_FREE_DELAYCOUNT_MAX > 0
  if (mem == NULL && free_delaylist(heap, true))
    {
      return mm_memalign(heap, alignment, size);
    }
#endif

  return mem;
}

/****************************************************************************
 * Name: mm_heapmember
 *
 * Description:
 *   Check if an address lies in the heap.
 *
 * Parameters:
 *   heap - The heap to check
 *   mem  - The address to check
 *
 * Return Value:
 *   true if the address is a member of the heap.  false if not
 *   not.  If the address is not a member of the heap, then it
 *   must be a member of the user-space heap (unchecked)
 *
 ****************************************************************************/

bool mm_heapmember(struct mm_heap_s *heap, void *mem)
{
  return true;
}

/****************************************************************************
 * Name: mm_brkaddr
 *
 * Description:
 *   Return the break address of a heap region.  Zero is returned if the
 *   memory region is not initialized.
 *
 ****************************************************************************/

void *mm_brkaddr(struct mm_heap_s *heap, int region)
{
  return NULL;
}

/****************************************************************************
 * Name: mm_extend
 *
 * Description:
 *   Extend a heap region by add a block of (virtually) contiguous memory
 *   to the end of the heap.
 *
 ****************************************************************************/

void mm_extend(struct mm_heap_s *heap, void *mem, size_t size,
               int region)
{
}

/****************************************************************************
 * Name: mm_mallinfo
 *
 * Description:
 *   mallinfo returns a copy of updated current heap information.
 *
 ****************************************************************************/

struct mallinfo mm_mallinfo(struct mm_heap_s *heap)
{
  struct mallinfo info;

  memset(&info, 0, sizeof(struct mallinfo));
  info.aordblks = atomic_read(&heap->aordblks);
  info.uordblks = atomic_read(&heap->uordblks);
  info.usmblks  = atomic_read(&heap->usmblks);
  return info;
}

/****************************************************************************
 * Name: mm_mallinfo_task
 *
 * Description:
 *   mallinfo_task returns a copy of updated current task's heap information.
 *
 ****************************************************************************/

struct mallinfo_task mm_mallinfo_task(struct mm_heap_s *heap,
                                      const struct malltask *task)
{
  struct mallinfo_task info =
    {
      0, 0
    };

  return info;
}

/****************************************************************************
 * Name: mm_memdump
 *
 * Description:
 *   mm_memdump returns a memory info about specified pid of task/thread.
 *
 ****************************************************************************/

void mm_memdump(struct mm_heap_s *heap, const struct mm_memdump_s *dump)
{
}

#ifdef CONFIG_DEBUG_MM

/****************************************************************************
 * Name: mm_checkcorruption
 *
 * Description:
 *   mm_checkcorruption is used to check whether memory heap is normal.
 *
 ****************************************************************************/

void mm_checkcorruption(struct mm_heap_s *heap)
{
}

#endif /* CONFIG_DEBUG_MM */

/****************************************************************************
 * Name: malloc_size
 ****************************************************************************/

size_t mm_malloc_size(struct mm_heap_s *heap, void *mem)
{
  return host_mallocsize(mem);
}

/****************************************************************************
 * Name: up_allocate_heap
 *
 * Description:
 *   This function will be called to dynamically set aside the heap region.
 *
 *   If a protected kernel-space heap is provided, the kernel heap must be
 *   allocated (and protected) by an analogous up_allocate_kheap().
 *
 ****************************************************************************/

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  *heap_start = NULL;
  *heap_size  = 0;
}

/****************************************************************************
 * Name: mm_heapfree
 *
 * Description:
 *   Return the total free size (in bytes) in the heap
 *
 ****************************************************************************/

size_t mm_heapfree(struct mm_heap_s *heap)
{
  return SIZE_MAX;
}

/****************************************************************************
 * Name: mm_heapfree_largest
 *
 * Description:
 *   Return the largest chunk of contiguous memory in the heap
 *
 ****************************************************************************/

size_t mm_heapfree_largest(struct mm_heap_s *heap)
{
  return SIZE_MAX;
}

#else /* CONFIG_MM_CUSTOMIZE_MANAGER */

void up_allocate_heap(void **heap_start, size_t *heap_size)
{
  *heap_start = host_allocheap(SIM_HEAP_SIZE, false);
  *heap_size  = SIM_HEAP_SIZE;
}

#endif /* CONFIG_MM_CUSTOMIZE_MANAGER */
