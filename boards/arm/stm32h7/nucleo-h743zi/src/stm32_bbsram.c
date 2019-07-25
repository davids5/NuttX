/****************************************************************************
 * configs/nucleo-h743zi/src/stm32_bbsram.c
 *
 *   Copyright (C) 2016, 2018-2019 Gregory Nutt. All rights reserved.
 *   Author: David Sidrane <david.sidrane@nscdg.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/ioctl.h>

#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <syslog.h>
#include <unistd.h>

#include <nuttx/fs/fs.h>
#include <nuttx/irq.h>
#include <nuttx/sched.h>

#include "up_internal.h"
#include "up_arch.h"
#include "stm32_bbsram.h"
#include "nvic.h"

#include "nucleo-h743zi.h"

#ifdef CONFIG_STM32H7_BBSRAM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#define FREEZE_STR(s)          #s
#define STRINGIFY(s)           FREEZE_STR(s)
#define HARDFAULT_FILENO       3
#define HARDFAULT_PATH         BBSRAM_PATH""STRINGIFY(HARDFAULT_FILENO)
#define HARDFAULT_REBOOT_      FILENO 0
#define HARDFAULT_REBOOT_PATH  BBSRAM_PATH""STRINGIFY(HARDFAULT_REBOOT_FILENO)

#define BBSRAM_SIZE_FN0        (sizeof(int))
#define BBSRAM_SIZE_FN1        384
#define BBSRAM_SIZE_FN2        384
#define BBSRAM_SIZE_FN3 -      1

/* The following guides in the amount of the user and interrupt stack
 * data we can save. The amount of storage left will dictate the actual
 * number of entries of the user stack data saved. If it is too big
 * It will be truncated by the call to stm32_bbsram_savepanic
 */
#define BBSRAM_HEADER_SIZE     20 /* This is an assumption */
#define BBSRAM_USED            ((4*BBSRAM_HEADER_SIZE)+ \
                                (BBSRAM_SIZE_FN0+BBSRAM_SIZE_FN1+ \
                                 BBSRAM_SIZE_FN2))
#define BBSRAM_REAMINING       (STM32H7_BBSRAM_SIZE-BBSRAM_USED)
#if CONFIG_ARCH_INTERRUPTSTACK <= 3
#  define BBSRAM_NUMBER_STACKS 1
#else
#  define BBSRAM_NUMBER_STACKS 2
#endif
#define BBSRAM_FIXED_ELEMENTS_SIZE (sizeof(struct info_t))
#define BBSRAM_LEFTOVER            (BBSRAM_REAMINING-\
                                    BBSRAM_FIXED_ELEMENTS_SIZE)

#define CONFIG_ISTACK_SIZE (BBSRAM_LEFTOVER/BBSRAM_NUMBER_STACKS/ \
                            sizeof(stack_word_t))
#define CONFIG_USTACK_SIZE (BBSRAM_LEFTOVER/BBSRAM_NUMBER_STACKS/ \
                            sizeof(stack_word_t))

/* The path to the Battery Backed up SRAM */

#define BBSRAM_PATH "/fs/bbr"

/* The sizes of the files to create (-1) use rest of BBSRAM memory */

#define BSRAM_FILE_SIZES \
{ \
  BBSRAM_SIZE_FN0, \
  BBSRAM_SIZE_FN1, \
  BBSRAM_SIZE_FN2, \
  BBSRAM_SIZE_FN3, \
  0 \
}

#define ARRAYSIZE(a) (sizeof((a))/sizeof(a[0]))

/* For Assert keep this much of the file name*/

#define MAX_FILE_PATH_LENGTH 40

#define HEADER_TIME_FMT      "%Y-%m-%d-%H:%M:%S"
#define HEADER_TIME_FMT_NUM  (2+ 0+ 0+ 0+ 0+ 0)
#define HEADER_TIME_FMT_LEN  (((ARRAYSIZE(HEADER_TIME_FMT)-1) + \
                                HEADER_TIME_FMT_NUM))

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Used for stack frame storage */

typedef uint32_t stack_word_t;

/* Stack related data */

struct stack_t
{
  uint32_t sp;
  uint32_t top;
  uint32_t size;
};

struct stacks_t
{
   struct stack_t user;
#if CONFIG_ARCH_INTERRUPTSTACK > 3
   struct stack_t interrupt;
#endif
};

/* Flags to identify what is in the dump */

enum fault_flags_t
{
  REGS_PRESENT          = 0x01,
  USERSTACK_PRESENT     = 0x02,
  INTSTACK_PRESENT      = 0x04,
  INVALID_USERSTACK_PTR = 0x20,
  INVALID_INTSTACK_PTR  = 0x40,
};

struct info_t
{
  enum fault_flags_t flags;                           /* What is in the dump */
  uintptr_t          current_regs;                    /* Used to validate the dump */
  int                lineno;                          /* __LINE__ to up_assert */
  int                pid;                             /* Process ID */
  uint32_t           regs[XCPTCONTEXT_REGS];          /* Interrupt register save area */
  struct stacks_t    stacks;                          /* Stack info */
#if CONFIG_TASK_NAME_SIZE > 0
  char               name[CONFIG_TASK_NAME_SIZE + 1]; /* Task name (with NULL terminator) */
#endif
  char               filename[MAX_FILE_PATH_LENGTH];  /* the Last of chars in __FILE__
                                                       *  to up_assert */
};

struct fullcontext_t
{
  struct info_t      info;              /* The info */
#if CONFIG_ARCH_INTERRUPTSTACK > 3 /* The amount of stack data is compile time
                                    * sized backed on what is left after the
                                    * other BBSRAM files are defined
                                    * The order is such that only the
                                    * ustack should be truncated
                                    */
  stack_word_t       istack[CONFIG_USTACK_SIZE];
#endif
  stack_word_t       ustack[CONFIG_ISTACK_SIZE];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint8_t g_sdata[STM32H7_BBSRAM_SIZE];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: hardfault_get_desc
 ****************************************************************************/

static int hardfault_get_desc(struct bbsramd_s *desc)
{
  FAR struct file filestruct;
  int ret;

  ret = file_open(&filestruct, HARDFAULT_PATH, O_RDONLY);
  if (ret < 0)
    {
      syslog(LOG_INFO, "stm32 bbsram: Failed to open Fault Log file [%s] "
             "(%d)\n", HARDFAULT_PATH, ret);
    }
  else
    {
      ret = file_ioctl(&filestruct, STM32H7_BBSRAM_GETDESC_IOCTL,
                       (unsigned long)((uintptr_t)desc));
      (void)file_close(&filestruct);

      if (ret < 0)
        {
          syslog(LOG_INFO, "stm32 bbsram: Failed to get Fault Log descriptor "
              "(%d)\n", ret);
        }
    }

  return ret;
}

/****************************************************************************
 * Name: copy_reverse
 ****************************************************************************/

#if defined(CONFIG_STM32H7_SAVE_CRASHDUMP)
static void copy_reverse(stack_word_t *dest, stack_word_t *src, int size)
{
  while (size--)
    {
      *dest++ = *src--;
    }
}
#endif /* CONFIG_STM32H7_SAVE_CRASHDUMP */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bbsram_int
 ****************************************************************************/

int stm32_bbsram_int(void)
{
  int filesizes[CONFIG_STM32H7_BBSRAM_FILES + 1] = BSRAM_FILE_SIZES;
  char buf[HEADER_TIME_FMT_LEN + 1];
  struct bbsramd_s desc;
  int rv;
  int state;
  struct tm tt;
  time_t time_sec;

#if defined(CONFIG_EXAMPLES_HARDFAULTLOG)
  uint32_t regval;

  /* Enable divide by 0 fault generation */

  regval = getreg32(NVIC_CFGCON);
  regval |= NVIC_CFGCON_DIV0TRP;
  putreg32(regval, NVIC_CFGCON);
#endif

  /* Using Battery Backed Up SRAM */

  stm32_bbsraminitialize(BBSRAM_PATH, filesizes);

#if defined(CONFIG_STM32H7_SAVE_CRASHDUMP)
  /* Panic Logging in Battery Backed Up Files */
  /* Do we have an hard fault in BBSRAM? */

  rv = hardfault_get_desc(&desc);
  if (rv >= OK)
    {
      state = (desc.lastwrite.tv_sec || desc.lastwrite.tv_nsec) ?  OK : 1;
      if (state == OK)
        {
          syslog(LOG_EMERG, "There is a hard fault logged.\n");
        }

      syslog(LOG_INFO, "Fault Log info File No %d Length %d flags:0x%02x "
          "state:%d\n",(unsigned int)desc.fileno, (unsigned int) desc.len,
          (unsigned int)desc.flags, state);

      if (state == OK)
        {
          time_sec = desc.lastwrite.tv_sec + (desc.lastwrite.tv_nsec / 1e9);
          gmtime_r(&time_sec, &tt);
          strftime(buf, HEADER_TIME_FMT_LEN , HEADER_TIME_FMT , &tt);

          syslog(LOG_INFO, "Fault Logged on %s - Valid\n", buf);
#  if defined(CONFIG_EXAMPLES_HARDFAULTLOG)
          syslog(LOG_INFO, "Use hardfaultlog command to investigate\n", buf);
#  endif
        }
    }
#endif /* CONFIG_STM32H7_SAVE_CRASHDUMP */

  return rv;
}

/****************************************************************************
 * Name: board_crashdump
 ****************************************************************************/

#if defined(CONFIG_STM32H7_SAVE_CRASHDUMP)
void board_crashdump(uintptr_t currentsp, FAR void *tcb,
                     FAR const uint8_t *filename, int lineno)
{
  struct fullcontext_t *pdump = (struct fullcontext_t *)&g_sdata;
  FAR struct tcb_s *rtcb;
  int rv;

  (void)enter_critical_section();

  rtcb = (FAR struct tcb_s *)tcb;

  /* Zero out everything */

  memset(pdump, 0, sizeof(struct fullcontext_t));

  /* Save Info */

  pdump->info.lineno = lineno;

  if (filename)
    {
      int offset = 0;
      unsigned int len = strlen((char *)filename) + 1;

      if (len > sizeof(pdump->info.filename))
        {
          offset = len - sizeof(pdump->info.filename);
        }

      strncpy(pdump->info.filename, (char *)&filename[offset],
              sizeof(pdump->info.filename));
    }

  /* Save the value of the pointer for current_regs as debugging info.
   * It should be NULL in case of an ASSERT and will aid in cross
   * checking the validity of system memory at the time of the
   * fault.
   */

  pdump->info.current_regs = (uintptr_t) CURRENT_REGS;

  /* Save Context */

#if CONFIG_TASK_NAME_SIZE > 0
  strncpy(pdump->info.name, rtcb->name, CONFIG_TASK_NAME_SIZE);
#endif

  pdump->info.pid = rtcb->pid;

  /* If  current_regs is not NULL then we are in an interrupt context
   * and the user context is in current_regs else we are running in
   * the users context
   */

  if (CURRENT_REGS)
    {
#if CONFIG_ARCH_INTERRUPTSTACK > 3
      pdump->info.stacks.interrupt.sp = currentsp;
#endif
      pdump->info.flags |= (REGS_PRESENT | USERSTACK_PRESENT | \
                            INTSTACK_PRESENT);
      memcpy(pdump->info.regs, (void *)CURRENT_REGS,
             sizeof(pdump->info.regs));
      pdump->info.stacks.user.sp = pdump->info.regs[REG_R13];
    }
  else
    {
      /* users context */

      pdump->info.flags |= USERSTACK_PRESENT;
      pdump->info.stacks.user.sp = currentsp;
    }

  if (pdump->info.pid == 0)
    {
      pdump->info.stacks.user.top = g_idle_topstack - 4;
      pdump->info.stacks.user.size = CONFIG_IDLETHREAD_STACKSIZE;
    }
  else
    {
      pdump->info.stacks.user.top = (uint32_t) rtcb->adj_stack_ptr;
      pdump->info.stacks.user.size = (uint32_t) rtcb->adj_stack_size;
    }

#if CONFIG_ARCH_INTERRUPTSTACK > 3
  /* Get the limits on the interrupt stack memory */

  pdump->info.stacks.interrupt.top = (uint32_t)&g_intstackbase;
  pdump->info.stacks.interrupt.size  = (CONFIG_ARCH_INTERRUPTSTACK & ~3);

  /* If In interrupt Context save the interrupt stack data centered
   * about the interrupt stack pointer
   */

  if ((pdump->info.flags & INTSTACK_PRESENT) != 0)
    {
      stack_word_t *ps = (stack_word_t *) pdump->info.stacks.interrupt.sp;
      copy_reverse(pdump->istack, &ps[ARRAYSIZE(pdump->istack) / 2],
                   ARRAYSIZE(pdump->istack));
    }

  /* Is it Invalid? */

  if (!(pdump->info.stacks.interrupt.sp <= pdump->info.stacks.interrupt.top &&
        pdump->info.stacks.interrupt.sp > pdump->info.stacks.interrupt.top -
          pdump->info.stacks.interrupt.size))
    {
      pdump->info.flags |= INVALID_INTSTACK_PTR;
    }

#endif
  /* If In interrupt context or User save the user stack data centered
   * about the user stack pointer
   */

  if ((pdump->info.flags & USERSTACK_PRESENT) != 0)
    {
      stack_word_t *ps = (stack_word_t *) pdump->info.stacks.user.sp;
      copy_reverse(pdump->ustack, &ps[ARRAYSIZE(pdump->ustack) / 2],
                   ARRAYSIZE(pdump->ustack));
    }

  /* Is it Invalid? */

  if (!(pdump->info.stacks.user.sp <= pdump->info.stacks.user.top &&
        pdump->info.stacks.user.sp > pdump->info.stacks.user.top -
          pdump->info.stacks.user.size))
    {
      pdump->info.flags |= INVALID_USERSTACK_PTR;
    }

  rv = stm32_bbsram_savepanic(HARDFAULT_FILENO, (uint8_t *)pdump,
                              sizeof(struct fullcontext_t));

  /* Test if memory got wiped because of using _sdata */

  if (rv == -ENXIO)
    {
      char *dead = "Memory wiped - dump not saved!";

      while (*dead)
        {
          up_lowputc(*dead++);
        }
    }
  else if (rv == -ENOSPC)
    {
      /* hard fault again */

      up_lowputc('!');
    }
}
#endif /* CONFIG_STM32H7_SAVE_CRASHDUMP */
#endif /* CONFIG_STM32H7_BBSRAM */
