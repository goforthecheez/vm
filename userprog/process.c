#include "userprog/process.h"
#include <debug.h>
#include <inttypes.h>
#include <round.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "userprog/gdt.h"
#include "userprog/pagedir.h"
#include "userprog/tss.h"
#include "filesys/directory.h"
#include "filesys/file.h"
#include "filesys/filesys.h"
#include "lib/user/syscall.h"
#include "threads/flags.h"
#include "threads/init.h"
#include "threads/interrupt.h"
#include "threads/malloc.h"
#include "threads/palloc.h"
#include "threads/thread.h"
#include "threads/synch.h"
#include "threads/vaddr.h"

#define CMD_LINE_MAX_CHARS 700

static thread_func start_process NO_RETURN;
static bool load (char *cmdline, void (**eip) (void), void **esp);

/* Starts a new thread running a user program loaded from
   CMD_LINE.  The new thread may be scheduled (and may even exit)
   before process_execute() returns.  Returns the new process's
   thread id, or TID_ERROR if the thread cannot be created. */
tid_t
process_execute (const char *cmd_line) 
{
  /* If the command line length exceeds the maximum, politely warn the user. */
  if (strlen (cmd_line) > CMD_LINE_MAX_CHARS)
    {
      printf ("Please limit command lines to %d  characters or fewer\n",
              CMD_LINE_MAX_CHARS);
      return TID_ERROR;
    }

  char *load_copy;
  char *file_name_copy;
  tid_t tid;

  /* Make a copy of CMD_LINE.
     Otherwise there's a race between the caller and load(). */
  load_copy = palloc_get_page (0);
  if (load_copy == NULL)
    return TID_ERROR;
  strlcpy (load_copy, cmd_line, PGSIZE);

  /* Make another copy of CMD_LINE.
     This copy is only used to pull the file name. */
  file_name_copy = palloc_get_page (0);
  if (file_name_copy == NULL)
    return TID_ERROR;
  strlcpy (file_name_copy, cmd_line, PGSIZE);

  char *unused_saved_ptr;
  char *file_name = strtok_r (file_name_copy, " ", &unused_saved_ptr);

  /* Create a new thread to execute CMD_LINE. */
  tid = thread_create (file_name, PRI_DEFAULT, start_process, load_copy);
  palloc_free_page (file_name_copy);

  if (tid == TID_ERROR)
    palloc_free_page (load_copy);

  return tid;
}

/* A thread function that loads a user process and starts it
   running. */
static void
start_process (void *cmd_line_)
{
  char *cmd_line = cmd_line_;
  struct intr_frame if_;
  bool success;

  /* Initialize interrupt frame and load executable. */
  memset (&if_, 0, sizeof if_);
  if_.gs = if_.fs = if_.es = if_.ds = if_.ss = SEL_UDSEG;
  if_.cs = SEL_UCSEG;
  if_.eflags = FLAG_IF | FLAG_MBS;
  success = load (cmd_line, &if_.eip, &if_.esp);

  /* Signal to exec that the child process has loaded. */
  struct thread *p = thread_current ()->parent;
  lock_acquire (&p->child_lock);
  p->child_ready = true;
  cond_signal (&p->child_cond, &p->child_lock);
  lock_release (&p->child_lock);

  /* If load failed, quit. */
  palloc_free_page (cmd_line);
  if (!success)
    exit (-1);

  /* Start the user process by simulating a return from an
     interrupt, implemented by intr_exit (in
     threads/intr-stubs.S).  Because intr_exit takes all of its
     arguments on the stack in the form of a `struct intr_frame',
     we just point the stack pointer (%esp) to our stack frame
     and jump to it. */
  asm volatile ("movl %0, %%esp; jmp intr_exit" : : "g" (&if_) : "memory");
  NOT_REACHED ();
}

/* Waits for thread TID to die and returns its exit status.  If
   it was terminated by the kernel (i.e. killed due to an
   exception), returns -1.  If TID is invalid or if it was not a
   child of the calling process, or if process_wait() has already
   been successfully called for the given TID, returns -1
   immediately, without waiting. */
int
process_wait (tid_t child_tid UNUSED) 
{
  struct thread *t = thread_current ();

  /* Look for the child process. */
  lock_acquire (&t->child_lock);
  struct child c;
  c.pid = child_tid;
  struct hash_elem *e = hash_find (t->children, &c.elem);
  if (e == NULL)
    {
      lock_release(&thread_current ()->child_lock);
      return -1;
    }
  struct child *found_child = hash_entry (e, struct child, elem);

  /* Wait for the child process to finish executing. */
  while (!found_child->done)
    cond_wait (&t->child_cond, &t->child_lock);

  /* Return the exit status. */
  int exit_status = found_child->exit_status;
  lock_release (&t->child_lock);

  return exit_status;
}

/* Print the process termination message and free the current process's
   resources. */
void
process_exit (void)
{
  uint32_t *pd;

  /* Print process termination message. */
  struct thread *t = thread_current ();
  struct child c;
  c.pid = t->tid;
  struct child *found_child = hash_entry (hash_find (t->parent->children,
                                                     &c.elem),
                                          struct child, elem);
  int status = found_child->exit_status;
  printf ("%s: exit(%d)\n", t->name, status);

  /* Destroy the current process's page directory and switch back
     to the kernel-only page directory. */
  pd = t->pagedir;
  if (pd != NULL) 
    {
      /* Correct ordering here is crucial.  We must set
         cur->pagedir to NULL before switching page directories,
         so that a timer interrupt can't switch back to the
         process page directory.  We must activate the base page
         directory before destroying the process's page
         directory, or our active page directory will be one
         that's been freed (and cleared). */
      t->pagedir = NULL;
      pagedir_activate (NULL);
      pagedir_destroy (pd);
    }
}

/* Sets up the CPU for running user code in the current
   thread.
   This function is called on every context switch. */
void
process_activate (void)
{
  struct thread *t = thread_current ();

  /* Activate thread's page tables. */
  pagedir_activate (t->pagedir);

  /* Set thread's kernel stack for use in processing
     interrupts. */
  tss_update ();
}

/* We load ELF binaries.  The following definitions are taken
   from the ELF specification, [ELF1], more-or-less verbatim.  */

/* ELF types.  See [ELF1] 1-2. */
typedef uint32_t Elf32_Word, Elf32_Addr, Elf32_Off;
typedef uint16_t Elf32_Half;

/* For use with ELF types in printf(). */
#define PE32Wx PRIx32   /* Print Elf32_Word in hexadecimal. */
#define PE32Ax PRIx32   /* Print Elf32_Addr in hexadecimal. */
#define PE32Ox PRIx32   /* Print Elf32_Off in hexadecimal. */
#define PE32Hx PRIx16   /* Print Elf32_Half in hexadecimal. */

/* Executable header.  See [ELF1] 1-4 to 1-8.
   This appears at the very beginning of an ELF binary. */
struct Elf32_Ehdr
  {
    unsigned char e_ident[16];
    Elf32_Half    e_type;
    Elf32_Half    e_machine;
    Elf32_Word    e_version;
    Elf32_Addr    e_entry;
    Elf32_Off     e_phoff;
    Elf32_Off     e_shoff;
    Elf32_Word    e_flags;
    Elf32_Half    e_ehsize;
    Elf32_Half    e_phentsize;
    Elf32_Half    e_phnum;
    Elf32_Half    e_shentsize;
    Elf32_Half    e_shnum;
    Elf32_Half    e_shstrndx;
  };

/* Program header.  See [ELF1] 2-2 to 2-4.
   There are e_phnum of these, starting at file offset e_phoff
   (see [ELF1] 1-6). */
struct Elf32_Phdr
  {
    Elf32_Word p_type;
    Elf32_Off  p_offset;
    Elf32_Addr p_vaddr;
    Elf32_Addr p_paddr;
    Elf32_Word p_filesz;
    Elf32_Word p_memsz;
    Elf32_Word p_flags;
    Elf32_Word p_align;
  };

/* Values for p_type.  See [ELF1] 2-3. */
#define PT_NULL    0            /* Ignore. */
#define PT_LOAD    1            /* Loadable segment. */
#define PT_DYNAMIC 2            /* Dynamic linking info. */
#define PT_INTERP  3            /* Name of dynamic loader. */
#define PT_NOTE    4            /* Auxiliary info. */
#define PT_SHLIB   5            /* Reserved. */
#define PT_PHDR    6            /* Program header table. */
#define PT_STACK   0x6474e551   /* Stack segment. */

/* Flags for p_flags.  See [ELF3] 2-3 and 2-4. */
#define PF_X 1          /* Executable. */
#define PF_W 2          /* Writable. */
#define PF_R 4          /* Readable. */

static bool setup_stack (void **esp, char **argv, int argc);
static bool validate_segment (const struct Elf32_Phdr *, struct file *);
static bool load_segment (struct file *file, off_t ofs, uint8_t *upage,
                          uint32_t read_bytes, uint32_t zero_bytes,
                          bool writable);

/* Loads an ELF executable from the first token of CMD_LINE into the
   current thread. Stores the executable's entry point into *EIP and
   its initial stack pointer into *ESP.
   Returns true if successful, false otherwise. */
bool
load (char *cmd_line, void (**eip) (void), void **esp) 
{
  char **argv = (char **)malloc (CMD_LINE_MAX_CHARS * sizeof (char *));
  if (argv == NULL)
    return false;

  /* Parse the command line. */
  char *token, *save_ptr;
  int i = 0;
  for (token = strtok_r (cmd_line, " ", &save_ptr); token != NULL;
       token = strtok_r (NULL, " ", &save_ptr))
    {
      argv[i] = token;
      i++;
    }
  int argc = i;

  struct thread *t = thread_current ();
  struct Elf32_Ehdr ehdr;
  struct file *file = NULL;
  off_t file_ofs;
  bool success = false;

  /* Allocate and activate page directory. */
  t->pagedir = pagedir_create ();
  if (t->pagedir == NULL) 
    goto done;
  process_activate ();

  /* Open executable file. */
  file = filesys_open (argv[0]);
  if (file == NULL) 
    {
      printf ("load: %s: open failed\n", argv[0]);
      success = false;
      goto done; 
    }

  /* Deny writes to executables. */
  file_deny_write (file);
  t->my_executable = file;

  /* Read and verify executable header. */
  if (file_read (file, &ehdr, sizeof ehdr) != sizeof ehdr
      || memcmp (ehdr.e_ident, "\177ELF\1\1\1", 7)
      || ehdr.e_type != 2
      || ehdr.e_machine != 3
      || ehdr.e_version != 1
      || ehdr.e_phentsize != sizeof (struct Elf32_Phdr)
      || ehdr.e_phnum > 1024) 
    {
      printf ("load: %s: error loading executable\n", argv[0]);
      goto done; 
    }

  /* Read program headers. */
  file_ofs = ehdr.e_phoff;
  for (i = 0; i < ehdr.e_phnum; i++) 
    {
      struct Elf32_Phdr phdr;

      if (file_ofs < 0 || file_ofs > file_length (file))
        goto done;
      file_seek (file, file_ofs);

      if (file_read (file, &phdr, sizeof phdr) != sizeof phdr)
        goto done;
      file_ofs += sizeof phdr;
      switch (phdr.p_type) 
        {
        case PT_NULL:
        case PT_NOTE:
        case PT_PHDR:
        case PT_STACK:
        default:
          /* Ignore this segment. */
          break;
        case PT_DYNAMIC:
        case PT_INTERP:
        case PT_SHLIB:
          goto done;
        case PT_LOAD:
          if (validate_segment (&phdr, file)) 
            {
              bool writable = (phdr.p_flags & PF_W) != 0;
              uint32_t file_page = phdr.p_offset & ~PGMASK;
              uint32_t mem_page = phdr.p_vaddr & ~PGMASK;
              uint32_t page_offset = phdr.p_vaddr & PGMASK;
              uint32_t read_bytes, zero_bytes;
              if (phdr.p_filesz > 0)
                {
                  /* Normal segment.
                     Read initial part from disk and zero the rest. */
                  read_bytes = page_offset + phdr.p_filesz;
                  zero_bytes = (ROUND_UP (page_offset + phdr.p_memsz, PGSIZE)
                                - read_bytes);
                }
              else 
                {
                  /* Entirely zero.
                     Don't read anything from disk. */
                  read_bytes = 0;
                  zero_bytes = ROUND_UP (page_offset + phdr.p_memsz, PGSIZE);
                }
              if (!load_segment (file, file_page, (void *) mem_page,
                                 read_bytes, zero_bytes, writable))
                goto done;
            }
          else
            goto done;
          break;
        }
    }

  /* Set up stack. */
  if (!setup_stack (esp, argv, argc))
    goto done;

  /* Start address. */
  *eip = (void (*) (void)) ehdr.e_entry;

  success = true;

 done:
  /* We arrive here whether the load is successful or not. */
  free (argv);
  return success;
}

/* load() helpers. */

/* Checks whether PHDR describes a valid, loadable segment in
   FILE and returns true if so, false otherwise. */
static bool
validate_segment (const struct Elf32_Phdr *phdr, struct file *file) 
{
  /* p_offset and p_vaddr must have the same page offset. */
  if ((phdr->p_offset & PGMASK) != (phdr->p_vaddr & PGMASK)) 
    return false; 

  /* p_offset must point within FILE. */
  if (phdr->p_offset > (Elf32_Off) file_length (file)) 
    return false;

  /* p_memsz must be at least as big as p_filesz. */
  if (phdr->p_memsz < phdr->p_filesz) 
    return false; 

  /* The segment must not be empty. */
  if (phdr->p_memsz == 0)
    return false;
  
  /* The virtual memory region must both start and end within the
     user address space range. */
  if (!is_user_vaddr ((void *) phdr->p_vaddr))
    return false;
  if (!is_user_vaddr ((void *) (phdr->p_vaddr + phdr->p_memsz)))
    return false;

  /* The region cannot "wrap around" across the kernel virtual
     address space. */
  if (phdr->p_vaddr + phdr->p_memsz < phdr->p_vaddr)
    return false;

  /* Disallow mapping page 0.
     Not only is it a bad idea to map page 0, but if we allowed
     it then user code that passed a null pointer to system calls
     could quite likely panic the kernel by way of null pointer
     assertions in memcpy(), etc. */
  if (phdr->p_vaddr < PGSIZE)
    return false;

  /* It's okay. */
  return true;
}

/* Loads a segment starting at offset OFS in FILE at address
   UPAGE.  In total, READ_BYTES + ZERO_BYTES bytes of virtual
   memory are initialized, as follows:

        - READ_BYTES bytes at UPAGE must be read from FILE
          starting at offset OFS.

        - ZERO_BYTES bytes at UPAGE + READ_BYTES must be zeroed.

   The pages initialized by this function must be writable by the
   user process if WRITABLE is true, read-only otherwise.

   Return true if successful, false if a memory allocation error
   or disk read error occurs. */
static bool
load_segment (struct file *file, off_t ofs, uint8_t *upage,
              uint32_t read_bytes, uint32_t zero_bytes, bool writable) 
{
  ASSERT ((read_bytes + zero_bytes) % PGSIZE == 0);
  ASSERT (pg_ofs (upage) == 0);
  ASSERT (ofs % PGSIZE == 0);

  while (read_bytes > 0 || zero_bytes > 0) 
    {
      /* Calculate how to fill this page.
         We will read PAGE_READ_BYTES bytes from FILE
         and zero the final PAGE_ZERO_BYTES bytes. */
      size_t page_read_bytes = read_bytes < PGSIZE ? read_bytes : PGSIZE;
      size_t page_zero_bytes = PGSIZE - page_read_bytes;

      /* Create supplemental page table entry. */
      struct sup_page_table_entry *spte = (
          struct sup_page_table_entry *)malloc (
              sizeof (struct sup_page_table_entry));
      spte->upage = upage;
      spte->pinned = false;
      spte->in_memory = false;
      spte->in_swap = false;
      spte->on_disk = true;
      spte->file = file;
      spte->ofs = ofs;
      spte->page_read_bytes = page_read_bytes;
      spte->page_zero_bytes = page_zero_bytes;
      spte->writable = writable;
      struct hash_elem *inserted = hash_insert (
          thread_current ()->sup_page_table, &(spte->elem));
      if (inserted != NULL)
        {
          printf ("Error: Creating duplicate supplemental page table entry.");
          return false;
        }

      /* Advance. */
      read_bytes -= page_read_bytes;
      zero_bytes -= page_zero_bytes;
      upage += PGSIZE;
      ofs += PGSIZE;
    }

  return true;
}

/* Creates a user stack, according to the convention described in
   "3.5.1 Program Startup Details." */
static bool
setup_stack (void **esp, char **argv, int argc) 
{
  uint8_t *kpage;
  bool success = false;

  kpage = palloc_get_page (PAL_USER | PAL_ZERO);
  if (kpage != NULL) 
    {
      /* Create supplemental page table entry. */
      struct sup_page_table_entry *spte = (
          struct sup_page_table_entry *)malloc (
              sizeof (struct sup_page_table_entry));
      uint8_t *upage = ((uint8_t *) PHYS_BASE) - PGSIZE;
      spte->upage = upage;
      spte->pinned = true;
      spte->in_memory = true;
      spte->in_swap = false;
      spte->on_disk = false;
      struct hash_elem *inserted = hash_insert (
          thread_current ()->sup_page_table, &(spte->elem));
      if (inserted != NULL)
        {
          printf ("Error: Creating duplicate supplemental page table entry.");
          return false;
        }      

      success = install_page (upage, kpage, true);
      if (success)
        {
          *esp = PHYS_BASE;

          int total_len = 0;
          int i, len;
          /* Read arguments from right to left. */
          for (i = argc - 1; i >= 0; i--)
            {
              len = strlen (argv[i]) + 1;      // Include null terminator
              total_len += len;
              *esp = (char *)*esp - len;
              strlcpy (*esp, argv[i], len);
              argv[i] = *esp;                  // Save address of first char
                                               // of argv[i]
            }

          /* Correct word-alignment */
          int pad = 4 - (total_len % 4);
          if (pad == 4)
            pad = 0;
          for (i = 0; i < pad; i++)
            *esp = (char *)*esp - 1;

          /* Push pointers to args onto the stack. */
          *esp = (int *)*esp - 1;              // NULL pointer for argc'th
                                               // index
          for (i = argc - 1; i >= 0; i--)
            {
              *esp = (int *)*esp - 1;
              memcpy (*esp, &argv[i], sizeof (void *));
            }

          /* Push pointer to beginning of argv[] onto stack. */
          *esp = (int *)*esp - 1;
          void *argv_front = (int *)*esp + 1;
          memcpy (*esp, &argv_front, sizeof (void *));

          /* Push argc onto stack. */
	  *esp = (int *)*esp - 1;
          memcpy (*esp, &argc, sizeof (int));

          /* Push dummy return address onto stack. */
          *esp = (int *)*esp - 1;
        }
      else
        palloc_free_page (kpage);
    }
  return success;
}

/* Adds a mapping from user virtual address UPAGE to kernel
   virtual address KPAGE to the page table.
   If WRITABLE is true, the user process may modify the page;
   otherwise, it is read-only.
   UPAGE must not already be mapped.
   KPAGE should probably be a page obtained from the user pool
   with palloc_get_page().
   Returns true on success, false if UPAGE is already mapped or
   if memory allocation fails. */
bool
install_page (void *upage, void *kpage, bool writable)
{
  struct thread *t = thread_current ();

  /* Verify that there's not already a page at that virtual
     address, then map our page there. */
  return (pagedir_get_page (t->pagedir, upage) == NULL
          && pagedir_set_page (t->pagedir, upage, kpage, writable));
}
