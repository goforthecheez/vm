#include "userprog/syscall.h"
#include <stdio.h>
#include <syscall-nr.h>
#include "devices/input.h"
#include "devices/shutdown.h"
#include "filesys/filesys.h"
#include "filesys/file.h"
#include "lib/user/syscall.h"
#include "threads/interrupt.h"
#include "threads/malloc.h"
#include "threads/synch.h"
#include "threads/thread.h"
#include "threads/vaddr.h"
#include "userprog/pagedir.h"
#include "userprog/process.h"

#define ARG_ONE ((int *)f->esp + 1)
#define ARG_TWO ((int *)f->esp + 2)
#define ARG_THREE ((int *)f->esp + 3)

static void syscall_handler (struct intr_frame *);
void halt (void);
pid_t exec (const char *);
int wait (pid_t);
bool create (const char *, unsigned);
bool remove (const char *);
int open (const char *);
int filesize (int );
int read (int, void *, unsigned);
int write (int, const void*, unsigned);
void seek (int, unsigned);
unsigned tell (int);
void close (int);
int mmap (int fd, void *addr);
void munmap (int);

void check_args (void *, void *, void *);
struct file *lookup_fd (int);
void clear_mappings (struct hash_elem *e, void *aux);

void
syscall_init (void) 
{
  intr_register_int (0x30, 3, INTR_ON, syscall_handler, "syscall");
  lock_init (&filesys_lock);
}

static void
syscall_handler (struct intr_frame *f UNUSED) 
{
  /* Check that the stack pointer is valid. */
  if (pagedir_get_page (thread_current ()->pagedir, f->esp) == NULL)
    exit (-1);

  switch (*(int *)f->esp)
    {
      /* For each syscall, check that its arguments are valid, then call
         the appropriate handler, writing the return value to EAX. */
      case SYS_HALT:
        shutdown_power_off ();
        break;
      case SYS_EXIT:
        check_args (ARG_ONE, NULL, NULL);
        exit (*ARG_ONE);
        break;
      case SYS_EXEC:
        check_args (ARG_ONE, NULL, NULL);
        f->eax = exec (*(char **) ARG_ONE);
        break;
      case SYS_WAIT:
        check_args (ARG_ONE, NULL, NULL);
        f->eax = wait (*(unsigned *) ARG_ONE);
        break;
      case SYS_CREATE:
        check_args (ARG_ONE, ARG_TWO, NULL);
        f->eax = create (*(char **) ARG_ONE, *(unsigned *) ARG_TWO);
        break;
      case SYS_REMOVE:
        check_args (ARG_ONE, NULL, NULL);
        f->eax = remove (*(char **) ARG_ONE);
        break;
      case SYS_OPEN:
        check_args (ARG_ONE, NULL, NULL);
	f->eax = open (*(char **) ARG_ONE);
        break;
      case SYS_FILESIZE:
        check_args (ARG_ONE, NULL, NULL);
        f->eax = filesize (*ARG_ONE);
        break;
      case SYS_READ:
        check_args (ARG_ONE, ARG_TWO, ARG_THREE);
	f->eax = read (*ARG_ONE, *(char **) ARG_TWO, *(unsigned *) ARG_THREE);
        break;
      case SYS_WRITE:
        check_args (ARG_ONE, ARG_TWO, ARG_THREE);
	f->eax = write (*ARG_ONE, *(char **) ARG_TWO, *(unsigned *) ARG_THREE);
        break;
      case SYS_TELL:
        check_args (ARG_ONE, NULL, NULL);
        f->eax = tell (*ARG_ONE);
        break;
      case SYS_SEEK:
        check_args (ARG_ONE, ARG_TWO, NULL);
        seek (*ARG_ONE, *(unsigned *) ARG_TWO);
        break;
      case SYS_CLOSE:
        check_args (ARG_ONE, NULL, NULL);
        close (*ARG_ONE);
        break;
      case SYS_MMAP:
        f->eax = mmap (*ARG_ONE, *(uint8_t **) ARG_TWO);
        break;
      case SYS_MUNMAP:
        munmap (*ARG_ONE);
        break;
      default:
        exit (-1);
    }
}

/* Terminates the current user program, returning STATUS to the kernel.
   a status of 0 indicates success and nonzero values indicate errors. */
void
exit (int status)
{
  struct thread *t = thread_current ();

  lock_acquire (&t->parent->child_lock);
  struct child c;
  c.pid = t->tid;;
  struct child *found_child = hash_entry (hash_find (t->parent->children,
                                                     &c.elem),
                                          struct child, elem);
  found_child->done = true;
  found_child->exit_status = status;
  cond_signal (&t->parent->child_cond, &t->parent->child_lock);
  lock_release (&t->parent->child_lock);

  thread_exit ();
}

/* Runs the executable whose name is given in cmd_line, passing any given
   arguments, and returns the new process's program id (pid). If the program
   cannot load or run for any reason, returns -1. */
pid_t
exec (const char *cmd_line)
{
  struct thread *t = thread_current ();

  if (pagedir_get_page (t->pagedir, cmd_line) == NULL)
    exit (-1);

  lock_acquire (&t->child_lock);
  pid_t pid = process_execute (cmd_line);
  cond_wait (&t->child_cond, &t->child_lock);
  t->child_ready = false;
  
  struct child c;
  c.pid = pid;
  struct child *found_child = hash_entry (hash_find (t->children, &c.elem),
                                         struct child, elem);
  if (found_child->exit_status == -1)
    pid = -1;
  lock_release (&t->child_lock);

  return pid;
}

/* Waits for a child process PID to die and returns its exit status.
   If PID was terminated by the kernel, returns -1. Returns -1 immediately if
   PID is invalid or if it was a child of the current process, or if wait() has
   already been successfully called for the given PID. */
int
wait (pid_t pid)
{
  int exit_status = process_wait (pid);

  if (exit_status == -1)
    return -1;

  lock_acquire (&thread_current ()->child_lock);
  struct child c;
  c.pid = pid;
  struct hash_elem *deleted_elem = hash_delete (thread_current ()->children,
                                                &c.elem);
  struct child *deleted_child = hash_entry (deleted_elem, struct child, elem);
  free (deleted_child);
  lock_release (&thread_current ()->child_lock);

  return exit_status;
}

/* Creates a new file initially initial_size bytes in size. Returns true if
   successful, false otherwise. Note the creating a file does not open it. */
bool
create (const char *file, unsigned initial_size)
{
  if (pagedir_get_page (thread_current ()->pagedir, file) == NULL)
    exit (-1);

  lock_acquire (&filesys_lock);
  bool success = filesys_create (file, initial_size);
  lock_release (&filesys_lock);

  return success;
}

/* Deletes the file FILE. Returns true if successful, false otherwise.
   Note that removing an open file does not close it. */
bool
remove (const char *file)
{
  if (pagedir_get_page (thread_current ()->pagedir, file) == NULL)
    exit (-1);

  lock_acquire (&filesys_lock);
  bool success = filesys_remove (file);
  lock_release (&filesys_lock);

  return success;
}

/* Opens the file FILE and returns its file descriptor, or -1 if the file could
   not be opened. */
int
open (const char *file)
{
  struct thread *t = thread_current ();

  if (pagedir_get_page (t->pagedir, file) == NULL)
    exit (-1);

  lock_acquire (&filesys_lock);
  struct file *f = filesys_open (file);
  lock_release (&filesys_lock);
  if (f == NULL)
      return -1;
  hash_insert (t->open_files, &f->elem);

  return f->fd;
}

/* Returns the size, in bytes, of the file open as FD. */
int
filesize (int fd)
{
  struct file *f = lookup_fd (fd);
  if (f == NULL)
      exit (-1);

  lock_acquire (&filesys_lock);
  int len = file_length (f);
  lock_release (&filesys_lock);

  return len;
}

/* Reads size bytes from the file open as FD into BUFFER. Returns the number
   of bytes actually read, or -1 if the file could not be read. */
int
read (int fd, void *buffer, unsigned size)
{
  struct thread *t = thread_current ();

  if (pagedir_get_page (t->pagedir, buffer) == NULL ||
      pagedir_get_page (t->pagedir, (char *)buffer + size) == NULL)
    exit (-1);

  if (fd == STDIN_FILENO)
    {
      unsigned i;
      for (i = 0; i < size; i++)
        input_getc ();
      return i;
    }

  struct file *f = lookup_fd (fd);
  if (f == NULL)
      exit (-1);

  lock_acquire (&filesys_lock);
  int bytes_read = file_read (f, buffer, size);
  lock_release (&filesys_lock);

  return bytes_read;
}

/* Writes SIZE bytes from BUFFER to the open file descriptor FD. Returns the
   number of bytes actually written, which may be less than SIZE if some bytes
   could not be written. */
int
write (int fd, const void *buffer, unsigned size)
{
  struct thread *t = thread_current ();

  if (pagedir_get_page (t->pagedir, buffer) == NULL ||
      pagedir_get_page (t->pagedir, (char *)buffer + size) == NULL)
    exit (-1);

  if (fd == STDOUT_FILENO)
    {
      putbuf (buffer, size);
      return size;
    }

  struct file *f = lookup_fd (fd);
  if (f == NULL)
      exit (-1);

  lock_acquire (&filesys_lock);
  int bytes_written = file_write (f, buffer, size);
  lock_release (&filesys_lock);

  return bytes_written;
}

/* Changes the next byte to be read or written in open file FD to POSITION,
   expressed in bytes from the beginning of the file. A seek past the current
   end of a file is not an error. */
void
seek (int fd, unsigned position)
{
  struct file *f = lookup_fd (fd);
  if (f == NULL)
      exit (-1);

  lock_acquire (&filesys_lock);
  file_seek (f, position);
  lock_release (&filesys_lock);
}

/* Returns the position of the next byte to be read or written in open
   file FD, expressed in bytes from the beginning of the file. */
unsigned
tell (int fd)
{
  struct file *f = lookup_fd (fd);
  if (f == NULL)
      exit (-1);

  lock_acquire (&filesys_lock);
  unsigned pos = file_tell (f);
  lock_release (&filesys_lock);

  return pos;
}

/* Closes file descriptor FD. Exiting or terminating a process implicitly
   closes all its open file descriptors, as if by calling this function for
   each one. */
void close (int fd)
{
  struct thread *t = thread_current ();

  /* File descriptors 0, 1, and 2 are reserved and cannot be closed. */
  if (fd == 0 || fd == 1 || fd == 2)
    exit (-1);

  struct file *f = lookup_fd (fd);
  if (f == NULL)
      exit (-1);

  /* If the lookup succeeded, delete the file from open_files. */
  struct file lookup;
  lookup.fd = fd;
  hash_delete (t->open_files, &lookup.elem);  
  lock_acquire (&filesys_lock);
  file_close (f);
  lock_release (&filesys_lock);
}

/* Maps the file open as fd into the process's virtual address space. The
   entire file is mapped into consecutive virtual pages starting at addr. */
int
mmap (int fd, void *addr)
{
  if (fd == 0 || fd == 1)
    return -1;

  if (addr == 0)
    return -1;

  if (pg_round_down (addr) != addr)
    return -1;

  int size = filesize (fd);
  off_t ofs = 0;
  while (size > 0)
    {
      /* Calculate how to fill this page.
         We will read PAGE_READ_BYTES bytes from FILE     
         and zero the final PAGE_ZERO_BYTES bytes. */
      size_t page_read_bytes = size < PGSIZE ? size : PGSIZE;
      size_t page_zero_bytes = PGSIZE - page_read_bytes;

      /* Create supplemental page table entry. */
      struct sup_page_table_entry *spte = (
          struct sup_page_table_entry *)malloc (
              sizeof (struct sup_page_table_entry));
      spte->upage = addr;
      spte->pinned = false;
      spte->in_memory = false;
      spte->in_swap = false;
      spte->on_disk = true;
      spte->mmapped = true;
      spte->file = lookup_fd (fd);
      spte->ofs = ofs;
      spte->page_read_bytes = page_read_bytes;
      spte->page_zero_bytes = page_zero_bytes;
      spte->writable = true;
      struct hash_elem *inserted = hash_insert (
          thread_current ()->sup_page_table, &(spte->elem));
      if (inserted != NULL)
	return -1;

      /* Advance. */
      size -= page_read_bytes;
      addr = (uint8_t *) addr + PGSIZE;
      ofs += PGSIZE;
    }
  return fd;
}

/* Unmaps the mapping designated by mapping, which must be a mapping ID
   returned by a previous call to mmap by the same process that has not yet
   been unmapped. */
void
munmap (int mapping)
{
  struct thread *t = thread_current ();
  t->sup_page_table->aux = mapping;

  hash_apply (t->sup_page_table, clear_mappings);
}

/* Verify that the passed syscall arguments are valid pointers.
   If not, exit(-1) the user program with an kernel error. */
void
check_args (void *first, void *second, void *third)
{
  uint32_t *pd = thread_current ()->pagedir;

  if (pagedir_get_page (pd, first) == NULL)
    exit (-1);

  if (second != NULL && pagedir_get_page (pd, second) == NULL)
    exit (-1);

  if (third != NULL && pagedir_get_page (pd, third) == NULL)
    exit (-1);
}

/* Given a file descriptor FD, returns its corresponding file. If no file is
   found, return NULL). */
struct file *
lookup_fd (int fd)
{
  struct thread *t = thread_current ();

  struct file lookup;
  lookup.fd = fd;
  struct hash_elem *e = hash_find (t->open_files, &lookup.elem);
  if (e == NULL)
    return NULL;
  struct file *f = hash_entry (e, struct file, elem);

  return f;
}

void
clear_mappings (struct hash_elem *e, void *aux)
{
  struct thread *t = thread_current ();
  struct sup_page_table_entry *spte = (
      struct sup_page_table_entry *) hash_entry (
          e, struct sup_page_table_entry, elem);

  if (lookup_fd ((int) aux) == spte->file)
    pagedir_clear_page (t->pagedir, spte->upage);
  hash_delete (t->sup_page_table, &(spte->elem));
  free (spte);
}
