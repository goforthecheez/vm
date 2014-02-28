#ifndef USERPROG_SYSCALL_H
#define USERPROG_SYSCALL_H

/* File system lock. Currently, the file system does not provide internal
   synchronization, so this will have to suffice. */
struct lock filesys_lock;

void syscall_init (void);
void exit (int);

#endif /* userprog/syscall.h */
