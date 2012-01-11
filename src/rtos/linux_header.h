/*  gdb script to update the header file
  according to kernel version and build option
  before executing function awareness
  kernel symbol must be loaded : symbol vmlinux

define awareness
 set logging off
 set logging file linux_header.h
 set logging on

 printf "#define QAT %p\n",&((struct task_struct *)(0))->stack
 set $a=&((struct list_head *)(0))->next
 set $a=(int)$a+(int)&((struct task_struct *)(0))->tasks
 printf "#define NEXT  %p\n",$a
 printf "#define COMM  %p\n",&((struct task_struct *)(0))->comm
 printf "#define MEM  %p\n",&((struct task_struct *)(0))->mm
 printf "#define ONCPU %p\n",&((struct task_struct *)(0))->on_cpu
 printf "#define PID %p\n",&((struct task_struct *)(0))->pid
 printf "#define CPU_CONT %p\n",&((struct thread_info *)(0))->cpu_context
 printf "#define PREEMPT %p\n",&((struct thread_info *)(0))->preempt_count
 printf "#define MM_CTX %p\n",&((struct mm_struct *)(0))->context
 end
*/
#define QAT 0x4
#define NEXT  0x1b0
#define COMM  0x2d4
#define MEM  0x1cc
#define ONCPU 0x18
#define PID 0x1f4
#define CPU_CONT 0x1c
#define PREEMPT 0x4
#define MM_CTX 0x160
