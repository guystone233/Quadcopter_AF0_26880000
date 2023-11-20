#include "usart.h"
#include "os_cpu.h"
#include <stdio.h>
#include "ucos_ii.h"
/*
#define OS_TRACE_TASK_CREATE(p_tcb) 
#define  OS_TRACE_TASK_READY(p_tcb)   
#define  OS_TRACE_TASK_SWITCHED_IN(p_tcb)           
#define  OS_TRACE_TASK_DLY(dly_ticks)
#define  OS_TRACE_TASK_SUSPEND(p_tcb)
#define  OS_TRACE_TASK_SUSPENDED(p_tcb)             
#define  OS_TRACE_TASK_RESUME(p_tcb)          
*/
void trace_task_create(OS_TCB *ptcb)
{
    char str[100]={0};
    sprintf(str,"%d[task create]%d\r\n",OSTimeGet(), ptcb->OSTCBPrio);
    SendString(str);
}
void trace_task_ready(OS_TCB *ptcb)
{
    char str[100]={0};
    sprintf(str,"%d[task ready]%d\r\n",OSTimeGet(), ptcb->OSTCBPrio);
    SendString(str);
}
void trace_task_switched_in(OS_TCB *ptcb)
{
    char str[100]={0};
    sprintf(str,"%d[task switched in]%d\r\n",OSTimeGet(), ptcb->OSTCBPrio);
    SendString(str);
}
void trace_task_dly(uint32_t dly_ticks)
{
//    char str[100]={0};
//    sprintf(str,"%d[task dly]%d\r\n",OSTimeGet(), dly_ticks);
//OS_ENTER_CRITICAL();     
//    SendString(str);
//		OS_EXIT_CRITICAL();     
}
void trace_task_suspend(OS_TCB *ptcb)
{
    char str[100]={0};
    sprintf(str,"%d[task suspend]%d\r\n",OSTimeGet(), ptcb->OSTCBPrio);
    SendString(str);
}
void trace_task_suspended(OS_TCB *ptcb)
{
//    char str[100]={0};
//    sprintf(str,"%d[task suspended]%d\r\n",OSTime, ptcb->OSTCBPrio);
//    SendString(str);
}
void trace_task_resume(OS_TCB *ptcb)
{
    char str[100]={0};
    sprintf(str,"%d[task resume]%d\r\n",OSTimeGet(), ptcb->OSTCBPrio);
    SendString(str);
}
