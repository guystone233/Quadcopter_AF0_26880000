#if (defined(OS_TRACE_EN) && (OS_TRACE_EN > 0u))
#define OS_TRACE_TASK_CREATE(p_tcb) trace_task_create(p_tcb)
#define  OS_TRACE_TASK_READY(p_tcb) trace_task_ready(p_tcb)           
#define  OS_TRACE_TASK_SWITCHED_IN(p_tcb) trace_task_switched_in(p_tcb)
#define  OS_TRACE_TASK_DLY(dly_ticks) trace_task_dly(dly_ticks)
#define  OS_TRACE_TASK_SUSPEND(p_tcb) trace_task_suspend(p_tcb)
#define  OS_TRACE_TASK_SUSPENDED(p_tcb) trace_task_suspended(p_tcb)    
#define  OS_TRACE_TASK_RESUME(p_tcb) trace_task_resume(p_tcb)            
#endif
