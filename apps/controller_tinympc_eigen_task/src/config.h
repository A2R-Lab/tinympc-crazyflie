
#define TINYMPC_TASK_STACKSIZE        (10 * configMINIMAL_STACK_SIZE)
#define TINYMPC_TASK_NAME       "TINYMPC ADMM"
// Must share priority 0 with CRTP link service so radio polling drains during a solve.
#define TINYMPC_TASK_PRI        0
