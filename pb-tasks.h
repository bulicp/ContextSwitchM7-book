/*
 * pb-tasks.h
 *
 *  Created on: Oct 25, 2023
 *      Author: patriciobulic
 */

#ifndef SRC_PB_TASKS_H_
#define SRC_PB_TASKS_H_




#define NTASKS 4
#define TASK_STACK_SIZE 256


typedef struct{
	unsigned int* sp;			// nazadnje vidseni SP opravila
	void (*pTaskFunction)();	// naslov finkcije, ki implementira opravilo
} TCB_Type;

typedef struct{
	unsigned int r0;
	unsigned int r1;
	unsigned int r2;
	unsigned int r3;
	unsigned int r12;
	unsigned int lr;
	unsigned int pc;
	unsigned int psr;
} HWSF_Type;

typedef struct{
	unsigned int r4;
	unsigned int r5;
	unsigned int r6;
	unsigned int r7;
	unsigned int r8;
	unsigned int r9;
	unsigned int r10;
	unsigned int r11;
} SWSF_Type;


void TaskCreate(TCB_Type* pTCB, unsigned int* pTaskStackBase, void (*TaskFunction)());
void TaskInit(TCB_Type* pTCB);
void InitScheduler(unsigned int* pStackRegion, TCB_Type pTCB[], void (*TaskFunctions[])());
void InitSchedulerSVC(unsigned int* pStackRegion, TCB_Type pTCB[], void (*TaskFunctions[])());
int  ContextSwitch(int current_task, TCB_Type pTCB[]);


void Task0();
void Task1();
void Task2();
void Task3();



/*
	The inline function saves the context (r4-r11) into the software stack-frame on the process stack.
 	The hardware stack frame already contains the context saved by hardware (pc, lr, sp, r0-r3, r12, spr)

 	This function should be defined into the same translational module in which is use
 	as it is defined as static inline!

 	Pa3cio Bulic, 27.10.2023
*/
__attribute__((always_inline)) static inline void __SAVE_CONTEXT(void)
{
  unsigned int tmp;
  __asm__ volatile
  (
  " MRS   %0, psp                    \n"
  " STMFD %0!, {r4-r11}              \n"
  " MSR   psp, %0                    \n"
  : "=r" (tmp)
  );
}


/*
	The function restores the context (r4-r11) from the software stack-frame on the process stack.

 	This function should be defined into the same translational module in which is use
 	as it is defined as static inline!

 	Pa3cio Bulic, 27.10.2023
*/
__attribute__((always_inline)) static inline void __RESTORE_CONTEXT(void)
{
  unsigned int tmp;
  __asm__ volatile
  (
  " MRS   %0, psp                    \n"
  " LDMFD %0!, {r4-r11}              \n"
  " MSR   psp, %0                    \n"
  : "=r" (tmp)
  );
}




/*
	The function implements the SVC handler in

 	This function should be defined into the same translational module in which is use
 	as it is defined as static inline!

 	Pa3cio Bulic, 27.10.2023
*/
__attribute__((always_inline)) static inline void __SVC(unsigned int number)
{
	 __asm__ volatile ("SVC %0" : : "r" (number) : );
}











#endif /* SRC_PB_TASKS_H_ */
