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
int  ContextSwitch(int current_task, TCB_Type pTCB[]);
int ZamenjajTask(int current_task, TCB_Type pTCB[]);


void Task0();
void Task1();
void Task2();
void Task3();





#endif /* SRC_PB_TASKS_H_ */
