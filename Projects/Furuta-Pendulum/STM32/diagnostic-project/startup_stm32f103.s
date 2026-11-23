/* startup_stm32f103.s — minimal startup for STM32F103C8T6 */
.syntax unified
.cpu cortex-m3
.thumb

.global _estack
.global Reset_Handler
.global Default_Handler
.global main
.global TIM3_IRQHandler

.section .isr_vector, "a"
.word _estack
.word Reset_Handler           /* vector  1: Reset */
.word Default_Handler         /*  2: NMI */
.word Default_Handler         /*  3: HardFault */
.word Default_Handler         /*  4: MemManage */
.word Default_Handler         /*  5: BusFault */
.word Default_Handler         /*  6: UsageFault */
.word 0
.word 0
.word 0
.word 0
.word Default_Handler         /* 11: SVC */
.word Default_Handler         /* 12: DebugMon */
.word 0
.word Default_Handler         /* 14: PendSV */
.word Default_Handler         /* 15: SysTick */
/* IRQs start at vector 16.
   TIM3 is IRQ 29 → vector 16+29 = 45.
   Fill vectors 16-44 with Default_Handler, place TIM3 at 45. */
.rept 29                      /* IRQ 0 (WWDG) through IRQ 28 (TIM2) */
.word Default_Handler
.endr
.word TIM3_IRQHandler         /* IRQ 29: TIM3 */
.rept 13                      /* IRQ 30 (TIM4) through IRQ 42 (Reserved) */
.word Default_Handler
.endr

.section .text
.thumb_func
Reset_Handler:
    ldr r0, =_estack
    mov sp, r0

    ldr r0, =_sidata
    ldr r1, =_sdata
    ldr r2, =_edata
1:  cmp r1, r2
    bge 2f
    ldr r3, [r0], #4
    str r3, [r1], #4
    b 1b

2:  ldr r1, =_sbss
    ldr r2, =_ebss
    movs r3, #0
3:  cmp r1, r2
    bge 4f
    str r3, [r1], #4
    b 3b

4:  bl main
    b .

.pool

.thumb_func
Default_Handler:
    b .

.thumb_func
.weak TIM3_IRQHandler
TIM3_IRQHandler:
    b .
