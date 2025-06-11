/*
 * File name: interrupts.h
 * Description: Header file containing CPU2 ISR and initialization routine declaration.
 */
#ifndef INC_INTERRUPTS_CPU2_H_
#define INC_INTERRUPTS_CPU2_H_

/***********************************************************************
 * FSI INTERRUPT HANDLERS
 ***********************************************************************/

// FSIRX data frame received ISR:
__attribute__((interrupt)) void fsi_rx_data_frame_isr(void);

// FSIRX error requiring reestablishing the communication link:
__attribute__((interrupt)) void fsi_rx_error_isr(void);

/***********************************************************************
 * Timer Interrupt Handlers
 ***********************************************************************/

// Timer ISR used to trigger packet transmissions after a delay:
__attribute__((interrupt)) void timer_timeout_isr(void);

#endif // INC_INTERRUPTS_CPU2_H_
