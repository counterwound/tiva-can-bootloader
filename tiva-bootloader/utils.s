;;based on bl_startup_ccs.s

	.cdecls C, NOLIST, WARN
	%{
	    #include "inc/hw_nvic.h"
	    #include "inc/hw_sysctl.h"
	    #include "bl_config.h"
	%}

	.thumb

    .global CallApplication
	.thumbfunc CallApplication
CallApplication: .asmfunc
    ;;
    ;; Copy the application's vector table to the target address if necessary.
    ;; Note that incorrect boot loader configuration could cause this to
    ;; corrupt the code!  Setting VTABLE_START_ADDRESS to 0x20000000 (the start
    ;; of SRAM) is safe since this will use the same memory that the boot loader
    ;; already uses for its vector table.  Great care will have to be taken if
    ;; other addresses are to be used.
    ;;
 .if (APP_START_ADDRESS != VTABLE_START_ADDRESS)
    movw    r0, #(VTABLE_START_ADDRESS & 0xffff)
 .if (VTABLE_START_ADDRESS > 0xffff)
    movt    r0, #(VTABLE_START_ADDRESS >> 16)
 .endif
    movw    r1, #(APP_START_ADDRESS & 0xffff)
 .if (APP_START_ADDRESS > 0xffff)
    movt    r1, #(APP_START_ADDRESS >> 16)
 .endif

    ;;
    ;; Calculate the end address of the vector table assuming that it has the
    ;; maximum possible number of vectors.  We don't know how many the app has
    ;; populated so this is the safest approach though it may copy some non
    ;; vector data if the app table is smaller than the maximum.
    ;;
    movw    r2, #(70 * 4)
    adds    r2, r2, r0
VectorCopyLoop:
        ldr     r3, [r1], #4
        str     r3, [r0], #4
        cmp     r0, r2
        blt     VectorCopyLoop
 .endif

    ;;
    ;; Set the application's vector table start address.  Typically this is the
    ;; application start address but in some cases an application may relocate
    ;; this so we can't assume that these two addresses are equal.
    ;;
    movw    r0, #(VTABLE_START_ADDRESS & 0xffff)
 .if (VTABLE_START_ADDRESS > 0xffff)
    movt    r0, #(VTABLE_START_ADDRESS >> 16)
 .endif
    movw    r1, #(NVIC_VTABLE & 0xffff)
    movt    r1, #(NVIC_VTABLE >> 16)
    str     r0, [r1]

    ;;
    ;; Load the stack pointer from the application's vector table.
    ;;
 .if (APP_START_ADDRESS != VTABLE_START_ADDRESS)
    movw    r0, #(APP_START_ADDRESS & 0xffff)
 .if (APP_START_ADDRESS > 0xffff)
    movt    r0, #(APP_START_ADDRESS >> 16)
 .endif
 .endif
    ldr     sp, [r0]

    ;;
    ;; Load the initial PC from the application's vector table and branch to
    ;; the application's entry point.
    ;;
    ldr     r0, [r0, #4]
    bx      r0
    .endasmfunc
    
    .global Delay
    .thumbfunc Delay
Delay: .asmfunc
    subs    r0, #1
    bne     Delay
    bx      lr
    .endasmfunc