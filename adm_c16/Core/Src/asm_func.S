.syntax unified
.thumb
.text

@ Simbolos exportados
.global asm_svc
.global asm_sum
.global asm_escalar32
.global asm_escalar16
.global asm_escalar12


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   void asm_svc (void)
@
.thumb_func
    asm_svc:
        svc 0
        bx lr


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   uint32_t asmSum (uint32_t firstOperand, uint32_t secondOperand)
@
@ Parametros recibidos:
@   r0: firstOperand
@   r1: secondOperand
@
@ Valor de retorno:
@   r0: resultado de la suma de firstOperand y secondOperand
@
.thumb_func
    asm_sum:
        add r0, r1  @ r0 = r0 + r1
        bx lr       @ vuelve adonde fue llamada (especificamente, si "pc"
                    @ es la direccion, en el codigo del programa, de la
                    @ instruccion "bl asm_sum" usada para saltar a
                    @ esta funcion, entonces "lr" contiene ese "pc" + 4).
                    @ el valor de retorno debe quedar en el registro r0.
                    @ En este caso, r0 es el resultado de r0 + r1.

        @ Otras alternativas para el retorno
        @ mov pc,lr
        @ bx lr
        @ push {lr} ... pop {pc}


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@   void asm_zeros (uint32_t * vector, uint32_t longitud)
@
@ Parametros recibidos:
@   r0: dirección de memoria de inicio del vector
@   r1: longitud
.thumb_func
    asm_zeros:
        mov r2, 0         @ r2 = 0
        asm_zeros_loop:
		    str r2, [r0]          @ Asigna 0 a la palabra en dirección r0.
		    add r0, 4             @ Pasa al siguiente elemento del vector.
		    subs r1, 1            @ Resta 1 a la cantidad de elementos y actualiza flags de estado del procesador. En particular, estamos interesados en el flag Z.
		    bne asm_zeros_loop    @ La instrucción se ejecuta mientras el resultado de r1 - 1 no sea cero (Z == 0).
	    bx lr                     @ La instrucción de salto no se ejecutó porque Z == 1, volvemos a quién nos llamó.


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@   void productoEscalar32 (uint32_t * vectorIn, uint32_t * vectorOut, uint32_t longitud, uint32_t escalar)
@
@ Parametros recibidos:
@   r0: dirección de memoria del vectorIN
@   r1: dirección de memoria del vectorOUT
@   r2: longitud
@   r3: escalar
.thumb_func
    asm_escalar32:
        push {r4}			  	@ Contexto de r4 en stack
        asm_escalar32_loop:
	        ldr r4, [r0]		  @ Guarda elemento del venctorIN en r4
	        muls r4, r3, r4		  @ Multiplica r4*r3 donde r3 es el escalar
		    str r4, [r1]          @ Asigna r4 a la palabra en dirección r1 (vectorOUT).
		    add r0, 4             @ Pasa a la siguiente palabra/elemento en vectorIN.
		    add r1, 4			  @ Pasa a la siguiente palabra/elemento en vectorOUT.
		    subs r2, 1            @ Resta 1 a la cantidad de elementos y actualiza flags de estado del procesador. En particular, estamos interesados en el flag Z.
		    bne asm_escalar32_loop	@ La instrucción se ejecuta mientras el resultado de r1 - 1 no sea cero (Z == 0).
	  	pop {r4}			  @ recuperar r4 del stack
	    bx lr                 @ La instrucción de salto no se ejecutó porque Z == 1

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@   void productoEscalar16 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint32_t escalar)
@
@ Parametros recibidos:
@   r0: dirección de memoria de inicio del vectorIN
@   r1: dirección de memoria de inicio del vectorOUT
@   r2: longitud
@   r3: escalar
.thumb_func
    asm_escalar16:
        push {r4}			  @ salvar el contexto de r4 en el stack
        asm_escalar16_loop:
	        ldrh r4, [r0]		  @ Guarda elemento del venctorIN en r4
	        muls r4, r3, r4		  @ Multiplica r4*r3 donde r3 es el escalar
		    strh r4, [r1]         @ Asigna r4 a la palabra en dirección r1 (vectorOUT).
		    add r0, 4             @ Pasa a la siguiente palabra/elemento en vectorIN.
		    add r1, 4			  @ Pasa a la siguiente palabra/elemento en vectorOUT.
		    subs r2, 1            @ Resta 1 a la cantidad de elementos y actualiza flags de estado del procesador.
		    bne asm_escalar16_loop	@ La instrucción se ejecuta mientras el resultado de r1 - 1 no sea cero (Z == 0).
	  	pop {r4}			  		@ recuperar r4 del stack
	    bx lr                 		@ La instrucción de salto no se ejecutó porque Z == 1.

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@   void productoEscalar12 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint32_t escalar)
@
@ Parametros recibidos:
@   r0: dirección de memoria de inicio del vectorIN
@   r1: dirección de memoria de inicio del vectorOUT
@   r2: longitud
@   r3: escalar
.thumb_func               @ Se avisa al emsablador que esta es una funcion
    asm_escalar12:            @ Etiqueta/nombre de la función.
        push {r4-r5}		  @ salvar el contexto de r4-r5 en el stack
        mov r5, #4095		  @ Guarda #4095 en r5
	    asm_escalar12_loop:
	        ldrh r4, [r0]		  @ Guarda elemento del venctorIN en r4
	        muls r4, r3, r4		  @ Multiplica r4*r3 donde r3 es el escalar
	        usat r4, 12, r4		  @ check si saturo a 12 bits
		    strh r4, [r1]         @ Asigna r4 a la palabra en dirección r1 (vectorOUT).
		    add r0, 4             @ Pasa a la siguiente palabra/elemento en vectorIN.
		    add r1, 4			  @ Pasa a la siguiente palabra/elemento en vectorOUT.
		    subs r2, 1            @ Resta 1 a la cantidad de elementos y actualiza flags de estado del procesador.
		    bne asm_escalar12_loop	@ La instrucción se ejecuta mientras el resultado de r1 - 1 no sea cero (Z == 0).
	  	pop {r4-r5}			  		@ recuperar r4 del stack
	    bx lr                 		@ La instrucción de salto no se ejecutó porque Z == 1, volvemos a quién nos llamó.


