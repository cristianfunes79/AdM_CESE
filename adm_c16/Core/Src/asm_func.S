.syntax unified
.thumb
.text

@ Simbolos exportados
.global asm_svc
.global asm_zeros
.global asm_sum
.global asm_escalar32
.global asm_escalar16
.global asm_escalar12
.global asm_filtroVentana10
.global asm_pack16
.global asm_max
.global asm_downsampleM
.global asm_invertir
.global asm_eco

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
        @ push c{lr} ... pop {pc}


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


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   void filtroVentana10 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud)
@
@ Parametros recibidos:
@   r0: dirección de memoria de inicio del vector IN
@   r1: dirección de memoria de inicio del vector OUT
@   r2: longitud
.thumb_func               @ Se avisa al emsablador que esta es una funcion
    asm_filtroVentana10:            @ Etiqueta/nombre de la función.
        push {r3-r8}			@ salvar el contexto de r3-r8 en el stack
        mov r3, r0				@ Copia la direccion de memoria del vector IN en r3
        lsl r4, r2, 2			@ Multiplica por 2 la longitud (desplaza a la izq)
        asm_filtroVentana10_bucle:  @ Etiqueta de comienzo de bucle.
	        mov r5, #10			@ Asigna #10 a r5
	        	bucle2:
		        adds r6, r3, r5
		        subs r6,r4,r6   @ En r6 guarda la posicion de memoria del final de la ventana, checkea que este en rango y actualiza flags
		      	ldr r6, [r0]	@ carga el elemento del vector en r6
		        it pl			@ Si la resta dio positivo
		        addpl r7, r6 	@ Suma cero a r7

		        subs r6, r3, r5
		        subs r6, r3, r6 @ En r6 guarda la posicion de memoria del comienzo de la ventana, checkea que este en rango y actualiza flags
		        ldr r6, [r0]    @ carga el elemento del vector en r6
		        it pl           @ Si la resta dio positivo
		        addpl r7,r6     @ Suma cero a r7
		        subs r5,1
		        bne bucle2			@sale del bucle
		    mov r8, #10
		    udiv r7,r7,r8			@calcula el proemdio
		    strh r7, [r1]			@Carga en el vector de salida
		    add r0, 2             @ Pasa a la siguiente palabra/elemento en vector IN.
		    add r1, 2			  @ Pasa a la siguiente palabra/elemento en vector OUT.
		    subs r2, 1            @ Resta 1 a la cantidad de elementos y actualiza flags de estado del procesador. En particular, estamos interesados en el flag Z.
		    bne asm_filtroVentana10_bucle@ La instrucción se ejecuta mientras el resultado de r1 - 1 no sea cero (Z == 0).
	  	pop {r3-r8}			  @ recuperar r3-r8 del stack
	    bx lr                 @ La instrucción de salto no se ejecutó porque Z == 1, volvemos a quién nos llamó.

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   void pack32to16 (int32_t * vectorIn, int16_t *vectorOut, uint32_t longitud)
@
@ Parametros recibidos:
@   r0: dirección de memoria de inicio del vector IN
@   r1: dirección de memoria de inicio del vector OUT
@   r2: longitud
.thumb_func               @ Se avisa al emsablador que esta es una funcion
    asm_pack16:            @ Etiqueta/nombre de la función.
        push {r4-r5}	   @ salvar el contexto de r4-r5 en el stack
        asm_pack16_bucle:  @ Etiqueta de comienzo de bucle.
	        ldr r4, [r0]		  @ Guarda elemento del venctor IN en r4
	        lsr r4, 4		  	  @ desplazo 4 a la derecha (parte alta)
	        mov r5,#0xffff		  @ guardo todos 1
	        and r5,r4,r5		  @ hago and
		    strh r5, [r1]         @ Asigna r5 a la palabra en dirección r1 (vector OUT).
		    add r0, 4             @ Pasa a la siguiente palabra/elemento en vector IN.
		    add r1, 2			  @ Pasa a la siguiente palabra/elemento en vector OUT.
		    subs r2, 1            @ Resta 1 a la cantidad de elementos y actualiza flags de estado del procesador. En particular, estamos interesados en el flag Z.
		    bne asm_pack16_bucle  @ La instrucción se ejecuta mientras el resultado de r1 - 1 no sea cero (Z == 0).
	  	pop {r4-r5}			  @ recuperar r4-r5 del stack
	    bx lr                 @ La instrucción de salto no se ejecutó porque Z == 1, volvemos a quién nos llamó.

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   int32_t max (int32_t * vectorIn, uint32_t longitud)
@
@ Parametros recibidos:
@   r0: dirección de memoria de inicio del vector IN
@   r1: longitud
.thumb_func               @ Se avisa al emsablador que esta es una funcion
    asm_max:              @ Etiqueta/nombre de la función.
        push {r4-r7}	  @ salvar el contexto de r4-r7 en el stack
        mov r5, #0
        mov r7, r1		  @ Copio longitud en r7
        asm_max_bucle:    @ Etiqueta de comienzo de bucle.
	        ldr r4, [r0]	  @ Guarda elemento del venctor IN en r4
	        subs r5, r4       @ Check si el elemento actual es mayor que el anterior
			ITT MI
			movmi r5, r4
			subsmi r6, r1, r7	@Obtengo el indice del elemento mayor

		    add r0, 4             @ Pasa a la siguiente palabra/elemento en vector IN.
		    subs r1, 1            @ Resta 1 a la cantidad de elementos y actualiza flags de estado del procesador. En particular, estamos interesados en el flag Z.
		    bne asm_max_bucle     @ La instrucción se ejecuta mientras el resultado de r1 - 1 no sea cero (Z == 0).
	  	mov r0, r6			  @ Retorno valor de salida en r0
	  	pop {r4-r7}			  @ recuperar r4-r7 del stack
	    bx lr                 @ La instrucción de salto no se ejecutó porque Z == 1, volvemos a quién nos llamó.


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   void downsampleM (int32_t * vectorIn, int32_t * vectorOut, uint32_t longitud, uint32_t N)
@
@ Parametros recibidos:
@   r0: dirección de memoria de inicio del vector IN
@   r1: direccion de memoria de inicio del vector Out
@	r2: longitud del vector
@	r3: N

.thumb_func
	asm_downsampleM:
		push {r4-r5}		@salvo el contexto
		mov r4, r3			@ r4 = N

		@ valido los argumentos de entrada
		cmp r0, #0	
		itt eq
		popeq {r4-r5}
		bxeq lr

		cmp r1, #0	
		itt eq
		popeq {r4-r5}
		bxeq lr

		cmp r2, #0	
		itt eq
		popeq {r4-r5}
		bxeq lr

		cmp r3, #0	
		itt eq
		popeq {r4-r5}
		bxeq lr

		asm_compare_loop:

			cmp r4, #0		@comparo el valor de N para verificar si se dejo pasar la cantidad de datos
			ittt eq			@if (r4 == 0) then -> reset r4=N, i++, longitud --
			moveq r4, r3
			addeq r0, #4
			subeq r2, #1

			beq asm_compare_loop

			ldr r5, [r0]	@copio Vin[i] en Vout[j]
			str r5, [r1]
			add r0, #4		@i++
			add r1, #4		@j++
			sub r4, #1		@N--
			sub r2, #1		@longitud--
			cmp r2, #0		@ if (longitud == 0) salgo del loop 
			bne asm_compare_loop
		pop {r4-r5}
		bx lr

@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@   void invertir (uint16_t * vector, uint32_t longitud)
@
@ Parametros recibidos:
@   r0: dirección de memoria de inicio del vector
@	r1: longitud del vector
@   r2: &vector[longitud-1]
@   r3: aux
.thumb_func
	asm_invertir:

		push {r4-r5}

		mov r2, #2 @calculo la direccion del ultimo valor del vector
		mul r2, r1
		add r2, r0
		sub r2, #2
		lsr r1, #1

		invertir_loop:
			ldrh r3, [r0] @aux1 = vector[i]
			ldrh r4, [r2] @aux2 = vector[longitud-1]
			strh r4, [r0], #2 @vector[i] = aux2
			strh r3, [r2], -2	@vector[i+1]
			subs r1, #1			@longitud--
			bne invertir_loop

		pop {r4-r5}
		bx lr


@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
@ Prototipo en "C":
@	void eco(int16_t* vectorIn, int16_t* vectorOut)
@
@ Parametros recibidos:
@   r0: dirección de memoria de inicio del vector de Entrada
@   r1: dirección de memoria de inicio del vector de Salida
@	r2: Longitud
@	r3: Registro para el guardado de los elementos del vector
@	r4: Indice para recorrer el vector de entrada
@	r5: Auxliar para la division signada y para guardar el valor
@	r6: Registro para guardar la primer posicion del vector de entrada
.thumb_func
	asm_eco:
		push {r4-r6}				@ Salvo el contexto
		mov r6, r0					@ Guardo el la posicion de memoria del inicio del vector de entrada
		mov r5, #882				@ Muevo el valor #882 al registro r5
		mov r4, #0					@ Muevo el valor #0 al registro r4
		asm_eco_primerbucle:
			ldr r3, [r0], #4		@ Cargo de memoria los 2 datos signados de 16 bits de la posicion de r0 al registro r3 y le sumo 4 a r0
			str r3, [r1], #4		@ Guardo en memoria los 2 datos signados de 16 bits del registro r3 en la posicion de memoria apuntada por r1 y le sumo 4 a r1
			add r4, #2				@ Sumo 2 a r4
			cmp r4, r5				@ Comparo r4 con r5 y actualizo los flags del procesador
			bne asm_eco_primerbucle	@ Si la anterior operacion no dio 0 salto a la etiqueta asm_eco_primerbucle
		mov r5, #0					@ Muevo el valor #0 a r5
		asm_eco_segundobucle:
			ldr r3, [r6], #4		@ Cargo en r3 los 2 datos signados apuntados en r6 y le sumo a r6 el valor #4
			shadd16 r3, r3, r5		@ Hago la suma signada de 2 numeros de 16 bits entre r3 y r5 y divido los resultados por 2. Guardo el resultado en r3
			ldr r5, [r0], #4		@ Cargo en r5 los 2 datos signados apuntados en r0 y le sumo a r0 el valor #4
			sadd16 r3, r3, r5		@ Hago la suma signada de 2 numeros de 16 bits entre r3 y r5 y guardo el valor en r3
			mov r5, #0				@ Muevo el valor #0 a r5
			add r4, #2				@ Sumo 2 a r4
			str r3, [r1], #4		@ Guardo en memorio los 2 datos signados de 16 bits de r3 en la posicion de r1 y le sumo 4 a r1
			cmp r4, r2				@ Comparo r4 y r2 y actualizo los flags del procesador
			bne asm_eco_segundobucle@ si la anterior operacion no dio 0 salto a la etiqueta asm_eco_segundobucle
		pop {r4-r6}					@ Retiro del stack los valores anterior de r4, r5 y r6
		bx lr						@ Vuelvo al programa principal
		

