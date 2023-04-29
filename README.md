# AdM_CESE
Repositorio para la materia Arquitectura de Microprocesadores

# Preguntas orientadoras
1. Describa brevemente los diferentes perfiles de familias de
microprocesadores/microcontroladores de ARM. Explique alguna de sus diferencias
características.

- **Perfil A**: *Diseñado para plataformas que ejecuten aplicaciones (por ej. SOs: iOS, Windows, Android, Linux, etc). Estas aplicaciones requieren la máxima capacidad de procesamiento posible y soporte para memoria virtual (MMU). Tambien tienen memoria cache, esto le quita determinismo al programa ya que no sabemos cuanto va a tardar el microcontrolador en buscar un dato de la memoria. Algunos ejemplos de productos que pueden utilizar este tipo de procesador son los smartphones, tablets, smart-tv, etc. Algo a tener en cuenta en estos casos es que no se necesita cumplir con exigencias de tiempo real. Este perfil tiene una arquitectura ARMv7-A.*
- **Perfil R**: *Diseñado para sistemas embebidos del tipo high-end en los que se necesite performance de tiempo real (por ej: controladores de disco duro, sistemas autónomos, etc.). En estos sistemas la baja latencia y el determinismo son lo más importante, por lo que no tiene cache. Es como un cortex A pero sin las caracteristicas que danan el determinismo. Este perfil tiene una arquitectura ARMv7-R.*
- **Perfil M**: *Diseñado para sistemas del tipo microcontrolador, donde interesa cumplir con una seria de requisitos tales como el bajo costo, bajo consumo de energía, baja latencia de interrupciones. Al mismo tiempo se espera obtener un comportamiento determinístico. Este perfil tiene una arquitectura ARMv7-M y ARMv6-M.*

# Cortex M
1. Describa brevemente las diferencias entre las familias de procesadores Cortex M0, M3 y
M4.

Para describir las diferencias entre las distintas familias de procesadores se tuvieron en cuentas los siguientes aspectos:
- Set de instrucciones: Los Cortex-M0 y Cortex-M0+ soportan 56 instrucciones, la mayoría de 16 bits. Los cortex-M3 soportan más de 100 instrucciones y los cortex-M4 además tienen instrucciones de DSP y opcionalmente de punto flotante.
- Complejidad de la arquitectura: Los Cortex-M0 son los procesadores más pequeños (contienen aproximadamente 12000 compuertas lógicas). Los Cortex-M0+ son similares a los M0 pero con mejor eficiencia energética. Los Cortex-M3 y Cortex-M4 tienen mejores características de sistema y de debug, pero con una mayor complejidad, lo que implica un mayor número de compuertas lógicas (aproximadamente 40000).
- Soporte para interrupciones: Los Cortex-M0 y Cortex-M0+ soportan hasta 32 interrupciones con 4 niveles de prioridades, mientras que los Cortex-M3 y Cortex-M4 soportan hasta 240 interrupciones con hasta 256 niveles de prioridades.
- FPU: Los Cortex-M4 tienen soporte opcional para FPU de precisión simple.
- MPU: Los Cortex-M0 no tienen soporte para protección de memoria, mientras que los Cortex-M0+, Cortex-M3 y Cortex-M4 tienen la opción de incorporar MPU.

2. ¿Por qué se dice que el set de instrucciones Thumb permite mayor densidad de código?
Explique

- Porque al ser instrucciones de 16 bits (Thumb), se necesita menos memoria para realizar una misma tarea que con instrucciones de 32 bits (ARM). Con los cortex-M aparece Thumb2, donde se mezclan instrucciones de 16 bits con instrucciones de 32 bits. Quien define en que tamano se codifican las instrucciones? El assembler es el que decide como se codifican en base a como utilizamos las instrucciones. Debido a que se pueden usar instrucciones de 32 bits es que tambien se puede hacer mas de una operacion por cada instruccion.
