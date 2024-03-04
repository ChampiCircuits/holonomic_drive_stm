# Holonomic Drive STM

## Linker Error With CLion

Erreur:
```shell
/opt/st/stm32cubeclt/GNU-tools-for-STM32/bin/../lib/gcc/arm-none-eabi/11.3.1/../../../../arm-none-eabi/bin/ld: read in flex scanner failed
collect2: error: ld returned 1 exit status
```

Dans le fichier CMakelists.txt généré par CLion, trouver la ligne suivante:

```cmake
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/)
```
et la remplacer par:
```cmake
set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/STM32G431KBTX_FLASH.ld)
```