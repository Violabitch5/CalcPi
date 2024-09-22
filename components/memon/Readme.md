## Usage Requirements:
- platformIO CLI:
    - pio run -t menuconfig
        - Component config -> FreeRTOS -> Kernel
        - Set "configGNERATE_RUN_TIME_STATS"
        - Set "Enable display of xCoreID in vTaskList"