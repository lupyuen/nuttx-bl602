# SPI Test Driver

Read the articles...

-   ["SPI on Apache NuttX OS"](https://lupyuen.github.io/articles/spi2)

-   ["LoRa SX1262 on Apache NuttX OS"](https://lupyuen.github.io/articles/sx1262)

-   ["LoRaWAN on Apache NuttX OS"](https://lupyuen.github.io/articles/lorawan3)

To install the SPI Test Driver in your NuttX project...

1.  Copy the files...

    -   [`Kconfig`](Kconfig)
    
    -   [`spi_test_driver.c`](spi_test_driver.c)

    To the folder `nuttx/drivers/rf`

1.  Update the NuttX Build Config...

    ```bash
    ## TODO: Change this to the path of our "incubator-nuttx" folder
    cd nuttx/nuttx

    ## Preserve the Build Config
    cp .config ../config

    ## Erase the Build Config and Kconfig files
    make distclean

    ## For BL602: Configure the build for BL602
    ./tools/configure.sh bl602evb:nsh

    ## For ESP32: Configure the build for ESP32.
    ## TODO: Change "esp32-devkitc" to our ESP32 board.
    ./tools/configure.sh esp32-devkitc:nsh

    ## Restore the Build Config
    cp ../config .config

    ## Edit the Build Config
    make menuconfig 
    ```

1.  In menuconfig...
    
    -   Enable the SPI Port (SPI0)

    -   Enable the SPI Character Driver

    -   Enable the SPI Test Driver

    As explained here...

    ["Load the SPI Test Driver"](https://lupyuen.github.io/articles/spi2#load-the-spi-test-driver)

1.  Edit the function `bl602_bringup` or `esp32_bringup` in this file...

    ```text
    ## For BL602:
    nuttx/boards/risc-v/bl602/bl602evb/src/bl602_bringup.c

    ## For ESP32: Change "esp32-devkitc" to our ESP32 board 
    nuttx/boards/xtensa/esp32/esp32-devkitc/src/esp32_bringup.c
    ```

    And call `spi_test_driver_register` to register our SPI Test Driver.
    
    [(See this)](https://lupyuen.github.io/articles/spi2#register-device-driver)

1.  The SPI Test Driver is accessible by NuttX Apps as "`/dev/spitest0`"
