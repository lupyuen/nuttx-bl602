# SPI Test Driver

Read the article...

-   ["SPI on Apache NuttX OS"](https://lupyuen.github.io/articles/spi2)

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

1.  Call `spi_test_driver_register` in `bl602_bringup` (or `esp32_bringup`)

    In menuconfig...
    
    -   Enable the SPI Port (SPI0)

    -   Enable the SPI Character Driver

    -   Enable the SPI Test Driver

    As explained here...

    ["Load the SPI Test Driver"](https://lupyuen.github.io/articles/spi2#load-the-spi-test-driver)

1.  The SPI Test Driver is accessible by NuttX Apps as "`/dev/spitest0`"
