# LSM6DSV16X
Arduino library to support the LSM6DSV16X 3D accelerometer and 3D gyroscope

## API

This sensor uses I2C, I3C or SPI to communicate.
For I2C it is then required to create a TwoWire interface before accessing to the sensors:  

    TwoWire dev_i2c(I2C_SDA, I2C_SCL);  
    dev_i2c.begin();

For SPI it is then required to create a SPI interface before accessing to the sensors:  

    SPIClass dev_spi(SPI_MOSI, SPI_MISO, SPI_SCK);  
    dev_spi.begin();

For I3C it is then required to create an I3C interface before accessing to the sensors:

    I3C.begin(I3C_SDA, I3C_SCL, 1000000U);

An instance can be created and enabled when the I2C bus is used following the procedure below:  

    LSM6DSV16XSensor AccGyr(&dev_i2c);
    AccGyr.begin();
    AccGyr.Enable_X();  
    AccGyr.Enable_G();

An instance can be created and enabled when the SPI bus is used following the procedure below:  

    LSM6DSV16XSensor AccGyr(&dev_spi, CS_PIN);
    AccGyr.begin();	
    AccGyr.Enable_X();  
    AccGyr.Enable_G();


An instance can be created and enabled when the I3C bus is used with SETDASA (static-to-dynamic address assignment):  

    LSM6DSV16XSensor AccGyr(&I3C, LSM6DSV16X_I3C_ADD_H);
    I3C.resetDynamicAddresses();
    I3C.assignDynamicAddress(AccGyr.getStaticAddress(), LSM6DSV16X_DYNAMIC_ADDRESS);
    AccGyr.begin(LSM6DSV16X_DYNAMIC_ADDRESS);
    I3C.setClock(12500000);
    AccGyr.Enable_X();
    AccGyr.Enable_G();

An instance can be created and enabled when the I3C bus is used with ENTDAA (dynamic address discovery):  

    LSM6DSV16XSensor AccGyr(&I3C);
    I3C.begin(I3C_SDA, I3C_SCL, 1000000U);
    I3C.discover(devices, 8, &found);
    // find dynAddr by matching LSM6DSV16X_I3C_PID_H in discovered devices
    AccGyr.begin(dynAddr);
    I3C.setClock(12500000);
    AccGyr.Enable_X();
    AccGyr.Enable_G();

The access to the sensor values is done as explained below:  

  Read accelerometer and gyroscope.

    int32_t accelerometer[3];
    int32_t gyroscope[3];
    AccGyr.Get_X_Axes(accelerometer);  
    AccGyr.Get_G_Axes(gyroscope);

## Examples

* LSM6DSV16X_DataLog_Terminal_I2C: This application shows how to get data from LSM6DSV16X accelerometer and gyroscope and print them on terminal over I2C.

* LSM6DSV16X_6D_Orientation_I2C: This application shows how to use LSM6DSV16X accelerometer to find out the 6D orientation and display data on a hyperterminal over I2C.

* LSM6DSV16X_Datalog_Terminal_I3C: This application shows how to use LSM6DSV16X accelerometer and gyroscope over I3C using SETDASA.

* LSM6DSV16X_Datalog_Terminal_I3C_ENTDAA: This application shows how to discover and use LSM6DSV16X dynamic address over I3C.

* LSM6DSV16X_Double_Tap_Detection_I2C: This application shows how to detect the double tap event using the LSM6DSV16X accelerometer over I2C.

* LSM6DSV16X_Free_Fall_Detection_I2C: This application shows how to detect the free fall event using the LSM6DSV16X accelerometer over I2C.

* LSM6DSV16X_MLC_I2C: This application shows how to detect the activity using the LSM6DSV16X Machine Learning Core over I2C.

* LSM6DSV16X_Pedometer_I2C: This application shows how to use LSM6DSV16X accelerometer to count steps over I2C.

* LSM6DSV16X_Qvar_Polling_I2C: This application shows how to use LSM6DSV16X Qvar features in polling mode over I2C.

* LSM6DSV16X_Sensor_Fusion_I2C: This application shows how to use LSM6DSV16X Sensor Fusion features for reading quaternions over I2C.

* LSM6DSV16X_Single_Tap_Detection_I2C: This application shows how to detect the single tap event using the LSM6DSV16X accelerometer over I2C.

* LSM6DSV16X_Tilt_Detection_I2C: This application shows how to detect the tilt event using the LSM6DSV16X accelerometer over I2C.

* LSM6DSV16X_Wake_Up_Detection_I2C: This application shows how to detect the wake-up event using the LSM6DSV16X accelerometer over I2C.

* LSM6DSV16X_FIFO_Polling_I2C: This application shows how to get accelerometer and gyroscope data from FIFO in pooling mode and print them on terminal over I2C.

* LSM6DSV16X_FIFO_Interrupt_I2C: This application shows how to get accelerometer and gyroscope data from FIFO using interrupt and print them on terminal over I2C.
## Documentation

You can find the source files at  
https://github.com/stm32duino/LSM6DSV16X

The LSM6DSV16X datasheet is available at  
https://www.st.com/content/st_com/en/products/mems-and-sensors/inemo-inertial-modules/lsm6dsv16x.html