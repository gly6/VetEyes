# Pi_Eyes

Username and Password for the Raspberry Pi 3 is ns948 

Username and Password for the Raspberry Pi 4 is pi and veteyes I think... if that doesn't work I would just reimage it anyways since I think I fuck up something

To run the eyes

    - "make start" to have the eyes run 
    
Setup 

    - The buttons should be connected to the GPIO pins (4,17,18,27,22)
    
    - The multiplexer should be connected to the I2C of the Raspberry Pi
    
        - The light sensor should be connected to the multiplexer SC0/SD0 and SC1/SD1 
        
        - at the moment light sensor is commented out (CTRL - F uncomment to find the lines to uncomment when testing the lux sensor) 
    
