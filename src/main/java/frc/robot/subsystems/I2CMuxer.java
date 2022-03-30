package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;

/**
 * Wrapper for the TCA9548A I2C Multiplexer
 */
public class I2CMuxer extends I2C {
    public I2CMuxer() {
        //Address is 0X70 for the muxer
        super(I2C.Port.kOnboard, 0X70);
    }       

    /**
     * Sends a byte representing the id of the port to select
     * @param id Port id starting from 0 and going to 7
     */
    public void setPort(byte id) {
        this.write(0X70, 1 << id + 1);    
    }
}
