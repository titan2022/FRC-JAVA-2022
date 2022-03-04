package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// I wrote this as a wrapper to prevent potential merge conflicts
public class OdometrySubsystemWrapper extends SubsystemBase {
    private final int x;
    private final int y;
    private final int rot;

    public OdometrySubsystemWrapper(int startX, int startY, int startRot) {
        this.x = startX;
        this.y = startY;
        this.rot = startRot;
    }
    public OdometrySubsystemWrapper(int startX, int startY) {
        this.x = startX;
        this.y = startY;
        // TODO: initialize with value from Gyroscope
        this.rot = 0;
    }

    /**
     * 
     * @return absolute [x position, y position, rotation]
     */
    public int[] getPos() {
        return new int[] {x, y, rot};
    }

    // This is what I think the command might look like
    public void update(int deltaX, int deltaY, int deltaRot) {
        
    }
}
