package common.pixy2Api;

public class IntersectionLine {

    private int index, reserved = 0;
    private short angle = 0;

    /**
     * Constructs IntersectionLine object
     * 
     * @param index    IntersectionLine index
     * @param reserved Reserved
     * @param angle    Line angle
     */
    public IntersectionLine(int index, int reserved, short angle) {
        this.index = index;
        this.reserved = reserved;
        this.angle = angle;
    }

    /**
     * @return IntersectionLine index
     */
    public int getIndex() {
        return index;
    }

    /**
     * @return Reserved
     */
    public int getReserved() {
        return reserved;
    }

    /**
     * @return Line angle
     */
    public short getAngle() {
        return angle;
    }

}
