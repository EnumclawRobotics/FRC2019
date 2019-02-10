package common.pixy2Api;

public class Intersection {

    private int x, y, number, reserved = 0;
    private IntersectionLine[] lines = new IntersectionLine[Pixy2Line.LINE_MAX_INTERSECTION_LINES];

    /**
     * Constructs Intersection object
     * 
     * @param x        X value
     * @param y        Y value
     * @param number   Number of lines
     * @param reserved Reserved
     * @param lines    Array of lines
     */
    public Intersection(int x, int y, int number, int reserved, IntersectionLine[] lines) {
        this.x = x;
        this.y = y;
        this.number = number;
        this.reserved = reserved;
        this.lines = lines;
    }

    /**
     * Prints intersection data to console
     */
    public void print() {
        System.out.println("intersection: (" + x + " " + y + ")");
        for (int i = 0; i < lines.length; i++) {
            IntersectionLine line = lines[i];
            System.out.println(" " + i + " index: " + line.getIndex() + " angle: " + line.getAngle());
        }
    }

    /**
     * @return X value
     */
    public int getX() {
        return x;
    }

    /**
     * @return Y value
     */
    public int getY() {
        return y;
    }

    /**
     * @return Number of lines
     */
    public int getNumber() {
        return number;
    }

    /**
     * @return Reserved
     */
    public int getReserved() {
        return reserved;
    }

    /**
     * @return Array of lines
     */
    public IntersectionLine[] getLines() {
        return lines;
    }

}
