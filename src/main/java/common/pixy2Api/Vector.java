package common.pixy2Api;

public class Vector {
    private double x0, y0, x1, y1 = 0;
    private int index, flags = 0;

    /**
     * Constructs Vector instance
     * 
     * @param x0    X0 value
     * @param y0    Y0 value
     * @param x1    X1 value
     * @param y1    Y1 value
     * @param index Vector index
     * @param flags Vector flags
     */
    public Vector(int x0, int y0, int x1, int y1, int index, int flags) {
        this.x0 = x0;
        this.y0 = y0;
        this.x1 = x1;
        this.y1 = y1;
        this.index = index;
        this.flags = flags;
    }

    /**
     * Constructs Vector instance
     * 
     * @param x0    X0 value
     * @param y0    Y0 value
     * @param x1    X1 value
     * @param y1    Y1 value
     * @param index Vector index
     * @param flags Vector flags
     */
    public Vector(double x0, double y0, double x1, double y1, int index, int flags) {
        this.x0 = x0;
        this.y0 = y0;
        this.x1 = x1;
        this.y1 = y1;
        this.index = index;
        this.flags = flags;
    }

    /**
     * Prints vector data to console
     */
    public void print() {
        System.out.println(toString());
    }

    /**
     * Returns a string of vector data
     * 
     * @return String of vector data
     */
    public String toString() {
        return "vector: (" + x0 + " " + y0 + ") (" + x1 + " " + y1 + ") index: " + index + " flags: " + flags;
    }

    /**
     * @return X0 value
     */
    public double getX0() {
        return x0;
    }

    /**
     * @return Y0 value
     */
    public double getY0() {
        return y0;
    }

    /**
     * @return X1 value
     */
    public double getX1() {
        return x1;
    }

    /**
     * @return Y1 value
     */
    public double getY1() {
        return y1;
    }

    /**
     * @return Vector index
     */
    public int getIndex() {
        return index;
    }

    /**
     * @return Vector flags
     */
    public int getFlags() {
        return flags;
    }
}
