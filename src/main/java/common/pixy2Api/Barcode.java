package common.pixy2Api;

public class Barcode {
    private int x, y, flags, code = 0;

    /**
        * Constructs barcode object
        * 
        * @param x     X value
        * @param y     Y value
        * @param flags Barcode flags
        * @param code  Code
        */

        public  Barcode(int x, int y, int flags, int code) {
        this.x = x;
        this.y = y;
        this.flags = flags;
        this.code = code;
    }

    /**
        * Prints barcode data to console
        */
    public void print() {
        System.out.println(toString());
    }

    /**
        * Returns a string of barcode data
        * 
        * @return String of barcode data
        */
    public String toString() {
        return "barcode: (" + x + " " + y + ") value: " + code + " flags: " + flags;
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
        * @return Barcode flags
        */
    public int getFlags() {
        return flags;
    }

    /**
        * @return Code
        */
    public int getCode() {
        return code;
    }

}
