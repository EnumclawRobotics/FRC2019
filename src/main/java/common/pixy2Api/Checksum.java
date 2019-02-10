package common.pixy2Api;

public class Checksum {

    int cs = 0;

    public void updateChecksum(int b) {
        cs += b;
    }

    public int getChecksum() {
        return cs;
    }

    public void reset() {
        cs = 0;
    }

}
