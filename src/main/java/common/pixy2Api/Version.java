package common.pixy2Api;

public class Version {

    protected int hardware = 0;
    protected int firmwareMajor = 0;
    protected int firmwareMinor = 0;
    protected int firmwareBuild = 0;
    protected char[] firmwareType = new char[10];

    /**
     * Constructs version object with given buffer of version data
     * 
     * @param version Buffer output from Pixy2 containing version data
     */
    public Version(byte[] version) {
        hardware = ((int) (version[1] & 0xff) << 8) | (int) (version[0] & 0xff);
        firmwareMajor = version[2];
        firmwareMinor = version[3];
        firmwareBuild = ((int) (version[5] & 0xff) << 8) | (int) (version[4] & 0xff);
        for (int i = 0; i < 10; i++) {
            firmwareType[i] = (char) (version[i + 6] & 0xFF);
        }
    }

    /**
     * Prints version data to console
     */
    public void print() {
        System.out.println(toString());
    }

    /**
     * Returns a string of version data
     * 
     * @return String of version data;
     */
    public String toString() {
        return "hardware ver: 0x" + hardware + " firmware ver: " + firmwareMajor + "." + firmwareMinor + "."
                + firmwareBuild + " " + new String(firmwareType);
    }

    /**
     * Gets Pixy2 Hardware Version
     * 
     * @return Pixy2 Hardware Version
     */
    public int getHardware() {
        return hardware;
    }

    /**
     * Gets Pixy2 Firmware Version Major
     * 
     * @return Pixy2 Firmware Version Major
     */
    public int getFirmwareMajor() {
        return firmwareMajor;
    }

    /**
     * Gets Pixy2 Firmware Version Minor
     * 
     * @return Pixy2 Firmware Version Minor
     */
    public int getFirmwareMinor() {
        return firmwareMinor;
    }

    /**
     * Gets Pixy2 Firmware Version Build
     * 
     * @return Pixy2 Firmware Version Build
     */
    public int getFirmwareBuild() {
        return firmwareBuild;
    }

    /**
     * Gets Pixy2 Firmware Type
     * 
     * @return Pixy2 Firmware Type
     */
    public char[] getfirmwareType() {
        return firmwareType;
    }

    /**
     * Gets Pixy2 Firmware Type
     * 
     * @return Pixy2 Firmware Type
     */
    public String getFirmwareTypeString() {
        return new String(firmwareType);
    }
}
