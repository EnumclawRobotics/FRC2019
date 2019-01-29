package common.instrumentation;

import edu.wpi.first.networktables.*;

public class Telemetry {
    NetworkTable networkTable; 

    public Telemetry(String tablePath) {
        networkTable = NetworkTableInstance.getDefault().getTable(tablePath); 
    }

    public void putString(String label, String value) {
        networkTable.getEntry(label).setString(value);
    }

    public void putDouble(String label, Double value) {
        networkTable.getEntry(label).setDouble(value);
    }

    public void putDouble(String label, Integer value) {
        networkTable.getEntry(label).setDouble(value + 0d);
    }

    public void putDouble(String label, Float value) {
        networkTable.getEntry(label).setDouble(value + 0d);
    }

    public void putBoolean(String label, Boolean value) {
        networkTable.getEntry(label).setBoolean(value);
    }

    public String getString(String label) {
        return networkTable.getEntry(label).getString("");
    }

    public Double getDouble(String label) {
        return networkTable.getEntry(label).getDouble(0.00d);
    }    
 
    public Boolean getBoolean(String label) {
        return networkTable.getEntry(label).getBoolean(false);
    }
}