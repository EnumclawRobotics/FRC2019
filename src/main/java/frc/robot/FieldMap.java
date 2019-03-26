
package frc.robot;

// important field numbers and environment specifics. 
public class FieldMap {

    // colors / constants
    public final static int cargoColorNumber = 10;                        // MR Color Sensor number for orange
    public final static int cargoColorNumberVariance = 1;                 // +- amount around color number that is still cargo color

    //public final static int[] cargoColor = new int[] { 0, 0, 0, 0 };     // TODO: Fill in accurate color values 
    //public final static double cargoColorThreshold = .1d;

    // field geometries (in inches)
    public final static double whiteLineLength = 18;

    // public final static double heightRocketHatch1 = 19;
    // public final static double heightRocketHatch2 = 47;
    // public final static double heightRocketHatch3 = 75;

    // public final static double heightRocketCargo1 = 27.5;
    // public final static double heightRocketCargo2 = 55.5;
    // public final static double heightRocketCargo3 = 83.5;

    // public final static double heightFloorCargo = 9.250;             // requires wrist to be straight with arm instead of held level

    // public final static double heightStationHatch = 19;
    // public final static double heightStationCargo = 44.125;

    // public final static double heightShipHatch = 19;
    // public final static double heightShipCargo = 38.75;

    // hab geometries
    public final static double heightHabLevel1 = 3; 
    public final static double heightHabLevel2 = 6; //Based off of hab 1 as the ground
    public final static double heightHabLevel3 = 19; //Based off of hab 1 as the ground

    // game piece geometries
    public final static double hatchDiameter = 19;
    public final static double hatchHoleDiameter = 6;
    public final static double cargoDiameter = 13;
}
