
package frc.robot;

// important field numbers and environment specifics. 
public class FieldMap {

  // colors / constants
  public final static int[] cargoColor = new int[] { 0, 0, 0, 0 };     // TODO: Fill in accurate color values 
  public final static double cargoColorThreshold = .1d;

  // arm geometries (in inches)
  public final static double lengthArm = 38; 
  public final static double heightArmPivot = 45;

  // field geometries
  public final static double whiteLineLength = 18;

  public final static double heightHatchRocket1 = 19;
  public final static double heightHatchRocket2 = 47;
  public final static double heightHatchRocket3 = 75;
  
  public final static double heightCargoRocket1 = 27.5;
  public final static double heightCargoRocket2 = 55.5;
  public final static double heightCargoRocket3 = 83.5;
  
  public final static double heightCargoFloor = 9.250;             // requires wrist to be straight with arm instead of held level
  public final static double heightHatchStation = 19;
  public final static double heightCargoStation = 44.125;
  
  public final static double heightCargoShip = 38.75;
  public final static double heightHatchShip = 19;
  
  // hab geometries
  public final static double heightHabLevel1 = 3; 
  public final static double heightHabLevel2 = 6;
  public final static double heightHabLevel3 = 19;

  // game piece geometries
  public final static double hatchDiameter = 19;
  public final static double hatchHoleDiameter = 6;
  public final static double cargoDiameter = 13;

}
