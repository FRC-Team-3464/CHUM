// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.Map;

import javax.lang.model.type.NullType;

import com.fasterxml.jackson.annotation.ObjectIdGenerators.None;

import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  public static LEDSubsystem instance;
  public static Boolean reversed;


  private final AddressableLED ledStrip = new AddressableLED(1);
  private final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(20);

  public static LEDSubsystem getInstance() {
    if (instance == null) {
      instance = new LEDSubsystem();
    }
    return instance;
  }
  
  public LEDSubsystem() {
    // four score and seven years ago.......
    ledStrip.setLength(ledBuffer.getLength());
    ledStrip.setData(ledBuffer);
    ledStrip.start();
    if (reversed == null) {
      reversed = false;
    }
  }

  public void setOff() {
    LEDPattern off = LEDPattern.kOff;
    off.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  public void redLED() {
    // basically this makes it red ahaha
    LEDPattern red = LEDPattern.solid(Color.kRed);
    red.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  public void redorangeLED() {
    // basically this makes it rorange ahaha
    LEDPattern rorange = LEDPattern.solid(Color.kOrangeRed);
    rorange.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  public void orangeLED() {
    // basically this makes it orange ahaha
    LEDPattern orange = LEDPattern.solid(Color.kOrange);
    orange.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  public void goldLED() {
    // basically this makes it yellow ahaha
    LEDPattern gold = LEDPattern.solid(Color.kGold);
    gold.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }
  
  public void ghostwhiteLED() {
    // basically this makes it grenyell ahaha
    LEDPattern ghost = LEDPattern.solid(Color.kGhostWhite);
    ghost.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  public void greenLED() {
    // basically this makes it green ahaha
    LEDPattern green = LEDPattern.solid(Color.kGreen);
    green.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }  
  
  public void blueLED() {
    // basically this makes it blueh ahaha
    LEDPattern blueh = LEDPattern.solid(Color.kBlue);
    blueh.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }  

  public void denimLED() {
    // basically this makes it demim ahaha
    LEDPattern demim = LEDPattern.solid(Color.kDenim);
    demim.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }  

  public void bluevioletLED() {
    // basically this makes it bluviol ahaha
    LEDPattern bluviol = LEDPattern.solid(Color.kBlueViolet);
    bluviol.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  } 

  public void rainbowLED () {
    // probs for the shooter
    // basically this makes it all of the above ahaha - micromax, 2025, also 2025 ahahah
    // guys guys guys look raibowee!!!! - Juliet Caufield, 2025
    LEDPattern rainbow = LEDPattern.rainbow(255, 128);
    Distance ledSpacing = Meters.of(1 / 120.0);
    LEDPattern scrollingRainbow =
      rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(4), ledSpacing);
    scrollingRainbow.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }
 
  public void blugrenConGradientLED () {
    // four score and fifty TRILLION years agoo... - aberhum something, 2024
    // no but really all this does is make a gradient between gren and blu - ben frankin, 3024
    LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kGreen, Color.kBlue);
    gradient.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  public void yelredConGradientLED () {
    //oh my god... NOW THE'RE YELLOW AND RED?!?!?!?!?? SUCH INNOVATION!!!! 
    LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kYellow, Color.kRed);
    gradient.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }
  
  public void redbluNonContGradLED () {
    // for when you don't want a continuous gradient... obviously
    LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlue);
    gradient.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
  }

  // Create an LED pattern that displays a red-to-blue gradient, breathing at a 2 second period (0.5 Hz)
  public void climbersLED(){
    //LEDPattern base = LEDPattern.discontinuousGradient(Color.kRed, Color.kYellow);
    LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kLightCoral, Color.kGreen);
    LEDPattern pattern = base.breathe(Microseconds.of(1500000));
    // Apply the LED pattern to the data buffer
    pattern.applyTo(ledBuffer);
    // Write the data to the LED strip
    ledStrip.setData(ledBuffer);
  }

  // This will create a LED line that shows the level of the elevators but for now since its a branch it will show red lines
// public void elevatorSequence(){
// LEDPattern pattern = LEDPattern.progressMaskLayer(() -> elevatorHeight.getY() / maxElevatorHeight.getY());
// pattern.applyTo(ledBuffer);
// ledStrip.setData(ledBuffer);
// }

// // This is line of code is for when the Robot encounters a issue, it blinks
public void warningPulse(){
// Create an LED pattern that displays a red-to-blue gradient, blinking at various rates.
LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kCrimson);
LEDPattern pattern = base.blink(Microseconds.of(300000));
LEDPattern asymmetric = base.blink(Microseconds.of(100000), Microseconds.of(100000));
LEDPattern sycned = base.synchronizedBlink(RobotController::getRSLState);
pattern.applyTo(ledBuffer);
ledStrip.setData(ledBuffer);
}

// // When the robot is picking up coral/algae (whatever it is) it will load with the scroll code here
public void intakeLED(){
// For a half-meter length of a 120 LED-per-meter strip, this is equivalent to scrolling at 12.5 centimeters per second.
Distance ledSpacing = Meters.of(1 / 120.0);
// Note if the WPI Library gives you a Discontinuous, switch the GradientType to either GradientType.kDiscontinuous or GradientType.kContinuous
LEDPattern base = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kOlive);
LEDPattern pattern = base.scrollAtRelativeSpeed(Percent.per(Microseconds).of(50));
LEDPattern absolute = base.scrollAtAbsoluteSpeed(Meters.per(Microseconds).of(47.5), ledSpacing);
pattern.applyTo(ledBuffer);
ledStrip.setData(ledBuffer);
}

public void funEffectLED(){
  Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.5, Color.kBlack);
LEDPattern base = LEDPattern.rainbow(255, 255);
LEDPattern mask =
   LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Microseconds).of(51));
LEDPattern pattern = base.mask(mask);
LEDPattern revPattern = pattern.reversed();
revPattern.applyTo(ledBuffer);
ledStrip.setData(ledBuffer);
System.out.println(ledBuffer.getRed(9));
}

public LEDPattern pattern;

public void benPeiLED(){
  Map<Double, Color> maskSteps = Map.of(0.0, Color.kWhite, 0.25, Color.kBlack);
    LEDPattern red = LEDPattern.solid(Color.kRed);
    LEDPattern mask = 
    LEDPattern.steps(maskSteps).scrollAtRelativeSpeed(Percent.per(Second).of(50));
    LEDPattern pattern = red.mask(mask);
    LEDPattern revPattern = pattern.reversed();
    double pos = ledBuffer.getRed(16);
    double pos2 = ledBuffer.getRed(3);

    if (reversed == true) {
      System.out.println("This is working");
      revPattern.applyTo(ledBuffer); 
      ledStrip.setData(ledBuffer);
      if (pos2 == 255) {
        reversed = false; 
      } 
    }
  
    else if (reversed == false) {
    pattern.applyTo(ledBuffer);
    ledStrip.setData(ledBuffer);
    
    if (pos == 255) {
      reversed = true; 
    }
  }
  

    System.out.println(ledBuffer.getRed(0));
  }
  

  //manhunt vs 4 GOVERNMENT AGENTS IRL!!
  // trust me bro it's legal <3

  @Override
  public void periodic() {
    // This method will be called once per scheduler run, whatever that means
  
  }
}

