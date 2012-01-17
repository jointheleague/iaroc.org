package org.sunspotworld;
//$Id: StartApplication.java 2522 2009-06-06 17:23:23Z vic $

/**********************************************************************
 * Vic's best effort at interfacing the Parallax ultrasonic sensor to the Create via the SPOT.
 * Utilizes Wintriss Technical Schools Roomba/SPOT interface board and Parallax Ping sensor.
 * Parallax Ping sensor wires to modular connector standard...gnd to 3, +5 to 2, signal to 1.
 * Version 1.1, March 10, 2010 Implemented Roomba waitAngle() script command.
 **********************************************************************/
import com.sun.spot.sensorboard.EDemoBoard;
import com.sun.spot.sensorboard.io.IIOPin;
import com.sun.spot.sensorboard.io.IInputPinListener;
import com.sun.spot.sensorboard.io.PinDescriptor;
import com.sun.spot.sensorboard.peripheral.ISwitch;
import com.sun.spot.sensorboard.peripheral.ITriColorLED;
import javax.microedition.midlet.MIDlet;
import javax.microedition.midlet.MIDletStateChangeException;

public class StartApplication extends MIDlet implements IIOPin
{
    protected EDemoBoard eDemoBoard = EDemoBoard.getInstance(); // The interface board on the SPOT controller.
    public Roomba georgeTheRoomba; // The iRobot Roomba Ccreate robot.
    protected ITriColorLED[] leds = EDemoBoard.getInstance().getLEDs();
    public IIOPin d3 = eDemoBoard.getIOPins()[EDemoBoard.D3]; // Input/output pins for generating and measuring pulses.
    public int sonarEchoPulseWidth;
    private ISwitch sw1 = eDemoBoard.getSwitches()[EDemoBoard.SW1];
    public int relativeHeading = 0;
    public int wheelSpeed = 25;
    private int irWallSignal;
    private int missedPulseCount = 0; // Parallax sonar.
    private int readingCount = 0; // Parallax sonar.
    int pw = 0; // Parallax sonar.

    protected void startApp() throws MIDletStateChangeException
    {
        System.out.println("Starting BestParallaxUltrasonic2010, version 1.1");
        georgeTheRoomba = new Roomba();

        /***************************************************************************************
         * Turn on lights
         ***************************************************************************************/
        leds[0].setRGB(100, 0, 0);                // set led 0 (leftmost) color to moderate red (turn left indicator)

        leds[3].setRGB(0, 0, 100);                // set led 3 color to moderate blue
        leds[3].setOn();

        leds[7].setRGB(0, 100, 0);                // set led 7 (rightmost) color to moderate green (turn right indicator)

        System.out.println("Lights should be on!");

        /***************************************************************************************
         * George moving forward at wheelSpeed while switch #1 is not pushed.
         ***************************************************************************************/
        georgeTheRoomba.driveDirect(0, wheelSpeed);// Start George moving...turning right to test waitAngle(-30).
        georgeTheRoomba.waitAngle(-30);
        georgeTheRoomba.driveDirect(wheelSpeed, wheelSpeed);
        while (sw1.isOpen()) // Push left switch to stop app.
        {
            georgeTheRoomba.readSensors(RoombaConstants.SENSORS_GROUP_ID6); // Reads all sensors into int[] sensorValues.

            /***************************************************************************************
             * Send trigger pulse and measure time for sonar to return echo (good from 1" to 40").  Getting about 20 good readings per second.
             ***************************************************************************************/
            d3.setAsOutput(true); // Prepare D3 as an output pin so that it can send out a pulse.
            eDemoBoard.startPulse(d3, true, 100); // Send out a 100 microsecond pulse on pin D3.
            d3.setAsOutput(false); // Prepare D3 as an input so that it can measure the sonar return pulse width.
            pw = eDemoBoard.getPulse(d3, true, 8); // Looks for a positive going pulse on D3 and returns its width. Waits up to 8 milliseconds for a pulse to arrive.

            if (pw == 0) // Missed reading
            {
                missedPulseCount++;
            }
            else // Assume good reading...although 54 means that things have hung up and are not getting good readings.
            {
                sonarEchoPulseWidth = pw;
                readingCount++;
            }

            /***************************************************************************************
             * Check infrarad wall sensor readings (only good out to about 2").
             ***************************************************************************************/
            irWallSignal = georgeTheRoomba.getWallSignal();

            /***************************************************************************************
             * Keep track of relative heading plus means to right
             ***************************************************************************************/
            relativeHeading -= georgeTheRoomba.getAngle();

            /***************************************************************************************
             * Check for bumps.
             ***************************************************************************************/
            boolean bumpRightSignal = georgeTheRoomba.isBumpRight();
            if (bumpRightSignal)
            {
                georgeTheRoomba.driveDirect(wheelSpeed, 0);//turn left
            }

            boolean bumpLeftSignal = georgeTheRoomba.isBumpLeft();
            if (bumpLeftSignal)
            {
                georgeTheRoomba.driveDirect(0, wheelSpeed);//turn right
            }
           
            System.out.print("missed pulses/good pulses = " + missedPulseCount + "/" + readingCount);
            System.out.println("\t Sonar distance = " + sonarEchoPulseWidth + "\t relativeHeading = " + relativeHeading + "\t IR wall signal = " + irWallSignal);
        }

        System.out.println("Somebody pushed switch #1...I quit!");
        leds[0].setOff();
        leds[3].setOff();
        leds[7].setOff();
        notifyDestroyed(); // Causes the MIDlet in the SPOT to exit.
        System.exit(0); // Causes the desktop app to quit.
    }

    protected void pauseApp() // Required by startApp()
    {
    }

    protected void destroyApp(boolean unconditional) throws MIDletStateChangeException // Required by startApp()
    {
        System.out.println("Shutting down.");
    }

    public boolean isOutput()
    {
        return true;
    }

    public void setAsOutput(boolean b)
    {
    }

    public PinDescriptor getIndex()
    {
        return null;
    }

    public void setHigh()
    {
    }

    public void setLow()
    {
    }

    public void setHigh(boolean high)
    {
    }

    public boolean getState()
    {
        return true;
    }

    public String[] getTags()
    {
        return null;
    }

    public void addTag(String tag)
    {
    }

    public void removeTag(String tag)
    {
    }

    public boolean hasTag(String tag)
    {
        return true;
    }

    public boolean isLow()
    {
        return true;
    }

    public boolean isHigh()
    {
        return true;
    }

    public void waitForChange()
    {
    }

    public void addIInputPinListener(IInputPinListener who)
    {
    }

    public void removeIInputPinListener(IInputPinListener who)
    {
    }

    public IInputPinListener[] getIInputPinListeners()
    {
        return null;
    }
}
