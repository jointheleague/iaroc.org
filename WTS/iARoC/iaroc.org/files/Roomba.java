// $Id: Roomba.java 3609 2010-02-15 20:05:39Z vic $
package org.sunspotworld;

import com.sun.spot.sensorboard.EDemoBoard;
import com.sun.spot.util.Utils;

/**
 * Interface class that provides a high level interface to the iRobot series of
 * Roomba/Create robots.
 * 
 * The names of most of the sensors are derived mostly from the
 * "Create Open Interface_v2.pdf" document found on the http://www.irobot.com
 * site. Recommend reading this document in order to get a better understanding
 * of how to work witht the robot.
 * 
 * NOTE: The default constructor will not return until it has managed to connect
 * with the create via the default D0/D1/D2 serial interface from the Sun SPOT
 * to the create. This means that once you have received the instance, you are
 * ok to control the robot.  Plays the Pink Panther theme when connected.
 */
public class Roomba
{
    private final int[] sensorValues;
    private final RoombaSerialConnection serialConnection;
    private static final int SENSORID_advanceButton = 0;
    private static final int SENSORID_angle = 25;
    private static final int SENSORID_batteryCapacity = 1;
    private static final int SENSORID_batteryCharge = 2;
    private static final int SENSORID_batteryTemperature = 3;
    private static final int SENSORID_bumpLeft = 4;
    private static final int SENSORID_bumpRight = 5;
    private static final int SENSORID_cargoBayAnalogSignal = 6;
    private static final int SENSORID_cargoBayDeviceDetectBaudRateChange = 11;
    private static final int SENSORID_cargoBayDigitalInput0 = 7;
    private static final int SENSORID_cargoBayDigitalInput1 = 8;
    private static final int SENSORID_cargoBayDigitalInput2 = 9;
    private static final int SENSORID_cargoBayDigitalInput3 = 10;
    private static final int SENSORID_chargingState = 12;
    private static final int SENSORID_cliffFrontLeft = 16;
    private static final int SENSORID_cliffFrontLeftSignal = 13;
    private static final int SENSORID_cliffFrontRight = 17;
    private static final int SENSORID_cliffFrontRightSignal = 14;
    private static final int SENSORID_cliffLeft = 18;
    private static final int SENSORID_cliffLeftSignal = 19;
    private static final int SENSORID_cliffRight = 20;
    private static final int SENSORID_cliffRightSignal = 15;
    private static final int SENSORID_current = 21;
    private static final int SENSORID_distance = 26;
    private static final int SENSORID_homeBaseChargerAvailable = 22;
    private static final int SENSORID_infraredByte = 23;
    private static final int SENSORID_internalChargerAvailable = 24;
    private static final int SENSORID_leftWheelOvercurrent = 30;
    private static final int SENSORID_lowSideDriver0Overcurrent = 27;
    private static final int SENSORID_lowSideDriver1Overcurrent = 28;
    private static final int SENSORID_lowSideDriver2Overcurrent = 29;
    private static final int SENSORID_numberOfStreamPackets = 31;
    private static final int SENSORID_oiMode = 32;
    private static final int SENSORID_playButton = 33;
    private static final int SENSORID_requestedLeftVelocity = 34;
    private static final int SENSORID_requestedRadius = 35;
    private static final int SENSORID_requestedRightVelocity = 36;
    private static final int SENSORID_requestedVelocity = 37;
    private static final int SENSORID_rightWheelOvercurrent = 38;
    private static final int SENSORID_songNumber = 39;
    private static final int SENSORID_songPlaying = 40;
    private static final int SENSORID_virtualWall = 41;
    private static final int SENSORID_voltage = 42;
    private static final int SENSORID_wall = 43;
    private static final int SENSORID_wallSignal = 44;
    private static final int SENSORID_wheelDropCaster = 45;
    private static final int SENSORID_wheelDropLeft = 46;
    private static final int SENSORID_wheelDropRight = 47;
    private static final int SENSORID_MAX = 48;

    /**
     * Create a new instance that uses the default configuration of using the EDemoBoard for UART and uses pin D2
     * as the baud rate change pin to the iRobot Create.
     */
    public Roomba()
    {
        this(true, true);
    }

    /**
     * Return an instance that uses serialConnection as its means of sending and reading date
     * to and from the Create.
     * 
     * @param fullMode if true enter full mode, otherwise enter safe mode
     * @param waitButton if true wait until play button is pressed
     */
    public Roomba(boolean fullMode, boolean waitButton)
    {
        EDemoBoard eDemoBoard = EDemoBoard.getInstance();
        this.serialConnection = new RoombaSerialConnection(eDemoBoard, eDemoBoard.getIOPins()[EDemoBoard.D2]);

        if (fullMode)
        {
            full();
        } else
        {
            safe();
        }
        sensorValues = new int[SENSORID_MAX];
        // Some initialization to make it such that we minimize the events that will be fired
        // when we first connect to the Create
        sensorValues[SENSORID_infraredByte] = RoombaConstants.INFRARED_BYTE_NONE;
        leds(true, true, true);
        if (waitButton)
        {
            waitButtonPressed(true, true);
        }
    }

    /**
     * Create attempts to cover an entire room using a combination of behaviors, such as 
     * random bounce, wall following, and spiraling.
     */
    public synchronized void cover()
    {
        serialConnection.sendByte(RoombaConstants.COMMAND_DEMO_COVER);
        serialConnection.afterCommandPause();
    }

    /**
     * Identical to the Cover demo, with one exception. If Create sees an infrared signal from 
     * an iRobot Home Base, it uses that signal to dock with the Home Base and recharge itself.
     */
    public synchronized void coverAndDock()
    {
        serialConnection.sendByte(RoombaConstants.COMMAND_DEMO_COVER_AND_DOCK);
        serialConnection.afterCommandPause();
    }

    /**
     * This command starts the requested built-in demo.
     * 
     * @param demoType one IRobotCreateConstants.DEMO_*
     */
    public synchronized void demo(int demoType)
    {
        serialConnection.sendByte(RoombaConstants.COMMAND_DEMO);
        serialConnection.sendByte(demoType);
        serialConnection.afterCommandPause();
    }

    /**
     * This command controls the state of the 3 digital output pins on the 25
     * pin Cargo Bay Connector. The digital outputs can provide up to 20 mA of
     * current.
     * 
     * @param pin0High
     * @param pin1High
     * @param pin2High
     */
    public synchronized void digitalOutputs(boolean pin0High, boolean pin1High, boolean pin2High)
    {
        serialConnection.sendByte(RoombaConstants.COMMAND_DIGITAL_OUTPUTS);
        serialConnection.sendByte((pin0High ? RoombaConstants.DIGITAL_OUTPUT_PIN0 : 0) | (pin1High ? RoombaConstants.DIGITAL_OUTPUT_PIN1 : 0) | (pin2High ? RoombaConstants.DIGITAL_OUTPUT_PIN2 : 0));
        serialConnection.afterCommandPause();
    }

    /**
     * This command controls Create’s drive wheels. It takes four data bytes,
     * interpreted as two 16-bit signed values using two’s complement. The first
     * two bytes specify the average velocity of the drive wheels in millimeters
     * per second (mm/s), with the high byte being sent first. The next two
     * bytes specify the radius in millimeters at which Create will turn. The
     * longer radii make Create drive straighter, while the shorter radii make
     * Create turn more. The radius is measured from the center of the turning
     * circle to the center of Create. A Drive command with a positive velocity
     * and a positive radius makes Create drive forward while turning toward the
     * left. A negative radius makes Create turn toward the right. Special cases
     * for the radius make Create turn in place or drive straight, as specified
     * below. A negative velocity makes Create drive backward. <br>
     * NOTE: Internal and environmental restrictions may prevent Create from
     * accurately carrying out some drive commands. For example, it may not be
     * possible for Create to drive at full speed in an arc with a large radius
     * of curvature. <cr> • Available in modes: Safe or Full <cr> • Changes mode
     * to: No Change <cr> • Drive data byte 2: Radius () Special cases: Straight
     * = 32768 or 32767 = hex 8000 or 7FFF Turn in place clockwise = hex FFFF
     * Turn in place counter-clockwise = hex 0001
     * 
     * @param velocity
     *            -500 – 500 mm/s
     * @param radius
     *            -2000 – 2000 mm, special cases
     *            IRobotCreateConstants.DRIVE_STRAIGHT,
     *            DRIVE_TURN_IN_PLACE_CLOCKWISE,
     *            DRIVE_TURN_IN_PLACE_COUNTER_CLOCKWISE
     */
    public synchronized void drive(int velocity, int radius)
    {
        serialConnection.sendByte(RoombaConstants.COMMAND_DRIVE);
        serialConnection.sendSignedWord(velocity);
        serialConnection.sendSignedWord(radius);
        serialConnection.afterCommandPause();
    }

    /**
     * This command lets you control the forward and backward motion of Create’s
     * drive wheels independently. It takes four data bytes, which are
     * interpreted as two 16-bit signed values using two’s complement. The first
     * two bytes specify the velocity of the right wheel in millimeters per
     * second (mm/s), with the high byte sent first. The next two bytes specify
     * the velocity of the left wheel, in the same format. A positive velocity
     * makes that wheel drive forward, while a negative velocity makes it drive
     * backward. <br>
     * • Available in modes: Safe or Full <br>
     * • Changes mode to: No Change
     * 
     * @param rightVelocity
     *            Right wheel velocity (-500 – 500 mm/s)
     * @param leftVelocity
     *            Left wheel velocity (-500 – 500 mm/s)
     */
    public synchronized void driveDirect(int rightVelocity, int leftVelocity)
    {
        serialConnection.sendByte(RoombaConstants.COMMAND_DRIVE_DIRECT);
        serialConnection.sendSignedWord(rightVelocity);
        serialConnection.sendSignedWord(leftVelocity);
        serialConnection.afterCommandPause();
    }

    /**
     * This command gives you complete control over Create by putting the OI
     * into Full mode, and turning off the cliff, wheel-drop and internal
     * charger safety features. That is, in Full mode, Create executes any
     * command that you send it, even if the internal charger is plugged in, or
     * the robot senses a cliff or wheel drop. <br>
     * • Available in modes: Passive, Safe, or Full <br>
     * • Changes mode to: Full <br>
     * Note: Use the Start command (128) to change the mode to Passive.
     */
    public synchronized void full()
    {
        serialConnection.sendByte(RoombaConstants.COMMAND_MODE_FULL);
        serialConnection.afterCommandPause();
    }

    /**
     * The angle in degrees that iRobot Create has turned since the angle was
     * last requested. Counter-clockwise angles are positive and clockwise
     * angles are negative. If the value is not polled frequently enough, it is
     * capped at its minimum or maximum. <br>
     * Range: -32768 – 32767 <br>
     * NOTE: Create uses wheel encoders to measure distance and angle. If the
     * wheels slip, the actual distance or angle traveled may differ from
     * Create’s measurements.
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized int getAngle()
    {
        return getSensorInteger(SENSORID_angle);
    }

    /**
     * The estimated charge capacity of Create’s battery in milliamphours (mAh).
     * Note that this value is inaccurate if you are using the alkaline battery
     * pack. <br>
     * Range: 0 – 65535
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized int getBatteryCapacity()
    {
        return getSensorInteger(SENSORID_batteryCapacity);
    }

    /**
     * The current charge of Create’s battery in milliamp-hours (mAh). The
     * charge value decreases as the battery is depleted during running and
     * increases when the battery is charged. <br>
     * Note that this value will not be accurate if you are using the alkaline
     * battery pack. <br>
     * Range: 0 – 65535
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized int getBatteryCharge()
    {
        return getSensorInteger(SENSORID_batteryCharge);
    }

    /**
     * The temperature of Create’s battery in degrees Celsius. <br>
     * Range: -128 – 127
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized int getBatteryTemperature()
    {
        return getSensorInteger(SENSORID_batteryTemperature);
    }

    /**
     * The 10-bit value of the analog input on the 25-pin Cargo Bay Connector is
     * returned, high byte first. 0 = 0 volts; 1023 = 5 volts. The analog input
     * is on pin 4. <br>
     * Range: 0 - 1023
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized int getCargoBayAnalogSignal()
    {
        return getSensorInteger(SENSORID_cargoBayAnalogSignal);
    }

    /**
     * This code indicates Create’s current charging state.
     * 
     * @return One of IRobotCreateConstants.CHARGING_STATE*
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized int getChargingState()
    {
        return getSensorInteger(SENSORID_chargingState);
    }

    /**
     * The strength of the front left cliff sensor’s signal. <br>
     * Range: 0-4095
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized int getCliffFrontLeftSignal()
    {
        return getSensorInteger(SENSORID_cliffFrontLeftSignal);
    }

    /**
     * The strength of the front right cliff sensor’s signal. <br>
     * Range: 0-4095
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized int getCliffFrontRightSignal()
    {
        return getSensorInteger(SENSORID_cliffFrontRightSignal);
    }

    /**
     * The strength of the left cliff sensor’s signal is returned. <br>
     * Range: 0-4095
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized int getCliffLeftSignal()
    {
        return getSensorInteger(SENSORID_cliffLeftSignal);
    }

    /**
     * The strength of the front left cliff sensor’s signal is returned. <br>
     * Range: 0-4095
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized int getCliffRightSignal()
    {
        return getSensorInteger(SENSORID_cliffRightSignal);
    }

    /**
     * The current in milliamps (mA) flowing into or out of Create’s battery.
     * Negative currents indicate that the current is flowing out of the
     * battery, as during normal running. Positive currents indicate that the
     * current is flowing into the battery, as during charging. <br>
     * Range: -32768 – 32767
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized int getCurrent()
    {
        return getSensorInteger(SENSORID_current);
    }

    /**
     * The distance that Create has traveled in millimeters since the distance
     * it was last requested is sent as a signed 16-bit value, high byte first.
     * This is the same as the sum of the distance traveled by both wheels
     * divided by two. Positive values indicate travel in the forward direction;
     * negative values indicate travel in the reverse direction. If the value is
     * not polled frequently enough, it is capped at its minimum or maximum. <br>
     * Range: -32768 – 32767
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized int getDistance()
    {
        return getSensorInteger(SENSORID_distance);
    }

    /**
     * This value identifies the IR byte currently being received by iRobot
     * Create. A value of 255 indicates that no IR byte is being received. These
     * bytes include those sent by the Roomba Remote, the Home Base, Create
     * robots using the Send IR command, and user-created devices. <br>
     * Range: 0 – 255
     * 
     * @return One of IRobotCreateConstants.INFRARED_BYTE_*
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized int getInfraredByte()
    {
        return getSensorInteger(SENSORID_infraredByte);
    }

    /**
     * The number of data stream packets is returned. <br>
     * Range: 0-43
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized int getNumberOfStreamPackets()
    {
        return getSensorInteger(SENSORID_numberOfStreamPackets);
    }

    /**
     * Create’s connection to the Home Base and Internal Charger are returned as
     * individual bits, as below.
     * 
     * @return One of IRobotCreateConstants.OI_MODE_*
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized int getOiMode()
    {
        return getSensorInteger(SENSORID_oiMode);
    }

    /**
     * The velocity most recently requested with a Drive command is returned. <br>
     * Range: -500 - 500 mm/s
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized int getRequestedLeftVelocity()
    {
        return getSensorInteger(SENSORID_requestedLeftVelocity);
    }

    /**
     * The radius most recently requested with a Drive command is returned. <br>
     * Range: -32768 - 32767 mm
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized int getRequestedRadius()
    {
        return getSensorInteger(SENSORID_requestedRadius);
    }

    /**
     * The right wheel velocity most recently requested with a Drive Direct
     * command is returned. <br>
     * Range: -500 - 500 mm/s
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized int getRequestedRightVelocity()
    {
        return getSensorInteger(SENSORID_requestedRightVelocity);
    }

    /**
     * The velocity most recently requested with a Drive command is returned. <br>
     * Range: -500 - 500 mm/s
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized int getRequestedVelocity()
    {
        return getSensorInteger(SENSORID_requestedVelocity);
    }

    private boolean getSensorBoolean(int sensorId)
    {
        return sensorValues[sensorId] != 0;
    }

    private int getSensorInteger(int sensorId)
    {
        return sensorValues[sensorId];
    }

    /**
     * The currently selected OI song is returned. <br>
     * Range: 0-15
     * 
     * @return
     */
    public synchronized int getSongNumber()
    {
        return getSensorInteger(SENSORID_songNumber);
    }

    /**
     * This code indicates the voltage of Create’s batter y in millivolts (mV). <br>
     * Range: 0 – 65535
     * 
     * @return
     */
    public synchronized int getVoltage()
    {
        return getSensorInteger(SENSORID_voltage);
    }

    /**
     * The strength of the wall sensor’s signal is returned. <br>
     * Range: 0-4095
     * 
     * @return
     */
    public synchronized int getWallSignal()
    {
        return getSensorInteger(SENSORID_wallSignal);
    }

    /**
     * Return if advance button is down.
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isAdvanceButtonDown()
    {
        return getSensorBoolean(SENSORID_advanceButton);
    }

    /**
     * Return true if the left bumper is pressed.
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isBumpLeft()
    {
        return getSensorBoolean(SENSORID_bumpLeft);
    }

    /**
     * Return true if the right bumper is pressed.
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isBumpRight()
    {
        return getSensorBoolean(SENSORID_bumpRight);
    }

    /**
     * Return true if the device detect pin is high.
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isCargoBayDeviceDetectBaudRateChangeHigh()
    {
        return getSensorBoolean(SENSORID_cargoBayDeviceDetectBaudRateChange);
    }

    /**
     * Return true if the cargo bay input 0 pin is high. The state of the
     * digital inputs on the 25-pin Cargo Bay Connector are sent as individual
     * bits (false = low, true = high (5V)). Note that the Baud Rate Change pin
     * is active low; it is high by default.
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isCargoBayDigitalInput0High()
    {
        return getSensorBoolean(SENSORID_cargoBayDigitalInput0);
    }

    /**
     * Return true if the cargo bay input 1 pin is high. The state of the
     * digital inputs on the 25-pin Cargo Bay Connector are sent as individual
     * bits (false = low, true = high (5V)). Note that the Baud Rate Change pin
     * is active low; it is high by default.
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isCargoBayDigitalInput1High()
    {
        return getSensorBoolean(SENSORID_cargoBayDigitalInput1);
    }

    /**
     * Return true if the cargo bay input 2 pin is high. The state of the
     * digital inputs on the 25-pin Cargo Bay Connector are sent as individual
     * bits (false = low, true = high (5V)). Note that the Baud Rate Change pin
     * is active low; it is high by default.
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isCargoBayDigitalInput2High()
    {
        return getSensorBoolean(SENSORID_cargoBayDigitalInput2);
    }

    /**
     * Return true if the cargo bay input 3 pin is high. The state of the
     * digital inputs on the 25-pin Cargo Bay Connector are sent as individual
     * bits (false = low, true = high (5V)). Note that the Baud Rate Change pin
     * is active low; it is high by default.
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isCargoBayDigitalInput3High()
    {
        return getSensorBoolean(SENSORID_cargoBayDigitalInput3);
    }

    /**
     * Return true if the cliff front left sensor.
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isCliffFrontLeft()
    {
        return getSensorBoolean(SENSORID_cliffFrontLeft);
    }

    /**
     * Return true if the cliff front right sensor.
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isCliffFrontRight()
    {
        return getSensorBoolean(SENSORID_cliffFrontRight);
    }

    /**
     * Return true if the cliff left sensor.
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isCliffLeft()
    {
        return getSensorBoolean(SENSORID_cliffLeft);
    }

    /**
     * Return true if the cliff right sensor.
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isCliffRight()
    {
        return getSensorBoolean(SENSORID_cliffRight);
    }

    /**
     * Create’s connection to the Home Base and Internal Charger.
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isHomeBaseChargerAvailable()
    {
        return getSensorBoolean(SENSORID_homeBaseChargerAvailable);
    }

    /**
     * Create’s connection to the Home Base and Internal Charger.
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isInternalChargerAvailable()
    {
        return getSensorBoolean(SENSORID_internalChargerAvailable);
    }

    /**
     * The state of the three Low Side driver and two wheel overcurrent sensors
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isLeftWheelOvercurrent()
    {
        return getSensorBoolean(SENSORID_leftWheelOvercurrent);
    }

    /**
     * The state of the three Low Side driver and two wheel overcurrent sensors
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isLowSideDriver0Overcurrent()
    {
        return getSensorBoolean(SENSORID_lowSideDriver0Overcurrent);
    }

    /**
     * The state of the three Low Side driver and two wheel overcurrent sensors
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isLowSideDriver1Overcurrent()
    {
        return getSensorBoolean(SENSORID_lowSideDriver1Overcurrent);
    }

    /**
     * The state of the three Low Side driver and two wheel overcurrent sensors
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isLowSideDriver2Overcurrent()
    {
        return getSensorBoolean(SENSORID_lowSideDriver2Overcurrent);
    }

    /**
     * Is play button pressed.
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isPlayButtonDown()
    {
        return getSensorBoolean(SENSORID_playButton);
    }

    /**
     * The state of the three Low Side driver and two wheel overcurrent sensors
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isRightWheelOvercurrent()
    {
        return getSensorBoolean(SENSORID_rightWheelOvercurrent);
    }

    /**
     * The state of the OI song player is returned.
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isSongPlaying()
    {
        return getSensorBoolean(SENSORID_songPlaying);
    }

    /**
     * The state of the virtual wall detector (false = no virtual wall detected,
     * true = virtual wall detected). <br>
     * Note that the force field on top of the Home Base also trips this sensor.
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isVirtualWall()
    {
        return getSensorBoolean(SENSORID_virtualWall);
    }

    /**
     * The state of the wall sensor, true if there is a wall.
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isWall()
    {
        return getSensorBoolean(SENSORID_wall);
    }

    /**
     * The state of the wheel drop sensor.
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isWheelDropCaster()
    {
        return getSensorBoolean(SENSORID_wheelDropCaster);
    }

    /**
     * The state of the wheel drop sensor.
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isWheelDropLeft()
    {
        return getSensorBoolean(SENSORID_wheelDropLeft);
    }

    /**
     * The state of the wheel drop sensor.
     * 
     * @return
     * @throws IllegalStateException
     *             If the last sensors, queryList, stream commands did not cause
     *             this sensor value to be read/updated.
     */
    public synchronized boolean isWheelDropRight()
    {
        return getSensorBoolean(SENSORID_wheelDropRight);
    }

    /**
     * This command controls the LEDs on Create. The state of the power Play and
     * Advance LEDs is specified by true or false. Power led on will use full
     * green to match other LEDs. <br>
     * • Available in modes: Safe or Full <br>
     * • Changes mode to: No Change <br>
     * 
     * @param powerLedOn
     * @param playLedOn
     * @param advanceLedOn
     */
    public synchronized void leds(boolean powerLedOn, boolean playLedOn, boolean advanceLedOn)
    {
        leds(0, powerLedOn ? 255 : 0, playLedOn, advanceLedOn);
    }

    /**
     * This command controls the LEDs on Create. The state of the Play and
     * Advance LEDs is specified by true or false. The power LED is specified by
     * two values: one for the color and the other for the intensity. <br>
     * • Available in modes: Safe or Full <br>
     * • Changes mode to: No Change <br>
     * Advance and Play use green LEDs. false = off, true = on <br>
     * Power uses a bicolor (red/green) LED. The intensity and color of this LED
     * can be controlled with 8-bit resolution.
     * 
     * @param powerColor
     *            Power LED Color (0 – 255) 0 = green, 255 = red. Intermediate
     *            values are intermediate colors (orange, yellow, etc).
     * @param powerIntensity
     *            Power LED Intensity (0 – 255) 0 = off, 255 = full intensity.
     *            Intermediate values are intermediate intensities.
     * @param playLedOn
     * @param advanceLedOn
     */
    public synchronized void leds(int powerColor, int powerIntensity, boolean playLedOn, boolean advanceLedOn)
    {
        serialConnection.sendByte(RoombaConstants.COMMAND_LEDS);
        serialConnection.sendByte((advanceLedOn ? 8 : 0) | (playLedOn ? 2 : 0));
        serialConnection.sendByte(powerColor);
        serialConnection.sendByte(powerIntensity);
        serialConnection.afterCommandPause();
    }

    /**
     * This command lets you control the three low side drivers. The state of
     * each driver is specified by true or false. Low side drivers 0 and 1 can
     * provide up to 0.5A of current. Low side driver 2 can provide up to 1.5 A
     * of current. If too much current is requested, the current is limited and
     * the overcurrent flag is set (sensor packet 14). <br>
     * • Available in modes: Safe or Full <br>
     * • Changes mode to: No Change
     * 
     * @param lowSideDriver0On
     * @param lowSideDriver1On
     * @param lowSideDriver2On
     */
    public synchronized void lowSideDrivers(boolean lowSideDriver0On, boolean lowSideDriver1On, boolean lowSideDriver2On)
    {
        serialConnection.sendByte(RoombaConstants.COMMAND_LOW_SIDE_DRIVERS);
        serialConnection.sendByte((lowSideDriver0On ? RoombaConstants.LOW_SIDE_DRIVER0 : 0) | (lowSideDriver1On ? RoombaConstants.LOW_SIDE_DRIVER1 : 0) | (lowSideDriver2On ? RoombaConstants.LOW_SIDE_DRIVER2 : 0));
        serialConnection.afterCommandPause();
    }

    /**
     * This command lets you stop and restart the steam without clearing the
     * list of requested packets. <br>
     * • Available in modes: Passive, Safe, or Full <br>
     * • Changes mode to: No Change <br>
     * An argument of false stops the stream without clearing the list of
     * requested packets. An argument of true starts the stream using the list
     * of packets last requested.
     * 
     * @param pause
     */
    public synchronized void pauseResumeStream(boolean pause)
    {
        serialConnection.sendByte(RoombaConstants.COMMAND_PAUSE_RESUME_STREAM);
        serialConnection.sendByte(pause ? 0 : 1);
    }

    /**
     * This command lets you select a song to play from the songs added to
     * iRobot Create using the Song command. You must add one or more songs to
     * Create using the Song command in order for the Play command to work.
     * Also, this command does not work if a song is already playing. Wait until
     * a currently playing song is done before sending this command. Note that
     * the “song playing” sensor packet can be used to check whether Create is
     * ready to accept this command. <br>
     * • Available in modes: Safe or Full <br>
     * • Changes mode to: No Change
     * 
     * @param songNumber
     *            The number of the song Create is to play.
     */
    public synchronized void playSong(int songNumber)
    {
        serialConnection.sendByte(RoombaConstants.COMMAND_PLAY_SONG);
        serialConnection.sendByte(songNumber);
        serialConnection.afterCommandPause();
    }

    /**
     * This command lets you control the three low side drivers with variable
     * power. You specify the PWM duty cycle for the low side driver (max 128).
     * For example, if you want to control a driver with 25% of battery voltage,
     * choose a duty cycle of 128 * 25% = 32. <br>
     * • Available in modes: Safe or Full <br>
     * • Changes mode to: No Change
     * 
     * @param lowSideDriver0DutyCycle
     *            Duty cycle for low side driver 0 (0 - 128)
     * @param lowSideDriver1DutyCycle
     *            Duty cycle for low side driver 1 (0 - 128)
     * @param lowSideDriver2DutyCycle
     *            Duty cycle for low side driver 2 (0 - 128)
     */
    public synchronized void pwmLowSideDrivers(int lowSideDriver0DutyCycle, int lowSideDriver1DutyCycle, int lowSideDriver2DutyCycle)
    {
        serialConnection.sendByte(RoombaConstants.COMMAND_PWM_LOW_SIDE_DRIVERS);
        serialConnection.sendByte(lowSideDriver2DutyCycle);
        serialConnection.sendByte(lowSideDriver1DutyCycle);
        serialConnection.sendByte(lowSideDriver0DutyCycle);
        serialConnection.afterCommandPause();
    }

    /**
     * This command lets you ask for a list of sensor packets. The result is
     * returned once, as in the Sensors command. The robot returns the packets
     * in the order you specify. <br>
     * 
     * • Available in modes: Passive, Safe, or Full <br>
     * • Changes mode to: No Change
     * 
     * @param packetIds Any of the IRobotCreate.SENSORS_* constants.
     * @param eventHandler
     */
    public synchronized void queryList(int[] packetIds)
    {
        serialConnection.sendByte(RoombaConstants.COMMAND_QUERY_LIST);
        serialConnection.sendByte(packetIds.length);
        serialConnection.sendBytes(packetIds, 0, packetIds.length);
        serialConnection.afterCommandPause();
        readSensorData(packetIds);
    }

    private void readSensorData(int sensorPacketId)
    {
        readSensorDataPrim(sensorPacketId);
    }

    private void readSensorData(int[] sensorPacketIds)
    {
        for (int i = 0, length = sensorPacketIds.length; i < length; i++)
        {
            readSensorDataPrim(sensorPacketIds[i]);
        }
    }

    private void readSensorDataPrim(int sensorPacketId)
    {
        int dataByte, dataWord;

        switch (sensorPacketId)
        {
            case RoombaConstants.SENSORS_GROUP_ID0:
                readSensorDataPrim(RoombaConstants.SENSORS_BUMPS_AND_WHEEL_DROPS);
                readSensorDataPrim(RoombaConstants.SENSORS_WALL);
                readSensorDataPrim(RoombaConstants.SENSORS_CLIFF_LEFT);
                readSensorDataPrim(RoombaConstants.SENSORS_CLIFF_FRONT_LEFT);
                readSensorDataPrim(RoombaConstants.SENSORS_CLIFF_FRONT_RIGHT);
                readSensorDataPrim(RoombaConstants.SENSORS_CLIFF_RIGHT);
                readSensorDataPrim(RoombaConstants.SENSORS_VIRTUAL_WALL);
                readSensorDataPrim(RoombaConstants.SENSORS_LOWS_SIDE_DRIVER_AND_WHEEL_OVERCURRENTS);
                readSensorDataPrim(RoombaConstants.SENSORS_UNUSED1);
                readSensorDataPrim(RoombaConstants.SENSORS_UNUSED2);
                readSensorDataPrim(RoombaConstants.SENSORS_INFRARED_BYTE);
                readSensorDataPrim(RoombaConstants.SENSORS_BUTTONS);
                readSensorDataPrim(RoombaConstants.SENSORS_DISTANCE);
                readSensorDataPrim(RoombaConstants.SENSORS_ANGLE);
                readSensorDataPrim(RoombaConstants.SENSORS_CHARGING_STATE);
                readSensorDataPrim(RoombaConstants.SENSORS_VOLTAGE);
                readSensorDataPrim(RoombaConstants.SENSORS_CURRENT);
                readSensorDataPrim(RoombaConstants.SENSORS_BATTERY_TEMPERATURE);
                readSensorDataPrim(RoombaConstants.SENSORS_BATTERY_CHARGE);
                readSensorDataPrim(RoombaConstants.SENSORS_BATTERY_CAPACITY);
                break;
            case RoombaConstants.SENSORS_GROUP_ID1:
                readSensorDataPrim(RoombaConstants.SENSORS_BUMPS_AND_WHEEL_DROPS);
                readSensorDataPrim(RoombaConstants.SENSORS_WALL);
                readSensorDataPrim(RoombaConstants.SENSORS_CLIFF_LEFT);
                readSensorDataPrim(RoombaConstants.SENSORS_CLIFF_FRONT_LEFT);
                readSensorDataPrim(RoombaConstants.SENSORS_CLIFF_FRONT_RIGHT);
                readSensorDataPrim(RoombaConstants.SENSORS_CLIFF_RIGHT);
                readSensorDataPrim(RoombaConstants.SENSORS_VIRTUAL_WALL);
                readSensorDataPrim(RoombaConstants.SENSORS_LOWS_SIDE_DRIVER_AND_WHEEL_OVERCURRENTS);
                readSensorDataPrim(RoombaConstants.SENSORS_UNUSED1);
                readSensorDataPrim(RoombaConstants.SENSORS_UNUSED2);
                break;
            case RoombaConstants.SENSORS_GROUP_ID2:
                readSensorDataPrim(RoombaConstants.SENSORS_INFRARED_BYTE);
                readSensorDataPrim(RoombaConstants.SENSORS_BUTTONS);
                readSensorDataPrim(RoombaConstants.SENSORS_DISTANCE);
                readSensorDataPrim(RoombaConstants.SENSORS_ANGLE);
                break;
            case RoombaConstants.SENSORS_GROUP_ID3:
                readSensorDataPrim(RoombaConstants.SENSORS_CHARGING_STATE);
                readSensorDataPrim(RoombaConstants.SENSORS_VOLTAGE);
                readSensorDataPrim(RoombaConstants.SENSORS_CURRENT);
                readSensorDataPrim(RoombaConstants.SENSORS_BATTERY_TEMPERATURE);
                readSensorDataPrim(RoombaConstants.SENSORS_BATTERY_CHARGE);
                readSensorDataPrim(RoombaConstants.SENSORS_BATTERY_CAPACITY);
                break;
            case RoombaConstants.SENSORS_GROUP_ID4:
                readSensorDataPrim(RoombaConstants.SENSORS_WALL_SIGNAL);
                readSensorDataPrim(RoombaConstants.SENSORS_CLIFF_LEFT_SIGNAL);
                readSensorDataPrim(RoombaConstants.SENSORS_CLIFF_FRONT_LEFT_SIGNAL);
                readSensorDataPrim(RoombaConstants.SENSORS_CLIFF_FRONT_RIGHT_SIGNAL);
                readSensorDataPrim(RoombaConstants.SENSORS_CLIFF_RIGHT_SIGNAL);
                readSensorDataPrim(RoombaConstants.SENSORS_CARGO_BAY_DIGITAL_INPUTS);
                readSensorDataPrim(RoombaConstants.SENSORS_CARGO_BAY_ANALOG_SIGNAL);
                readSensorDataPrim(RoombaConstants.SENSORS_CHARGING_SOURCES_AVAILABLE);
                break;
            case RoombaConstants.SENSORS_GROUP_ID5:
                readSensorDataPrim(RoombaConstants.SENSORS_OI_MODE);
                readSensorDataPrim(RoombaConstants.SENSORS_SONG_NUMBER);
                readSensorDataPrim(RoombaConstants.SENSORS_SONG_PLAYING);
                readSensorDataPrim(RoombaConstants.SENSORS_NUMBER_OF_STREAM_PACKETS);
                readSensorDataPrim(RoombaConstants.SENSORS_REQUESTED_VELOCITY);
                readSensorDataPrim(RoombaConstants.SENSORS_REQUESTED_RADIUS);
                readSensorDataPrim(RoombaConstants.SENSORS_REQUESTED_RIGHT_VELOCITY);
                readSensorDataPrim(RoombaConstants.SENSORS_REQUESTED_LEFT_VELOCITY);
                break;
            case RoombaConstants.SENSORS_GROUP_ID6:
                readSensorDataPrim(RoombaConstants.SENSORS_BUMPS_AND_WHEEL_DROPS);
                readSensorDataPrim(RoombaConstants.SENSORS_WALL);
                readSensorDataPrim(RoombaConstants.SENSORS_CLIFF_LEFT);
                readSensorDataPrim(RoombaConstants.SENSORS_CLIFF_FRONT_LEFT);
                readSensorDataPrim(RoombaConstants.SENSORS_CLIFF_FRONT_RIGHT);
                readSensorDataPrim(RoombaConstants.SENSORS_CLIFF_RIGHT);
                readSensorDataPrim(RoombaConstants.SENSORS_VIRTUAL_WALL);
                readSensorDataPrim(RoombaConstants.SENSORS_LOWS_SIDE_DRIVER_AND_WHEEL_OVERCURRENTS);
                readSensorDataPrim(RoombaConstants.SENSORS_UNUSED1);
                readSensorDataPrim(RoombaConstants.SENSORS_UNUSED2);
                readSensorDataPrim(RoombaConstants.SENSORS_INFRARED_BYTE);
                readSensorDataPrim(RoombaConstants.SENSORS_BUTTONS);
                readSensorDataPrim(RoombaConstants.SENSORS_DISTANCE);
                readSensorDataPrim(RoombaConstants.SENSORS_ANGLE);
                readSensorDataPrim(RoombaConstants.SENSORS_CHARGING_STATE);
                readSensorDataPrim(RoombaConstants.SENSORS_VOLTAGE);
                readSensorDataPrim(RoombaConstants.SENSORS_CURRENT);
                readSensorDataPrim(RoombaConstants.SENSORS_BATTERY_TEMPERATURE);
                readSensorDataPrim(RoombaConstants.SENSORS_BATTERY_CHARGE);
                readSensorDataPrim(RoombaConstants.SENSORS_BATTERY_CAPACITY);
                readSensorDataPrim(RoombaConstants.SENSORS_WALL_SIGNAL);
                readSensorDataPrim(RoombaConstants.SENSORS_CLIFF_LEFT_SIGNAL);
                readSensorDataPrim(RoombaConstants.SENSORS_CLIFF_FRONT_LEFT_SIGNAL);
                readSensorDataPrim(RoombaConstants.SENSORS_CLIFF_FRONT_RIGHT_SIGNAL);
                readSensorDataPrim(RoombaConstants.SENSORS_CLIFF_RIGHT_SIGNAL);
                readSensorDataPrim(RoombaConstants.SENSORS_CARGO_BAY_DIGITAL_INPUTS);
                readSensorDataPrim(RoombaConstants.SENSORS_CARGO_BAY_ANALOG_SIGNAL);
                readSensorDataPrim(RoombaConstants.SENSORS_CHARGING_SOURCES_AVAILABLE);
                readSensorDataPrim(RoombaConstants.SENSORS_OI_MODE);
                readSensorDataPrim(RoombaConstants.SENSORS_SONG_NUMBER);
                readSensorDataPrim(RoombaConstants.SENSORS_SONG_PLAYING);
                readSensorDataPrim(RoombaConstants.SENSORS_NUMBER_OF_STREAM_PACKETS);
                readSensorDataPrim(RoombaConstants.SENSORS_REQUESTED_VELOCITY);
                readSensorDataPrim(RoombaConstants.SENSORS_REQUESTED_RADIUS);
                readSensorDataPrim(RoombaConstants.SENSORS_REQUESTED_RIGHT_VELOCITY);
                readSensorDataPrim(RoombaConstants.SENSORS_REQUESTED_LEFT_VELOCITY);
                break;
            case RoombaConstants.SENSORS_BUMPS_AND_WHEEL_DROPS:
                dataByte = serialConnection.readUnsignedByte();
                setSensorBoolean(SENSORID_bumpRight, (dataByte & 0x01) != 0);
                setSensorBoolean(SENSORID_bumpLeft, (dataByte & 0x02) != 0);
                setSensorBoolean(SENSORID_wheelDropRight, (dataByte & 0x04) != 0);
                setSensorBoolean(SENSORID_wheelDropLeft, (dataByte & 0x08) != 0);
                setSensorBoolean(SENSORID_wheelDropCaster, (dataByte & 0x10) != 0);
                break;
            case RoombaConstants.SENSORS_WALL:
                dataByte = serialConnection.readUnsignedByte();
                setSensorBoolean(SENSORID_wall, (dataByte & 0x01) != 0);
                break;
            case RoombaConstants.SENSORS_CLIFF_LEFT:
                dataByte = serialConnection.readUnsignedByte();
                setSensorBoolean(SENSORID_cliffLeft, (dataByte & 0x01) != 0);
                break;
            case RoombaConstants.SENSORS_CLIFF_FRONT_LEFT:
                dataByte = serialConnection.readUnsignedByte();
                setSensorBoolean(SENSORID_cliffFrontLeft, (dataByte & 0x01) != 0);
                break;
            case RoombaConstants.SENSORS_CLIFF_FRONT_RIGHT:
                dataByte = serialConnection.readUnsignedByte();
                setSensorBoolean(SENSORID_cliffFrontRight, (dataByte & 0x01) != 0);
                break;
            case RoombaConstants.SENSORS_CLIFF_RIGHT:
                dataByte = serialConnection.readUnsignedByte();
                setSensorBoolean(SENSORID_cliffRight, (dataByte & 0x01) != 0);
                break;
            case RoombaConstants.SENSORS_VIRTUAL_WALL:
                dataByte = serialConnection.readUnsignedByte();
                setSensorBoolean(SENSORID_virtualWall, (dataByte & 0x01) != 0);
                break;
            case RoombaConstants.SENSORS_LOWS_SIDE_DRIVER_AND_WHEEL_OVERCURRENTS:
                dataByte = serialConnection.readUnsignedByte();
                setSensorBoolean(SENSORID_lowSideDriver0Overcurrent, (dataByte & 0x02) != 0);
                setSensorBoolean(SENSORID_lowSideDriver1Overcurrent, (dataByte & 0x01) != 0);
                setSensorBoolean(SENSORID_lowSideDriver2Overcurrent, (dataByte & 0x04) != 0);
                setSensorBoolean(SENSORID_rightWheelOvercurrent, (dataByte & 0x08) != 0);
                setSensorBoolean(SENSORID_leftWheelOvercurrent, (dataByte & 0x10) != 0);
                break;
            case RoombaConstants.SENSORS_UNUSED1:
                serialConnection.readUnsignedByte();
                break;
            case RoombaConstants.SENSORS_UNUSED2:
                serialConnection.readUnsignedByte();
                break;
            case RoombaConstants.SENSORS_INFRARED_BYTE:
                setSensorInteger(SENSORID_infraredByte, serialConnection.readUnsignedByte());
                break;
            case RoombaConstants.SENSORS_BUTTONS:
                dataByte = serialConnection.readUnsignedByte();
                setSensorBoolean(SENSORID_playButton, (dataByte & 0x01) != 0);
                setSensorBoolean(SENSORID_advanceButton, (dataByte & 0x04) != 0);
                break;
            case RoombaConstants.SENSORS_DISTANCE:
                dataWord = serialConnection.readSignedWord();
                setSensorInteger(SENSORID_distance, dataWord);
                break;
            case RoombaConstants.SENSORS_ANGLE:
                dataWord = serialConnection.readSignedWord();
                setSensorInteger(SENSORID_angle, dataWord);
                break;
            case RoombaConstants.SENSORS_CHARGING_STATE:
                setSensorInteger(SENSORID_chargingState, serialConnection.readUnsignedByte());
                break;
            case RoombaConstants.SENSORS_VOLTAGE:
                setSensorInteger(SENSORID_voltage, serialConnection.readUnsignedWord());
                break;
            case RoombaConstants.SENSORS_CURRENT:
                setSensorInteger(SENSORID_current, serialConnection.readSignedWord());
                break;
            case RoombaConstants.SENSORS_BATTERY_TEMPERATURE:
                setSensorInteger(SENSORID_batteryTemperature, serialConnection.readSignedByte());
                break;
            case RoombaConstants.SENSORS_BATTERY_CHARGE:
                setSensorInteger(SENSORID_batteryCharge, serialConnection.readUnsignedWord());
                break;
            case RoombaConstants.SENSORS_BATTERY_CAPACITY:
                setSensorInteger(SENSORID_batteryCapacity, serialConnection.readUnsignedWord());
                break;
            case RoombaConstants.SENSORS_WALL_SIGNAL:
                setSensorInteger(SENSORID_wallSignal, serialConnection.readUnsignedWord());
                break;
            case RoombaConstants.SENSORS_CLIFF_LEFT_SIGNAL:
                setSensorInteger(SENSORID_cliffLeftSignal, serialConnection.readUnsignedWord());
                break;
            case RoombaConstants.SENSORS_CLIFF_FRONT_LEFT_SIGNAL:
                setSensorInteger(SENSORID_cliffFrontLeftSignal, serialConnection.readUnsignedWord());
                break;
            case RoombaConstants.SENSORS_CLIFF_FRONT_RIGHT_SIGNAL:
                setSensorInteger(SENSORID_cliffFrontRightSignal, serialConnection.readUnsignedWord());
                break;
            case RoombaConstants.SENSORS_CLIFF_RIGHT_SIGNAL:
                setSensorInteger(SENSORID_cliffRightSignal, serialConnection.readUnsignedWord());
                break;
            case RoombaConstants.SENSORS_CARGO_BAY_DIGITAL_INPUTS:
                dataByte = serialConnection.readUnsignedByte();
                setSensorBoolean(SENSORID_cargoBayDigitalInput0, (dataByte & 0x01) != 0);
                setSensorBoolean(SENSORID_cargoBayDigitalInput1, (dataByte & 0x02) != 0);
                setSensorBoolean(SENSORID_cargoBayDigitalInput2, (dataByte & 0x04) != 0);
                setSensorBoolean(SENSORID_cargoBayDigitalInput3, (dataByte & 0x08) != 0);
                setSensorBoolean(SENSORID_cargoBayDeviceDetectBaudRateChange, (dataByte & 0x10) != 0);
                break;
            case RoombaConstants.SENSORS_CARGO_BAY_ANALOG_SIGNAL:
                setSensorInteger(SENSORID_cargoBayAnalogSignal, serialConnection.readUnsignedWord());
                break;
            case RoombaConstants.SENSORS_CHARGING_SOURCES_AVAILABLE:
                dataByte = serialConnection.readUnsignedByte();
                setSensorBoolean(SENSORID_internalChargerAvailable, (dataByte & 0x01) != 0);
                setSensorBoolean(SENSORID_homeBaseChargerAvailable, (dataByte & 0x02) != 0);
                break;
            case RoombaConstants.SENSORS_OI_MODE:
                setSensorInteger(SENSORID_oiMode, serialConnection.readUnsignedByte());
                break;
            case RoombaConstants.SENSORS_SONG_NUMBER:
                setSensorInteger(SENSORID_songNumber, serialConnection.readUnsignedByte());
                break;
            case RoombaConstants.SENSORS_SONG_PLAYING:
                dataByte = serialConnection.readUnsignedByte();
                setSensorBoolean(SENSORID_songPlaying, (dataByte & 0x01) != 0);
                break;
            case RoombaConstants.SENSORS_NUMBER_OF_STREAM_PACKETS:
                setSensorInteger(SENSORID_numberOfStreamPackets, serialConnection.readUnsignedByte());
                break;
            case RoombaConstants.SENSORS_REQUESTED_VELOCITY:
                setSensorInteger(SENSORID_requestedVelocity, serialConnection.readSignedWord());
                break;
            case RoombaConstants.SENSORS_REQUESTED_RADIUS:
                setSensorInteger(SENSORID_requestedRadius, serialConnection.readSignedWord());
                break;
            case RoombaConstants.SENSORS_REQUESTED_RIGHT_VELOCITY:
                setSensorInteger(SENSORID_requestedRightVelocity, serialConnection.readSignedWord());
                break;
            case RoombaConstants.SENSORS_REQUESTED_LEFT_VELOCITY:
                setSensorInteger(SENSORID_requestedLeftVelocity, serialConnection.readSignedWord());
                break;
            default:
                throw new IllegalArgumentException(String.valueOf(sensorPacketId));
        }
    }

    /**
     * This command puts the OI into Safe mode, enabling user control of Create.
     * It turns off all LEDs. The OI can be in Passive, Safe, or Full mode to
     * accept this command.
     */
    public synchronized void safe()
    {
        serialConnection.sendByte(RoombaConstants.COMMAND_MODE_SAFE);
        serialConnection.afterCommandPause();
    }

    /**
     * This command sends the requested byte out of low side driver 1 (pin 23 on
     * the Cargo Bay Connector), using the format expected by iRobot Create’s IR
     * receiver. You must use a preload resistor (suggested value: 100 ohms) in
     * parallel with the IR LED and its resistor in order turn it on. <br>
     * • Available in modes: Safe or Full <br>
     * • Changes mode to: No Change
     * 
     * @param byteValue
     *            Byte value to send (0 - 255)
     */
    public synchronized void sendIr(int byteValue)
    {
        serialConnection.sendByte(RoombaConstants.COMMAND_SEND_IR);
        serialConnection.sendByte(byteValue);
        serialConnection.afterCommandPause();
    }

    /**
     *This command requests the OI to send a packet of sensor data bytes. There
     * are 43 different sensor data packets. Each provides a value of a specific
     * sensor or group of sensors.
     * 
     * @param packetId
     *            One of the IRobotCreateConstants.SENSORS_* constants
     * @param eventHandler
     */
    public synchronized void readSensors(int packetId)
    {
        serialConnection.sendByte(RoombaConstants.COMMAND_SENSORS);
        serialConnection.sendByte(packetId);
        serialConnection.afterCommandPause();
        readSensorData(packetId);
    }

    private void setSensorBoolean(int sensorId, boolean value)
    {
        int intValue = value ? 1 : 0;
        sensorValues[sensorId] = intValue;
    }

    private void setSensorInteger(int sensorId, int value)
    {
        if (sensorId == 25)
        {
        }
        sensorValues[sensorId] = value;
    }

    /**
     * This command returns the values of a previously stored script, the
     * script’s commands and data bytes. It first halts the sensor stream, if
     * one has been started with a Stream or Pause/Resume Stream command. To
     * restart the stream, send Pause/Resume Stream (opcode 150).
     * 
     * @return
     */
    public synchronized int[] showScript()
    {
        serialConnection.sendByte(RoombaConstants.COMMAND_SHOW_SCRIPT);
        serialConnection.afterCommandPause();
        int size = serialConnection.readUnsignedByte();
        int[] script = new int[size];
        serialConnection.readUnsignedBytes(script, 0, size);
        return script;
    }

    /**
     * This command lets you specify up to sixteen songs to the OI that you can
     * play at a later time. Each song is associated with a song number. The
     * Play command uses the song number to identify your song selection. Each
     * song can contain up to sixteen notes. Each note is associated with a note
     * number that uses MIDI note definitions and a duration that is specified
     * in fractions of a second. <br>
     * • Available in modes: Passive, Safe, or Full <br>
     * • Changes mode to: No Change <br>
     * 
     * @param songNumber
     *            (0 – 15) <br>
     *            The song number associated with the specific song. If you send
     *            a second Song command, using the same song number, the old
     *            song is overwritten. <br>
     * @param notesAndDurations every even index represents a note, every odd index is the duration of that note.
     *            The pitch of the musical note Create will play, according to
     *            the MIDI note numbering scheme. The lowest musical note that
     *            Create will play is Note #31. Create considers all musical
     *            notes outside the range of IRobotCreateConstants.NOTE_FIRSTG –
     *            IRobotCreateConstnats.NOTE_LAST (31 – 127) as rest notes, and
     *            will make no sound during the duration of those notes. <br>
     *            The duration of a musical note, in increments of 1/64th of a
     *            second. Example: a half-second long musical note has a
     *            duration value of 32
     */
    public synchronized void song(int songNumber, int[] notesAndDurations)
    {
        song(songNumber, notesAndDurations, 0, notesAndDurations.length);
    }

    /**
     * This command lets you specify up to sixteen songs to the OI that you can
     * play at a later time. Each song is associated with a song number. The
     * Play command uses the song number to identify your song selection. Each
     * song can contain up to sixteen notes. Each note is associated with a note
     * number that uses MIDI note definitions and a duration that is specified
     * in fractions of a second. <br>
     * • Available in modes: Passive, Safe, or Full <br>
     * • Changes mode to: No Change <br>
     * 
     * @param songNumber
     *            (0 – 15) <br>
     *            The song number associated with the specific song. If you send
     *            a second Song command, using the same song number, the old
     *            song is overwritten. <br>
     * @param notesAndDurations every even index represents a note, every odd index is the duration of that note.
     *            The pitch of the musical note Create will play, according to
     *            the MIDI note numbering scheme. The lowest musical note that
     *            Create will play is Note #31. Create considers all musical
     *            notes outside the range of IRobotCreateConstants.NOTE_FIRSTG –
     *            IRobotCreateConstnats.NOTE_LAST (31 – 127) as rest notes, and
     *            will make no sound during the duration of those notes. <br>
     *            The duration of a musical note, in increments of 1/64th of a
     *            second. Example: a half-second long musical note has a
     *            duration value of 32
     * @param startIndex
     * @param length number of bytes to send from notesAndDuration, must be even number
     */
    private synchronized void song(int songNumber, int[] notesAndDurations, int startIndex, int length)
    {
        if (songNumber < 0 || songNumber > 15)
        {
            throw new IllegalArgumentException("songNumber " + songNumber);
        }
        if ((length & 0x01) == 0x01)
        {
            throw new IllegalArgumentException("length " + songNumber + "must be even");
        }
        if (length < 1 || length > (256 - (songNumber * 16 * 2)))
        {
            throw new IllegalArgumentException("length " + length);
        }
        serialConnection.sendByte(RoombaConstants.COMMAND_SONG);
        serialConnection.sendByte(songNumber);
        serialConnection.sendByte(length >> 1);
        serialConnection.sendBytes(notesAndDurations, startIndex, length);
        serialConnection.afterCommandPause();
    }

    /**
     * Create covers an area around its starting position by spiraling outward,
     * then inward.
     */
    public synchronized void spot()
    {
        serialConnection.sendByte(RoombaConstants.COMMAND_DEMO_SPOT_COVER);
        serialConnection.afterCommandPause();
    }

    /**
     * Vic's addition of waitAngle()
     * March 10, 2010
     *
     * Causes the iRobot Create to wait until it has rotated through the
     * specified andle in degrees.  When the Create turns counterclockwise,
     * the angle is inccremented.  When the Create turns clockwise,
     * the angle is decremented.  Until the Create turns through the
     * specified angle, its state does not change, nor does it react
     * to any serial or other inputs.
     */
    public synchronized void waitAngle(int angleToTurn)
    {
        serialConnection.sendByte(RoombaConstants.COMMAND_SCRIPT);
        serialConnection.sendByte(3); // This script length.
        serialConnection.sendByte(RoombaConstants.COMMAND_WAIT_ANGLE);
        serialConnection.sendSignedWord(angleToTurn);
//        serialConnection.sendByte(angleToTurn >> 8);
//        serialConnection.sendByte(angleToTurn);
        serialConnection.afterCommandPause();

        serialConnection.sendByte(RoombaConstants.COMMAND_PLAY_SCRIPT);
        serialConnection.afterCommandPause();
    }

    /**
     * Pause Java execution until either the play or the advance button is pressed.
     * While waiting, blink the LED next to the desired button.
     *
     * After the button is pressed, the power LED is turned on and the other
     * LEDs are turned off.
     * 
     * @param playButton If true wait for the play button, otherwise wait for
     *          the advance button
     * @param beep If true, beep while waiting for the button to be pressed.
     */
    public synchronized void waitButtonPressed(boolean playButton, boolean beep)
    {
        int totalTimeWaiting = 0;
        boolean gotButtonDown = false;
        final int noteDuration = 16;
        if (beep)
        {
            song(0, new int[]
                    {
                        54, 8,
                        55, 8,
                        0, 16,
                        56, 8,
                        57, 8
                    });
        }
        boolean isLedOn = true;
        if (playButton)
        {
            leds(true, isLedOn, false);
        } else
        {
            leds(true, false, isLedOn);
        }
        while (true)
        {
            readSensors(RoombaConstants.SENSORS_BUTTONS);
            if (gotButtonDown && playButton && !isPlayButtonDown())
            {
                break;
            }
            if (gotButtonDown && !playButton && !isAdvanceButtonDown())
            {
                break;
            }
            if (playButton && isPlayButtonDown())
            {
                gotButtonDown = true;
            }
            if (!playButton && isAdvanceButtonDown())
            {
                gotButtonDown = true;
            }
            Utils.sleep(noteDuration);
            totalTimeWaiting += noteDuration;
            if (totalTimeWaiting > 500)
            {
                if (beep)
                {
                    playSong(0);
                }
                isLedOn = !isLedOn;
                if (playButton)
                {
                    leds(true, isLedOn, false);
                } else
                {
                    leds(true, false, isLedOn);
                }
                totalTimeWaiting = 0;
            }
        }
        leds(true, false, false);
    }
}
