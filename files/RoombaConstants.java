// $Id: RoombaConstants.java 3609 2010-02-15 20:05:39Z vic $
package org.sunspotworld;

/**
 * All constants publicly available for use with the {@link IRobotCreate} class.
 * The constants were pulled out in order to make the code completion on an
 * instance of the {@link IRobotCreate} to be less cluttered.<br>
 * May not be the best option, but there are a lot of these constants :)
 * 
 */
public interface RoombaConstants {
    public static final int MAX_COMMAND_SIZE = 100;
    
    /**
     * Undocumented command that resets the Create and restarts it.
     */
    public static final int COMMAND_RESET = 7;
    /**
     * This command starts the OI. You must always send the Start command before sending
     * any other commands to the OI.
     */
    public static final int COMMAND_START = 128;
    /**
     * This command sets the baud rate in bits per second (bps) at which OI commands and
     * data are sent according to the baud code sent in the data byte.
     * 
     * Do not use this command, as we bring the baud rate down to the highest the Sun SPOT
     * can handle, 19, 200.
     */
    public static final int COMMAND_BAUD = 129;
    public static final int COMMAND_MODE_CONTROL = 130;
    /**
     * This command gives you complete control over Create by putting the OI into Full mode,
     * and turning off the cliff, wheel-drop and internal charger safety features. That is, in 
     * Full mode, Create executes any command that you send it, even if the internal charger 
     * is plugged in, or the robot senses a cliff or wheel drop.
     */
    public static final int COMMAND_MODE_SAFE = 131;
    /**
     * This command puts the OI into Safe mode, enabling user control of Create. It turns off
     * all LEDs. The OI can be in Passive, Safe, or Full mode to accept this command.
    */
    public static final int COMMAND_MODE_FULL = 132;
    /**
     * This command starts the Spot Cover demo.
     */
    public static final int COMMAND_DEMO_SPOT_COVER = 134;
    /**
     * This command starts the Cover demo.
     */
    public static final int COMMAND_DEMO_COVER = 135;
    /**
     * This command starts the requested built-in demo.  Demo can be specified using one
     * of the DEMO_* constants.
     */
    public static final int COMMAND_DEMO = 136;
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
     * of curvature. <br>
     * • Available in modes: Safe or Full <br>
     * • Changes mode to: No Change <br>
     * • Velocity (-500 – 500 mm/s) <br>
     * • Radius (-2000 – 2000 mm) <br>
     * Special cases: <br>
     * Straight = 32768 or 32767 = hex 8000 or 7FFF <br>
     * Turn in place clockwise = hex FFFF <br>
     * Turn in place counter-clockwise = hex 0001
     */
    public static final int COMMAND_DRIVE = 137;
    public static final int COMMAND_LOW_SIDE_DRIVERS = 138;
    public static final int COMMAND_LEDS = 139;
    public static final int COMMAND_SONG = 140;
    public static final int COMMAND_PLAY_SONG = 141;
    public static final int COMMAND_SENSORS = 142;
    /**
     * This command starts the Cover and Dock demo.
     */
    public static final int COMMAND_DEMO_COVER_AND_DOCK = 143;
    public static final int COMMAND_PWM_LOW_SIDE_DRIVERS = 144;
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
     * • Changes mode to: No Change <br>
     * • Right wheel velocity (-500 – 500 mm/s) <br>
     * • Left wheel velocity (-500 – 500 mm/s)
     */
    public static final int COMMAND_DRIVE_DIRECT = 145;
    public static final int COMMAND_DIGITAL_OUTPUTS = 147;
    public static final int COMMAND_STREAM = 148;
    public static final int COMMAND_QUERY_LIST = 149;
    public static final int COMMAND_PAUSE_RESUME_STREAM = 150;
    public static final int COMMAND_SEND_IR = 151;
    public static final int COMMAND_SCRIPT = 152;
    public static final int COMMAND_PLAY_SCRIPT = 153;
    public static final int COMMAND_SHOW_SCRIPT = 154;
    public static final int COMMAND_WAIT_TIME = 155;
    public static final int COMMAND_WAIT_DISTANCE = 156;
    public static final int COMMAND_WAIT_ANGLE = 157;
    public static final int COMMAND_WAIT_EVENT = 158;

    public static final int DRIVE_STRAIGHT = 0x8000;
    public static final int DRIVE_TURN_IN_PLACE_CLOCKWISE = 0xFFFF;
    public static final int DRIVE_TURN_IN_PLACE_COUNTER_CLOCKWISE = 0x0001;
    public static final int DRIVE_MAX_VELOCITY = 500;
    public static final int DRIVE_MAX_RADIUS = 2000;

    public static final int DEMO_ABORT = 255;
    /**
     * Create attempts to cover an entire room using a combination of behaviors, such as 
     * random bounce, wall following, and spiraling.
     */
    public static final int DEMO_COVER = 0;
    /**
     * Identical to the Cover demo, with one exception. If Create sees an infrared signal from 
     * an iRobot Home Base, it uses that signal to dock with the Home Base and recharge itself.
     */
    public static final int DEMO_COVER_AND_DOCK = 1;
    /**
     * Create covers an area around its starting position by spiraling outward, then inward.
     */
    public static final int DEMO_SPOT_COVER = 2;
    /**
     * Create drives in search of a wall. Once a wall is found, Create drives along the wall, 
     * traveling around circumference of the room.
     */
    public static final int DEMO_MOUSE = 3;
    /**
     * Create continuously drives in a figure 8 pattern.
     */
    public static final int DEMO_DRIVE_FIGURE_EIGHT = 4;
    /**
     * Create drives forward when pushed from behind. If Create hits an obstacle while
     * driving, it drives away from the obstacle.
     */
    public static final int DEMO_WIMP = 5;
    /**
     * Create drives toward an iRobot Virtual Wall as long as the back and sides of the virtual 
     * wall receiver are blinded by black electrical tape.
     * <br>
     * A Virtual Wall emits infrared signals that Create sees with its Omnidirectional Infrared 
     * Receiver, located on top of the bumper.
     * <br>
     * If you want Create to home in on a Virtual Wall, cover all but a small opening in the 
     * front of the infrared receiver with black electrical tape.
     * <br>
     * Create spins to locate a virtual wall, then drives toward it. Once Create hits the wall or 
     * another obstacle, it stops.
     */
    public static final int DEMO_HOME = 6;
    /**
     * Identical to the Home demo, except Create drives into multiple virtual
     * walls by bumping into one, turning around, driving to the next virtual
     * wall, bumping into it and turning around to bump into the next virtual
     * wall.
     */
    public static final int DEMO_TAG = 7;
    /**
     * Create plays the notes of Pachelbel’s Canon in sequence when cliff
     * sensors are activated.
     */
    public static final int DEMO_PACHEBEL = 8;
    /**
     * Create plays a note of a chord for each of its four cliff sensors. Select
     * the chord using the bumper, as follows: <br>
     * • No bumper: G major. <br>
     * • Right/left bumper: D major 7 <br>
     * • Both bumpers (center): C major
     */
    public static final int DEMO_BANJO = 9;

    public static final int EVENT_WHEEL_DROP = 1;
    public static final int EVENT_FRONT_WHEEL_DROP = 2;
    public static final int EVENT_LEFT_WHEEL_DROP = 3;
    public static final int EVENT_RIGHT_WHEEL_DROP = 4;
    public static final int EVENT_BUMP = 5;
    public static final int EVENT_LEFT_BUMP = 6;
    public static final int EVENT_RIGHT_BUMP = 7;
    public static final int EVENT_VIRTUAL_WALL = 8;
    public static final int EVENT_WALL = 9;
    public static final int EVENT_CLIFF = 10;
    public static final int EVENT_LEFT_CLIFF = 11;
    public static final int EVENT_FRONT_LEFT_CLIFF = 12;
    public static final int EVENT_FRONT_RIGHT_CLIFF = 13;
    public static final int EVENT_RIGHT_CLIFF = 14;
    public static final int EVENT_HOME_BASE = 15;
    public static final int EVENT_ADVANCE_BUTTON = 16;
    public static final int EVENT_PLAY_BUTTON = 17;
    public static final int EVENT_DIGITAL_INPUT_0 = 18;
    public static final int EVENT_DIGITAL_INPUT_1 = 19;
    public static final int EVENT_DIGITAL_INPUT_2 = 20;
    public static final int EVENT_DIGITAL_INPUT_3 = 21;
    public static final int EVENT_OI_MODE_PASSIVE = 22;

    public static final int SENSORS_GROUP_ID0 = 0;
    public static final int SENSORS_GROUP_ID1 = 1;
    public static final int SENSORS_GROUP_ID2 = 2;
    public static final int SENSORS_GROUP_ID3 = 3;
    public static final int SENSORS_GROUP_ID4 = 4;
    public static final int SENSORS_GROUP_ID5 = 5;
    public static final int SENSORS_GROUP_ID6 = 6;
    public static final int SENSORS_BUMPS_AND_WHEEL_DROPS = 7;
    public static final int SENSORS_WALL = 8;
    public static final int SENSORS_CLIFF_LEFT = 9;
    public static final int SENSORS_CLIFF_FRONT_LEFT = 10;
    public static final int SENSORS_CLIFF_FRONT_RIGHT = 11;
    public static final int SENSORS_CLIFF_RIGHT = 12;
    public static final int SENSORS_VIRTUAL_WALL = 13;
    public static final int SENSORS_LOWS_SIDE_DRIVER_AND_WHEEL_OVERCURRENTS = 14;
    public static final int SENSORS_UNUSED1 = 15;
    public static final int SENSORS_UNUSED2 = 16;
    public static final int SENSORS_INFRARED_BYTE = 17;
    public static final int SENSORS_BUTTONS = 18;
    public static final int SENSORS_DISTANCE = 19;
    public static final int SENSORS_ANGLE = 20;
    public static final int SENSORS_CHARGING_STATE = 21;
    public static final int SENSORS_VOLTAGE = 22;
    public static final int SENSORS_CURRENT = 23;
    public static final int SENSORS_BATTERY_TEMPERATURE = 24;
    public static final int SENSORS_BATTERY_CHARGE = 25;
    public static final int SENSORS_BATTERY_CAPACITY = 26;
    public static final int SENSORS_WALL_SIGNAL = 27;
    public static final int SENSORS_CLIFF_LEFT_SIGNAL = 28;
    public static final int SENSORS_CLIFF_FRONT_LEFT_SIGNAL = 29;
    public static final int SENSORS_CLIFF_FRONT_RIGHT_SIGNAL = 30;
    public static final int SENSORS_CLIFF_RIGHT_SIGNAL = 31;
    public static final int SENSORS_CARGO_BAY_DIGITAL_INPUTS = 32;
    public static final int SENSORS_CARGO_BAY_ANALOG_SIGNAL = 33;
    public static final int SENSORS_CHARGING_SOURCES_AVAILABLE = 34;
    public static final int SENSORS_OI_MODE = 35;
    public static final int SENSORS_SONG_NUMBER = 36;
    public static final int SENSORS_SONG_PLAYING = 37;
    public static final int SENSORS_NUMBER_OF_STREAM_PACKETS = 38;
    public static final int SENSORS_REQUESTED_VELOCITY = 39;
    public static final int SENSORS_REQUESTED_RADIUS = 40;
    public static final int SENSORS_REQUESTED_RIGHT_VELOCITY = 41;
    public static final int SENSORS_REQUESTED_LEFT_VELOCITY = 42;
    
    public static final int DIGITAL_OUTPUT_PIN0 = 1;
    public static final int DIGITAL_OUTPUT_PIN1 = 2;
    public static final int DIGITAL_OUTPUT_PIN2 = 4;

    public static final int LOW_SIDE_DRIVER0 = 1;
    public static final int LOW_SIDE_DRIVER1 = 2;
    public static final int LOW_SIDE_DRIVER2 = 4;

    public static final int INFRARED_BYTE_REMOTE_CONTROL_LEFT = 129;
    public static final int INFRARED_BYTE_REMOTE_CONTROL_FORWARD = 130;
    public static final int INFRARED_BYTE_REMOTE_CONTROL_RIGHT = 131;
    public static final int INFRARED_BYTE_REMOTE_CONTROL_SPOT = 132;
    public static final int INFRARED_BYTE_REMOTE_CONTROL_MAX = 133;
    public static final int INFRARED_BYTE_REMOTE_CONTROL_SMALL = 134;
    public static final int INFRARED_BYTE_REMOTE_CONTROL_MEDIUM = 135;
    public static final int INFRARED_BYTE_REMOTE_CONTROL_LARGE_CLEAN = 136;
    public static final int INFRARED_BYTE_REMOTE_CONTROL_PAUSE = 137;
    public static final int INFRARED_BYTE_REMOTE_CONTROL_POWER = 138;
    public static final int INFRARED_BYTE_REMOTE_CONTROL_ARC_FORWARD_LEFT = 139;
    public static final int INFRARED_BYTE_REMOTE_CONTROL_ARC_FORWARD_RIGHT = 140;
    public static final int INFRARED_BYTE_REMOTE_CONTROL_DRIVE_STOP = 141;
    public static final int INFRARED_BYTE_SCHEDULING_REMOTE_CONTROL_SEND_ALL = 142;
    public static final int INFRARED_BYTE_SCHEDULING_REMOTE_CONTROL_SEEK_DOCK = 143;
    public static final int INFRARED_BYTE_HOME_BASE_RESERVED = 240;
    public static final int INFRARED_BYTE_HOME_BASE_RED_BUOY = 248;
    public static final int INFRARED_BYTE_HOME_BASE_GREEN_BUOY = 244;
    public static final int INFRARED_BYTE_HOME_BASE_FORCE_FIELD = 242;
    public static final int INFRARED_BYTE_HOME_BASE_RED_GREEN_BUOY = 252;
    public static final int INFRARED_BYTE_HOME_BASE_RED_BUOY_FORCE_FIELD = 250;
    public static final int INFRARED_BYTE_HOME_BASE_GREEN_BUOY_FORCE_FIELD = 246;
    public static final int INFRARED_BYTE_HOME_BASE_RED_GREEN_BUOY_FORCE_FIELD = 254;
    public static final int INFRARED_BYTE_NONE = 255;
    
    public static final int OI_MODE_OFF = 0;
    public static final int OI_MODE_PASSIVE = 1;
    public static final int OI_MODE_SAFE = 2;
    public static final int OI_MODE_FULL = 3;

    public static final int CHARGING_STATE_NOT_CHARGING = 0;
    public static final int CHARGING_STATE_RECONDITIONING_CHARGING = 1;
    public static final int CHARGING_STATE_FULL_CHARGING = 2;
    public static final int CHARGING_STATE_TRICKLE_CHARGING = 3;
    public static final int CHARGING_STATE_WAITING = 4;
    public static final int CHARGING_STATE_CHARGING_FAULT_CONDITION = 5;
    
    public static final int NOTE_FIRST_G = 31;
    public static final int NOTE_FIRST_GSHARP = 32;
    public static final int NOTE_FIRST_A = 33;
    public static final int NOTE_FIRST_ASHARP = 34;
    public static final int NOTE_FIRST_B = 35;
    public static final int NOTE_FIRST_C = 36;
    public static final int NOTE_FIRST_CSHARP = 37;
    public static final int NOTE_FIRST_D = 38;
    public static final int NOTE_FIRST_DSHARP = 39;
    public static final int NOTE_FIRST_E = 40;
    public static final int NOTE_FIRST_F = 41;
    public static final int NOTE_FIRST_FSHARP = 42;
    public static final int NOTE_LAST = 126;
    public static final int NOTE_NUMBER_OF_OCTAVES = 8;
    public static final int NOTE_OCTAVE_SIZE = 12;
    
    public static final int BAUD_RATE_300 = 0;
    public static final int BAUD_RATE_600 = 1;
    public static final int BAUD_RATE_1200 = 2;
    public static final int BAUD_RATE_2400 = 3;
    public static final int BAUD_RATE_4800 = 4;
    public static final int BAUD_RATE_9600 = 5;
    public static final int BAUD_RATE_14400 = 6;
    public static final int BAUD_RATE_19200 = 7;
    public static final int BAUD_RATE_28800 = 8;
    public static final int BAUD_RATE_38400 = 9;
    public static final int BAUD_RATE_57600 = 10;
    public static final int BAUD_RATE_115200 = 11;

    public static final int AFTER_COMMAND_PAUSE_TIME = 20;

}
