// $Id: RoombaSerialConnection.java 3609 2010-02-15 20:05:39Z vic $
package org.sunspotworld;

import java.io.IOException;

import com.sun.spot.peripheral.TimeoutException;
import com.sun.spot.sensorboard.EDemoBoard;
import com.sun.spot.sensorboard.io.IIOPin;
import com.sun.spot.util.BootloaderListener;
import com.sun.spot.util.Utils;
import com.sun.squawk.util.UnexpectedException;

/**
 * Represents the top level class used to communicate with the Create. We did
 * have an interface definint this interface at one point, but it got removed to
 * "simplify" the code, reduce number of classes.
 * 
 * The communication channel is seperated out in order for us to be able to
 * support different modes of connecting to the Create.
 * 
 */
class RoombaSerialConnection
{
    private final EDemoBoard demoBoard;
    private final IIOPin baudRateSelectPin;
    private byte[] uartBuffer;

    protected RoombaSerialConnection(EDemoBoard demoBoard, IIOPin baudRateSelectPin)
    {
        this.demoBoard = demoBoard;
        this.baudRateSelectPin = baudRateSelectPin;
        new BootloaderListener().start();
        uartBuffer = new byte[RoombaConstants.MAX_COMMAND_SIZE];
        try
        {
            connectToCreate();
        } catch (UnexpectedException e)
        {
            Utils.sleep(2500);
            connectToCreate();
        }
    }

    protected void connectToCreate()
    {
        if (baudRateSelectPin == null)
        {
            demoBoard.initUART(EDemoBoard.SERIAL_SPEED_57600, false);
            demoBoard.writeUART((byte)RoombaConstants.COMMAND_BAUD);
            demoBoard.writeUART((byte)RoombaConstants.BAUD_RATE_19200);
            Utils.sleep(100);
            demoBoard.initUART(EDemoBoard.SERIAL_SPEED_19200, false);
            Utils.sleep(200);
        } else
        {
            demoBoard.initUART(EDemoBoard.SERIAL_SPEED_19200, false);
            baudRateSelectPin.setAsOutput(true);
            for (int i = 0; i < 4; i++)
            {
                baudRateSelectPin.setLow();
                Utils.sleep(50);
                baudRateSelectPin.setHigh();
                Utils.sleep(50);
            }
            baudRateSelectPin.setLow();
        }
        int readCount = 0;
        int errorCount = 0;
        while (true)
        {
            try
            {
                int read = demoBoard.readUART(100);
                readCount++;
            } catch (TimeoutException e)
            {
                break;
            } catch (IOException e)
            {
                errorCount++;
            }
        }
        final int numberOfStartsToSend = RoombaConstants.MAX_COMMAND_SIZE;
//        final int numberOfStartsToSend = 1;
        for (int i = 0; i < numberOfStartsToSend; i++)
        {
            uartBuffer[i] = (byte)RoombaConstants.COMMAND_START;
        }
        sendBytes(uartBuffer, 0, numberOfStartsToSend);
        while (true)
        {
            uartBuffer[0] = (byte)RoombaConstants.COMMAND_SENSORS;
            uartBuffer[1] = (byte)RoombaConstants.SENSORS_OI_MODE;
            sendBytes(uartBuffer, 0, 2);
            Utils.sleep(100);
            int mode = readUnsignedByte();
            if (mode == RoombaConstants.OI_MODE_PASSIVE)
            {
                break;
            }
        }
    }

    public int readSignedByte()
    {
        try
        {
            int result = demoBoard.readUART();
            return result;
        } catch (IOException e)
        {
            throw new UnexpectedException(e);
        }
    }

    public int readUnsignedByte()
    {
        try
        {
            int result = demoBoard.readUART() & 0xFF;
            return result;
        } catch (IOException e)
        {
            throw new UnexpectedException(e);
        }
    }

    public int readSignedWord()
    {
        try
        {
            int high = demoBoard.readUART();
            int low = demoBoard.readUART();
            // Java is already twos complement, so no need for any translation
            int signed = (high << 8) | (low & 0xFF);
            return signed;
        } catch (IOException e)
        {
            throw new UnexpectedException(e);
        }
    }

    public int readUnsignedBytes(int[] buffer, int start, int length)
    {
        if (length > uartBuffer.length)
        {
            uartBuffer = new byte[length];
        }
        try
        {
            int readCount = demoBoard.readUART(uartBuffer, 0, length);
            for (int i = 0; i < length; i++)
            {
                buffer[start + i] = uartBuffer[i] & 0xFF;
            }
            return readCount;
        } catch (IOException e)
        {
            throw new UnexpectedException(e);
        }
    }

    public int readUnsignedWord()
    {
        try
        {
            int high = demoBoard.readUART();
            int low = demoBoard.readUART();
            int unsigned = ((high & 0xFF) << 8) | (low & 0xFF);
            return unsigned;
        } catch (IOException e)
        {
            throw new UnexpectedException(e);
        }
    }

    public void sendByte(int b)
    {
        demoBoard.writeUART((byte)b);
        Utils.sleep(1);
    }

    public void sendBytes(byte[] bytes, int start, int length)
    {
        demoBoard.writeUART(bytes, start, length);
    }

    public void sendBytes(int[] bytes, int start, int length)
    {
        if (length > uartBuffer.length)
        {
            uartBuffer = new byte[length];
        }
        for (int i = 0; i < length; i++)
        {
            uartBuffer[i] = (byte)bytes[start + i];
        }
        demoBoard.writeUART(uartBuffer, 0, length);
    }

    public void sendSignedWord(int value)
    {
        // Java bit representation is already two's complement
        uartBuffer[0] = (byte)(value >> 8);
        uartBuffer[1] = (byte)(value & 0xFF);
        demoBoard.writeUART(uartBuffer, 0, 2);
    }

    public void sendUnsignedWord(int value)
    {
        uartBuffer[0] = (byte)(value >> 8);
        uartBuffer[1] = (byte)(value & 0xFF);
        demoBoard.writeUART(uartBuffer, 0, 2);
    }

    public void afterCommandPause()
    {
        Utils.sleep(RoombaConstants.AFTER_COMMAND_PAUSE_TIME);
    }
}
