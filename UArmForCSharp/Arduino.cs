/*
 * Arduino.cs - Arduino/firmata library for Visual C# .NET
 * Copyright (C) 2009 Tim Farley
 * 
 * Special thanks to David A. Mellis, on whose Processing library
 * this code is based.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General
 * Public License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place, Suite 330,
 * Boston, MA  02111-1307  USA
 *
 *
 * ***************** *
 * TODO/KNOWN ISSUES *
 * ***************** *
 * Exception Handling: At this time there is no exception handling.
 * It should be trivial to add exception handling as-needed.
 * 
 * $Id$
 */
using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace EVOL.NET
{

    /**
 * Together with the Firmata 2 firmware (an Arduino sketch uploaded to the
 * Arduino board), this class allows you to control the Arduino board from
 * Processing: reading from and writing to the digital pins and reading the
 * analog inputs.
 */

    public class Arduino
    {
        public static int INPUT = 0;
        public static int OUTPUT = 1;
        public static int LOW = 0;
        public static int HIGH = 1;
        public static int SERVO = 4;
        private const int MAX_DATA_BYTES = 512;

        private const int DIGITAL_MESSAGE = 0x90; // send data for a digital port
        private const int ANALOG_MESSAGE = 0xE0; // send data for an analog pin (or PWM)
        private const int REPORT_ANALOG = 0xC0; // enable analog input by pin #
        private const int REPORT_DIGITAL = 0xD0; // enable digital input by port
        private const int SET_PIN_MODE = 0xF4; // set a pin to INPUT/OUTPUT/PWM/etc
        private const int REPORT_VERSION = 0xF9; // report firmware version
        private const int SYSTEM_RESET = 0xFF; // reset from MIDI
        private const int START_SYSEX = 0xF0; // start a MIDI SysEx message
        private const int END_SYSEX = 0xF7; // end a MIDI SysEx message

        private const int EEPROM_READ = 0x40;
        private const int EEPROM_WRITE = 0x41;
        private const int EEPROM_READ_RESPONSE = 0x42;

        private int[] eeprom_data;

        private SerialPort _serialPort;
        private int delay;

        private int waitForData = 0;
        private int executeMultiByteCommand = 0;
        private int multiByteChannel = 0;
        private int[] storedInputData = new int[MAX_DATA_BYTES];
        private bool parsingSysex;
        private int sysexBytesRead;

        private volatile int[] digitalOutputData = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        private volatile int[] digitalInputData = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        private volatile int[] analogInputData = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

        private int majorVersion = 0;
        private int minorVersion = 0;
        private Thread readThread = null;
        private object locker = new object();

        private bool isDebug = false;

        /// <summary>
        /// 
        /// </summary>
        /// <param name="serialPortName">String specifying the name of the serial port. eg COM4</param>
        /// <param name="baudRate">The baud rate of the communication. Default 115200</param>
        /// <param name="delay">Time delay that may be required to allow some arduino models
        ///                     to reboot after opening a serial connection. The delay will only activate
        ///                     when autoStart is true.</param>
        public Arduino(string serialPortName, Int32 baudRate, int delay)
        {
            _serialPort = new SerialPort(serialPortName, baudRate);
            _serialPort.Parity = Parity.None;
            _serialPort.StopBits = StopBits.One;
            _serialPort.DataBits = 8;
            _serialPort.Handshake = Handshake.None;
            _serialPort.RtsEnable = true;
            _serialPort.DtrEnable = true;
            isDebug = true;
            this.delay = delay;
            _serialPort.DataReceived += new SerialDataReceivedEventHandler(DataReceivedHandler);
            _serialPort.Open();
            Thread.Sleep(delay);
            Init();

        }

        /// <summary>
        /// Creates an instance of the Arduino object, based on a user-specified serial port.
        /// Assumes default values for baud rate (115200) and reboot delay (8 seconds)
        /// and automatically opens the specified serial connection.
        /// </summary>
        /// <param name="serialPortName">String specifying the name of the serial port. eg COM4</param>
        public Arduino(string serialPortName) : this(serialPortName, 115200,  4000) { }

        /// <summary>
        /// Creates an instance of the Arduino object, based on user-specified serial port and baud rate.
        /// Assumes default value for reboot delay (8 seconds).
        /// and automatically opens the specified serial connection.
        /// </summary>
        /// <param name="serialPortName">String specifying the name of the serial port. eg COM4</param>
        /// <param name="baudRate">Baud rate.</param>
        public Arduino(string serialPortName, Int32 baudRate) : this(serialPortName, baudRate,  4000) { }

        /// <summary>
        /// Creates an instance of the Arduino object using default arguments.
        /// Assumes the arduino is connected as the HIGHEST serial port on the machine,
        /// default baud rate (115200), and a reboot delay (8 seconds).
        /// and automatically opens the specified serial connection.
        /// </summary>
        public Arduino() : this(Arduino.list().ElementAt(list().Length - 1), 115200, 4000) { }


        /// <summary>
        /// Data Receive Handler
        /// </summary>
        /// <param name="sender"></param>
        /// <param name="e"></param>
        private void DataReceivedHandler(
                    object sender,
                    SerialDataReceivedEventArgs e)
        {
            SerialPort sp = (SerialPort)sender;
            if (isDebug)
                Console.Write("Read Bytes: ");
            while (sp.BytesToRead > 0)
            {
                int indata = sp.ReadByte();
                if (isDebug)
                    Console.Write(indata.ToString("X2"));
                processInput(indata);
            }
            if (isDebug)
                Console.WriteLine();


        }


        /// <summary>
        /// Get All Available Port names
        /// </summary>
        /// <returns></returns>
        public static string[] list()
        {
            return SerialPort.GetPortNames();
        }

        ///<summary>
        /// init Firmata Arduino
        /// </summary>
        private void Init()
        {
            eeprom_data = new int[65536];
        }

        ///<summary>
        /// query EEPROM Data
        /// <param name="addr"> EEPROM Address 1 - 65536 </param>
        /// </summary>
        private void QueryEepromData(int addr)
        {
            byte[] b = new byte[5];
            b[0] = (byte)START_SYSEX;
            b[1] = (byte)EEPROM_READ;
            b[2] = (byte)(addr & 0x7F);
            b[3] = (byte)(addr >> 7);
            b[4] = (byte)END_SYSEX;
            _serialPort.Write(b, 0, b.Length);
            if (isDebug)
                Console.WriteLine("QueryEepromData:" + BitConverter.ToString(b).Replace("-", string.Empty));
        }


        /// <summary>
        /// Read Data from EEPROM
        /// </summary>
        /// <param name="addr">EEPROM ADDRESS 1 - 65536 </param>
        /// <returns></returns>
        public int EepromRead(int addr)
        {
            QueryEepromData(addr);
            Thread.Sleep(100);
            return GetEepromData(addr);
        }

        /// <summary>
        /// Write Data to EEPROM
        /// </summary>
        /// <param name="addr">EEPROM ADDERESS 1-65536</param>
        /// <param name="val">Data 0-255 </param>
        public void EepromWrite(int addr, int val)
        {
            byte[] b = new byte[7];
            b[0] = (byte)START_SYSEX;
            b[1] = (byte)EEPROM_WRITE;
            b[2] = (byte)(addr & 0x7F);
            b[3] = (byte)(addr >> 7);
            b[4] = (byte)(val & 0x7F);
            b[5] = (byte)(val >> 7);
            b[6] = (byte)(END_SYSEX);
            _serialPort.Write(b, 0, b.Length);
            if (isDebug)
                Console.WriteLine("EepromWrite:" + BitConverter.ToString(b).Replace("-", string.Empty));
        }

        /// <summary>
        /// Get EEPROM Data from array
        /// </summary>
        /// <param name="addr">array index</param>
        /// <returns></returns>
        private int GetEepromData(int addr)
        {
            if (addr > 0 && addr < 65535)
            {
                return eeprom_data[addr];
            }
            else
            {
                return 0;
            }
        }

        /// <summary>
        /// Closes the serial port.
        /// </summary>
        public void Close()
        {
            _serialPort.Close();
        }

        /// <summary>
        /// Lists all available serial ports on current system.
        /// </summary>
        /// <returns>An array of strings containing all available serial ports.</returns>



        /// <summary>
        /// Returns the last known state of the digital pin.
        /// </summary>
        /// <param name="pin">The arduino digital input pin.</param>
        /// <returns>Arduino.HIGH or Arduino.LOW</returns>
        public int digitalRead(int pin)
        {
            return (digitalInputData[pin >> 3] >> (pin & 0x07)) & 0x01;
        }

        /// <summary>
        /// Returns the last known state of the analog pin.
        /// </summary>
        /// <param name="pin">The arduino analog input pin.</param>
        /// <returns>A value representing the analog value between 0 (0V) and 1023 (5V).</returns>
        public int analogRead(int pin)
        {
            return analogInputData[pin];
        }

        /// <summary>
        /// Sets the mode of the specified pin (INPUT or OUTPUT).
        /// </summary>
        /// <param name="pin">The arduino pin.</param>
        /// <param name="mode">Mode Arduino.INPUT or Arduino.OUTPUT.</param>
        public void pinMode(int pin, int mode)
        {
            byte[] message = new byte[3];
            message[0] = (byte)(SET_PIN_MODE);
            message[1] = (byte)(pin);
            message[2] = (byte)(mode);
            _serialPort.Write(message, 0, message.Length);
            message = null;
        }

        /// <summary>
        /// Write to a digital pin that has been toggled to output mode with pinMode() method.
        /// </summary>
        /// <param name="pin">The digital pin to write to.</param>
        /// <param name="value">Value either Arduino.LOW or Arduino.HIGH.</param>
        public void digitalWrite(int pin, int value)
        {
            int portNumber = (pin >> 3) & 0x0F;
            byte[] message = new byte[3];

            if (value == 0)
                digitalOutputData[portNumber] &= ~(1 << (pin & 0x07));
            else
                digitalOutputData[portNumber] |= (1 << (pin & 0x07));

            message[0] = (byte)(DIGITAL_MESSAGE | portNumber);
            message[1] = (byte)(digitalOutputData[portNumber] & 0x7F);
            message[2] = (byte)(digitalOutputData[portNumber] >> 7);
            _serialPort.Write(message, 0, 3);
        }

        /// <summary>
        /// Write to an Servo pin using analog
        /// </summary>
        public void servoWrite(int pin, int value)
        {
            int portNumber = (pin >> 3) & 0x0F;
            byte[] message = new byte[3];
            message[0] = (byte)(ANALOG_MESSAGE | (pin & 0x0F));
            message[1] = (byte)(value & 0x7F);
            message[2] = (byte)(value >> 7);
            _serialPort.Write(message, 0, 3);
            if(isDebug)Console.WriteLine("servoWrite: " + BitConverter.ToString(message).Replace("-", string.Empty));

        }

        /// <summary>
        /// Write to an analog pin using Pulse-width modulation (PWM).
        /// </summary>
        /// <param name="pin">Analog output pin.</param>
        /// <param name="value">PWM frequency from 0 (always off) to 255 (always on).</param>
        public void analogWrite(int pin, int value)
        {
            byte[] message = new byte[3];
            message[0] = (byte)(ANALOG_MESSAGE | (pin & 0x0F));
            message[1] = (byte)(value & 0x7F);
            message[2] = (byte)(value >> 7);
            _serialPort.Write(message, 0, message.Length);
        }

        private void setDigitalInputs(int portNumber, int portData)
        {
            digitalInputData[portNumber] = portData;
        }

        private void setAnalogInput(int pin, int value)
        {
            analogInputData[pin] = value;
        }

        private void setVersion(int majorVersion, int minorVersion)
        {
            this.majorVersion = majorVersion;
            this.minorVersion = minorVersion;
        }

        private int available()
        {
            return _serialPort.BytesToRead;
        }

        private void processInput(int inputData)
        {
            //int inputData = _serialPort.ReadByte();
            //Console.WriteLine("inputData: " + inputData.ToString("X"));
            int command;

            if (parsingSysex)
            {
                if (inputData == END_SYSEX)
                {
                    // Console.WriteLine("END_SYSEX");
                    parsingSysex = false;
                    processSysexMessage();
                }
                else
                {
                    //Console.WriteLine("sysexBytesRead: "+ sysexBytesRead);
                    storedInputData[sysexBytesRead] = inputData;
                    sysexBytesRead++;
                }
            }
            else if (waitForData > 0 && inputData < 128)
            {
                waitForData--;
                storedInputData[waitForData] = inputData;

                if (executeMultiByteCommand != 0 && waitForData == 0)
                {
                    //we got everything
                    switch (executeMultiByteCommand)
                    {
                        case DIGITAL_MESSAGE:
                            setDigitalInputs(multiByteChannel, (storedInputData[0] << 7) + storedInputData[1]);
                            break;
                        case ANALOG_MESSAGE:
                            setAnalogInput(multiByteChannel, (storedInputData[0] << 7) + storedInputData[1]);
                            break;
                        case REPORT_VERSION:
                            setVersion(storedInputData[1], storedInputData[0]);
                            break;
                    }
                }
            }
            else
            {
                if (inputData < 0xF0)
                {
                    command = inputData & 0xF0;
                    multiByteChannel = inputData & 0x0F;
                }
                else
                {
                    command = inputData;
                    // commands in the 0xF* range don't use channel data
                }
                switch (command)
                {
                    case DIGITAL_MESSAGE:

                    case ANALOG_MESSAGE:
                    case REPORT_VERSION:
                        waitForData = 2;
                        executeMultiByteCommand = command;
                        break;
                    case START_SYSEX:
                        parsingSysex = true;
                        sysexBytesRead = 0;
                        break;
                }
            }
        }

        private void processSysexMessage()
        {
            switch (storedInputData[0])
            {

                case EEPROM_READ_RESPONSE:
                    //Console.WriteLine("EEPROM_READ_RESPONSE");
                    int addr = (short)(storedInputData[1] + (storedInputData[2] << 7));
                    int data = (short)(storedInputData[3] + (storedInputData[4] << 7));
                    if (addr > 0 && addr < 65535)
                        eeprom_data[addr] = data;
                    break;
            }
        }

    }

    public class Theta
    {

        public double x = 0.0;
        public double y = 0.0;
        public double z = 0.0;
    }

    public class ActionControl
    {

        private static double MATH_PI = 3.141592653;
        private static double MATH_TRANS = 57.2958;
        private static double MATH_L1 = (10.645 + 0.6);
        private static double MATH_L2 = 2.117;
        private static double MATH_L3 = 14.825;
        private static double MATH_L4 = 16.02;

        private static double g_x_in;
        private static double g_y_in;
        private static double g_z_in;
        private static double g_l43;
        private static double g_right_all;
        private static double g_sqrt_z_y;
        //	private double theta.x;
        //	private double theta.y;
        //	private double theta.z;
        private static double g_phi;
        private static double g_right_all_2;
        private static double g_sqrt_z_x;
        private static double g_cal_x;
        private static double g_cal_y;
        private static double g_cal_z;

        private static double g_l3_1_2;
        private static double g_l4_1_2;
        private static double g_l5_2;

        private double last_theta1 = 90;
        private double last_theta2 = 45;
        private double last_theta3 = 45;
        private double last_number;
        

        public static Theta calculateServoAngles(double x, double y, double z)
        {
            Theta theta = new Theta();
            g_y_in = (-y - MATH_L2) / MATH_L3;
            g_z_in = (z - MATH_L1) / MATH_L3;
            g_l43 = MATH_L4 / MATH_L3;
            g_right_all = (1 - g_y_in * g_y_in - g_z_in * g_z_in - g_l43 * g_l43) / (2 * g_l43);
            g_sqrt_z_y = Math.Sqrt(g_z_in * g_z_in + g_y_in * g_y_in);

            if (z > (MATH_L1 + MATH_L3))
            {
                z = 25;
            }

            if (x == 0)
            {
                // Calculate value of theta 1
                theta.x = 90;

                // Calculate value of theta 3
                if (g_z_in == 0)
                {
                    g_phi = 90;
                }

                else
                {
                    g_phi = Math.Atan(-g_y_in / g_z_in) * MATH_TRANS;
                }

                theta.z = Math.Asin(g_right_all / g_sqrt_z_y) * MATH_TRANS - g_phi;
                if (theta.z > 90)
                {
                    theta.z = 180 - Math.Abs(theta.z);
                }
                // Calculate value of theta 2

                theta.y = Math.Asin((z + MATH_L4 * Math.Sin(theta.z / MATH_TRANS) - MATH_L1) / MATH_L3) * MATH_TRANS;
            }

            else
            {
                // Calculate value of theta 1

                theta.x = Math.Atan(y / x) * MATH_TRANS;

                if (y / x > 0)
                {
                    theta.x = theta.x + 0;
                }
                if (y / x < 0)
                {
                    theta.x = theta.x + 180;
                }
                if (y == 0)
                {
                    if (x > 0)
                        theta.x = 180;
                    else
                        theta.x = 0;
                }

                // Calculate value of theta 3

                g_x_in = (-x / Math.Cos(theta.x / MATH_TRANS) - MATH_L2) / MATH_L3;

                if (g_z_in == 0)
                {
                    g_phi = 90;
                }

                else
                {
                    g_phi = Math.Atan(-g_x_in / g_z_in) * MATH_TRANS;
                }

                g_sqrt_z_x = Math.Sqrt(g_z_in * g_z_in + g_x_in * g_x_in);

                g_right_all_2 = -1 * (g_z_in * g_z_in + g_x_in * g_x_in + g_l43 * g_l43 - 1) / (2 * g_l43);
                theta.z = Math.Asin(g_right_all_2 / g_sqrt_z_x) * MATH_TRANS;
                theta.z = theta.z - g_phi;

                if (Math.Abs(theta.z) > 90)
                {
                    theta.z = 180 - Math.Abs(theta.z);
                }

                // Calculate value of theta 2

                theta.y = Math.Asin(g_z_in + g_l43 * Math.Sin(Math.Abs(theta.z / MATH_TRANS))) * MATH_TRANS;

            }

            theta.x = Math.Abs(theta.x);
            theta.y = Math.Abs(theta.y);

            if (theta.z < 0)
            {
            }
            else
            {
                if ((calYonly(theta.x, theta.y, theta.z) > y + 0.1) || (calYonly(theta.x, theta.y, theta.z) < y - 0.1))
                {
                    theta.y = 180 - theta.y;
                }
            }

            theta.z = Math.Abs(theta.z);
            return theta;
        }

        private static double calYonly(double theta_1, double theta_2, double theta_3)

        {

            g_l3_1_2 = MATH_L3 * Math.Cos(theta_2 / MATH_TRANS);
            g_l4_1_2 = MATH_L4 * Math.Cos(theta_3 / MATH_TRANS);
            g_l5_2 = (MATH_L2 + MATH_L3 * Math.Cos(theta_2 / MATH_TRANS) + MATH_L4 * Math.Cos(theta_3 / MATH_TRANS));

            return -Math.Sin(Math.Abs(theta_1 / MATH_TRANS)) * g_l5_2;

        }

    }

    public class UArm
    {
        private double servoRightLinearIntercept;
        private double servoRightLinearSlope;
        private double servoLeftLinearIntercept;
        private double servoLeftLinearSlope;
        private double servoRotLinearIntercept;
        private double servoRotLinearSlope;
        private double servoHandRotLinearIntercept;
        private double servoHandRotLinearSlope;

        private double servoRightOffset;
        private double servoLeftOffset;
        private double servoRotOffset;
        private double servoHandRotOffset;

        public const int OFFSET_START_ADDRESS = 90;
        public const int LINEAR_START_ADDRESS = 60;

        public const int SERVO_ROT_NUM = 1;
        public const int SERVO_LEFT_NUM = 2;
        public const int SERVO_RIGHT_NUM = 3;
        public const int SERVO_HAND_ROT_NUM = 4;

        public const int SERVO_ROT_PIN = 11;
        public const int SERVO_LEFT_PIN = 13;
        public const int SERVO_RIGHT_PIN = 12;
        public const int SERVO_HAND_ROT_PIN = 10;

        public const int SERVO_ROT_ANALOG_PIN = 2;
        public const int SERVO_LEFT_ANALOG_PIN = 0;
        public const int SERVO_RIGHT_ANALOG_PIN = 1;
        public const int SERVO_HAND_ROT_ANALOG_PIN = 3;

        public const int PUMP_PIN = 6;
        public const int VALVE_PIN = 5;
        public const int RELATIVE = 0;
        public const int ABSOULTE = 0;

        public const byte CALIBRATION_FLAG = 0xEE;

        private bool isDebug = false;
        private Arduino arduino;
        private double theta_x;
        private double theta_y;
        private double theta_z;

        public static string[] GetPortNames ()
        {
            return SerialPort.GetPortNames();
        }

        public void setDebug(bool isDebug)
        {
            this.isDebug = isDebug;
        }

        public bool getDebug(bool isDebug)
        {
            return isDebug;
        }

        public double GetTheta_x()
        {
            return this.theta_x;
        }

        public double GetTheta_y()
        {
            return this.theta_y;
        }

        public double GetTheta_z()
        {
            return this.theta_z;
        }

        public UArm(String port_name, bool isDebug, int delay)
        {
            arduino = new Arduino(port_name, 57600, delay);
            init();
            this.isDebug = isDebug;
            if (isDebug)
            {
                Console.WriteLine("ServoRot Offset: " + servoRotOffset);
                Console.WriteLine("ServoLeft Offset: " + servoLeftOffset);
                Console.WriteLine("ServoRight Offset: " + servoRightOffset);
                Console.WriteLine("ServoHandRot Offset: " + servoHandRotOffset);
            }
        }

        public UArm(String port_name) : this(port_name, false, 4000)
        {
        }



        private void init()
        {
            servoRotOffset = readServoOffset(SERVO_ROT_NUM);
            servoLeftOffset = readServoOffset(SERVO_LEFT_NUM);
            servoRightOffset = readServoOffset(SERVO_RIGHT_NUM);
            readLinearOffset(SERVO_ROT_NUM);
            readLinearOffset(SERVO_LEFT_NUM);
            readLinearOffset(SERVO_RIGHT_NUM);
            servoHandRotOffset = 0;
        }

        public double readServoOffset(int servo_num)
        {
            double servo_offset = (arduino.EepromRead(OFFSET_START_ADDRESS + (servo_num - 1) * 2 + 1)) / 10.00;
            // Console.WriteLine("servo_offset: " + servo_offset);
            if (arduino.EepromRead(OFFSET_START_ADDRESS + (servo_num - 1) * 2) == 0)
            {
                servo_offset = -servo_offset;
            }
            return servo_offset;
        }

        public void attachAll()
        {
            arduino.pinMode(SERVO_HAND_ROT_PIN, Arduino.SERVO);
            arduino.pinMode(SERVO_ROT_PIN, Arduino.SERVO);
            arduino.pinMode(SERVO_RIGHT_PIN, Arduino.SERVO);
            arduino.pinMode(SERVO_LEFT_PIN, Arduino.SERVO);
        }

        public void detachAll()
        {
            arduino.pinMode(SERVO_HAND_ROT_PIN, Arduino.INPUT);
            arduino.pinMode(SERVO_ROT_PIN, Arduino.INPUT);
            arduino.pinMode(SERVO_RIGHT_PIN, Arduino.INPUT);
            arduino.pinMode(SERVO_LEFT_PIN, Arduino.INPUT);
        }

        public void MoveTo(double x, double y, double z)
        {
            Theta theta = ActionControl.calculateServoAngles(x, y, z);

            attachAll();
            AdjustAngle(theta);
            theta_x = theta.x;
            theta_y = theta.y;
            theta_z = theta.z;
            arduino.servoWrite(SERVO_ROT_PIN, (int)theta_x);
            arduino.servoWrite(SERVO_LEFT_PIN, (int)theta_y);
            arduino.servoWrite(SERVO_RIGHT_PIN, (int)theta_z);
        }

        public void PumpON()
        {
            arduino.pinMode(PUMP_PIN, Arduino.OUTPUT);
            arduino.pinMode(VALVE_PIN, Arduino.OUTPUT);
            arduino.digitalWrite(VALVE_PIN, Arduino.LOW);
            arduino.digitalWrite(PUMP_PIN, Arduino.HIGH);
        }

        public void PumpOFF()
        {
            arduino.pinMode(PUMP_PIN, Arduino.OUTPUT);
            arduino.pinMode(VALVE_PIN, Arduino.OUTPUT);
            arduino.digitalWrite(VALVE_PIN, Arduino.HIGH);
            arduino.digitalWrite(PUMP_PIN, Arduino.LOW);
        }

        private void AdjustAngle(Theta theta)
        {
            theta.x = Math.Round(theta.x + servoRotOffset);
            theta.y = Math.Round(theta.y + servoLeftOffset);
            theta.z = Math.Round(theta.z + servoRightOffset);
        }

        public void readLinearOffset(int servo_num)
        {
            int address = LINEAR_START_ADDRESS + (servo_num - 1) * 6;

            double data_a = (arduino.EepromRead(address + 1) + arduino.EepromRead(address + 2) * 256) / 10.0;
            if (arduino.EepromRead(address) == 0)
            {
                data_a = -data_a;
            }
            double data_b = (arduino.EepromRead(address + 4) + arduino.EepromRead(address + 5) * 256) / 100.0;
            if (arduino.EepromRead(address + 3) == 0)
            {
                data_b = -data_b;
            }
            switch (servo_num)
            {
                case UArm.SERVO_ROT_NUM:
                    servoRotLinearIntercept = data_a;
                    servoRotLinearSlope = data_b;
                    break;
                case UArm.SERVO_LEFT_NUM:
                    servoLeftLinearIntercept = data_a;
                    servoLeftLinearSlope = data_b;
                    break;
                case UArm.SERVO_RIGHT_NUM:
                    servoRightLinearIntercept = data_a;
                    servoRightLinearSlope = data_b;
                    break;
                case UArm.SERVO_HAND_ROT_NUM:
                    servoHandRotLinearIntercept = data_a;
                    servoHandRotLinearSlope = data_b;
                    break;

            }
        }

        public int ReadAngle(int servo_num)
        {
            int angle = 0;
            switch (servo_num)
            {
                case UArm.SERVO_ROT_NUM:
                    angle = (int)(servoRotLinearIntercept + servoRotLinearSlope * arduino.analogRead(UArm.SERVO_ROT_ANALOG_PIN));
                    break;
                case UArm.SERVO_LEFT_NUM:
                    angle = (int)(servoLeftLinearIntercept + servoLeftLinearSlope * arduino.analogRead(UArm.SERVO_LEFT_ANALOG_PIN));
                    break;
                case UArm.SERVO_RIGHT_NUM:
                    angle = (int)(servoRightLinearIntercept + servoRightLinearSlope * arduino.analogRead(UArm.SERVO_RIGHT_ANALOG_PIN));
                    break;
                case UArm.SERVO_HAND_ROT_NUM:
                    angle = (int)(servoHandRotLinearIntercept + servoHandRotLinearSlope * arduino.analogRead(UArm.SERVO_HAND_ROT_ANALOG_PIN));
                    break;

            }
            return angle;
        }

        public int ReadAngleWithOffset(int servo_num)
        {
            int angle = ReadAngle(servo_num);
            switch (servo_num)
            {
                case UArm.SERVO_ROT_NUM:
                    angle = (int)(angle + servoRotOffset);
                    break;
                case UArm.SERVO_LEFT_NUM:
                    angle = (int)(angle + servoLeftOffset);
                    break;
                case UArm.SERVO_RIGHT_NUM:
                    angle = (int)(angle + servoRightOffset);
                    break;
                case UArm.SERVO_HAND_ROT_NUM:
                    angle = (int)(angle + servoHandRotOffset);
                    break;
            }
            return angle;
        }

        public void WriteServoOffset(int address, double data)
        {
            int dataWhole;

            byte Byte0;
            byte Byte1;
            byte Byte2;

            if (Math.Abs(data) < 1)
            {
                dataWhole = (int)(data * 100);
            }
            else
            {
                dataWhole = (int)(data * 10);
            }

            if (dataWhole > 0)
            {
                Byte0 = 1;
            }
            else
            {
                Byte0 = 0;
            }

            dataWhole = Math.Abs(dataWhole);

            Byte1 = (byte)((dataWhole >> 0) & 0xFF);
            Byte2 = (byte)((dataWhole >> 8) & 0xFF);

            arduino.EepromWrite(address, Byte0);
            arduino.EepromWrite(address + 1, Byte1);
            arduino.EepromWrite(address + 2, Byte2);
        }
    }
}
