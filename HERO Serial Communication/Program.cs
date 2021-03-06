﻿/**
 * Example HERO application can reads a serial port and echos the bytes back.
 * After deploying this application, the user can open a serial terminal and type while the HERO echoes the typed keys back.
 * Use a USB to UART (TTL) cable like the Adafruit Raspberry PI or FTDI-TTL cable.
 * Use device manager to figure out which serial port number to select in your PC terminal program.
 * HERO Gadgeteer Port 4 is used in this example, but can be changed at the top of Main().
 */
using System;
using System.Threading;
using Microsoft.SPOT;

namespace HERO_Serial_Example
{

    public class Program
    {
		/** Serial object, this is constructed on the serial number. */
		static System.IO.Ports.SerialPort _uart;
        /** Ring buffer holding the bytes to transmit. */
        static byte[] _tx = new byte[1024];
        static int _txIn = 0;
        static int _txOut = 0;
        static int _txCnt = 0;
        /** Cache for reading out bytes in serial driver. */
        static byte[] _rx = new byte[1024];
        /* initial message to send to the terminal */
        static byte[] _helloMsg = MakeByteArrayFromString("HERO Serial Example - Start Typing and HERO will echo the letters back.\r\n");
        /** @return the maximum number of bytes we can read*/
        private static int CalcRemainingCap()
        {
            /* first calc the remaining capacity in the ring buffer */
            int rem = _tx.Length - _txCnt;
            /* cap the return to the maximum capacity of the rx array */
            if (rem > _rx.Length)
                rem = _rx.Length;
            return rem;
        }
        /** @param received byte to push into ring buffer */
        private static void PushByte(byte datum)
        {
            _tx[_txIn] = datum;
            if (++_txIn >= _tx.Length)
                _txIn = 0;
            ++_txCnt;
        }
        /** 
         * Pop the oldest byte out of the ring buffer.
         * Caller must ensure there is at least one byte to pop out by checking _txCnt.
         * @return the oldest byte in buffer.
         */
        private static byte PopByte()
        {
            byte retval = _tx[_txOut];
            if (++_txOut >= _tx.Length)
                _txOut = 0;
            --_txCnt;
            return retval;
        }
        /** entry point of the application */
        public static void Main()
        {
            /* temporary array */
            byte[] scratch = new byte[1];
			
			bool start = false;
			int count = 0;
			int numcount = 1;
			string poop = "";
			/* open the UART, select the com port based on the desired gadgeteer port.
             *   This utilizes the CTRE.IO Library.
             *   The full listing of COM ports on HERO can be viewed in CTRE.IO
             *   
             */
			_uart = new System.IO.Ports.SerialPort(CTRE.HERO.IO.Port4.UART, 115200);
            _uart.Open();
            /* send a message to the terminal for the user to see */
            _uart.Write(_helloMsg, 0, _helloMsg.Length);
			/* loop forever */
			byte[] test = MakeByteArrayFromString("67.3");
			//float myFloat3 = System.BitConverter.ToSingle(test, 0);
			double num = double.Parse("67.555");
			Debug.Print(num.ToString());


			//Debug.Print(test[0]);
			//
			/////////

			while (true)
            {
                string test_str = "poop";
                _uart.Write(MakeByteArrayFromString(test_str), 0, test_str.Length);
                /* read bytes out of uart */
                if (_uart.BytesToRead > 0)
                {
                    int readCnt = _uart.Read(_rx, 0, CalcRemainingCap() );
					char[] array = new char[readCnt];
					for (int i = 0; i < readCnt; ++i)
                    {
						if (_rx[i] == 91)
						{
							Debug.Print("START");
							start = true;
						}
						if (start == true && _rx[i] != 91 && _rx[i] != 93)
						{
							array[i] = (char)_rx[i];
							count++;
							Debug.Print("ADD CHAR");
							poop = poop + array[i].ToString();
						}
						if (_rx[i] == 44)
						{
							numcount++;
						}
						if(_rx[i] == 93)
						{
							Debug.Print("STOP");
							start = false;
							
							Debug.Print(poop);
							string[] splitstrings = new string[numcount];
							double[] nums = new double[numcount];

							splitstrings = poop.Split(",".ToCharArray());
							
							
							for(int j = 0; j < numcount; j++)
							{
								nums[j] = double.Parse(splitstrings[j]);
								Debug.Print("THE NUM " + j.ToString() + ": "+ nums[j].ToString());
							}
							
							
							poop = "";
							numcount = 1;
						}
                        PushByte(_rx[i]);
						float myFloat = System.BitConverter.ToSingle(_rx, 0);						
					}
                }
				float myFloat2 = System.BitConverter.ToSingle(_rx, 0);

				/* if there are bufferd bytes echo them back out */
				if (_uart.CanWrite && (_txCnt > 0) )
                {
                    scratch[0] = PopByte();
                    _uart.Write(scratch, 0, 1);
                    
                }
                /* wait a bit, keep the main loop time constant, this way you can add to this example (motor control for example). */
                System.Threading.Thread.Sleep(10);
            }
        }
        /**
         * Helper routine for creating byte arrays from strings.
         * @param msg string message to covnert.
         * @return byte array version of string.
         */
        private static byte [] MakeByteArrayFromString(String msg)
        {
            byte[] retval = new byte[msg.Length];
            for(int i=0;i<msg.Length;++i)
                retval[i] = (byte)msg[i];
            return retval;
        }

	}
}
