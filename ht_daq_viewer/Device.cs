using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;
using System.Text;
using System.IO.Ports;
using System.Threading.Tasks;
using System.Diagnostics;
using MathNet.Numerics.IntegralTransforms;

namespace HtDaqViewer
{
    class Device
    {
        private String mName;
        private SerialPort mPort;
        private bool mConnected;
        private string mResponse;
        private int mExperimentPeriod;
        private int mAdc01Period;
        private int mAdc2Period;
        private int mAdc01SampleLength;
        private int mAdc2SampleLength;
        private string mWelcomeMessage;
        private ushort mChannelMask;
        private int mExpectedPlots;
        private int mHasPlots;
        private ConcurrentQueue<string> mMessages;

        public Device(String portName)
        {
            mConnected = false;
            mName = portName;
            mPort = new SerialPort();

            mResponse = "";
            mMessages = new ConcurrentQueue<string>();
            mWelcomeMessage = "";

            mExperimentPeriod = 1;
            mAdc01Period = 100;
            mAdc2Period = 100;
            mAdc01SampleLength = 128;
            mAdc2SampleLength = 128;
        }

        public void Connect(string name)
        {
            mName = name;

            try
            {
                Debug.Write("Trying {0}... ", mName);
                mPort.BaudRate = 2000000;
                mPort.ReadTimeout = 1000;
                mPort.PortName = mName;
                mPort.Handshake = Handshake.RequestToSend;
                mPort.RtsEnable = true;
                mPort.Open();

                Debug.WriteLine("OK");
                mConnected = true;
            }
            catch (Exception e)
            {
                Debug.WriteLine("Error: Invalid port, must try again (" + e.Message + ")");
                mConnected = false;
                throw;
            }
        }
        
        public bool IsConnected()
        {
            return mConnected;
        }

        public void Disconnect()
        {
            try
            {
                mPort.WriteLine("$stop");
                mPort.Close();
            }
            catch (Exception e)
            {
                Debug.WriteLine("Close port error: " + e.Message);
            }
            mConnected = false;
        }

        public void SetExperimentPeriod(int period)
        {
            mExperimentPeriod = period;
        }

        public void SetSampleLength(int adc01Length, int adc2Length)
        {
            Debug.WriteLine("Setting sample length to ADC0_1: {0} and ADC2: {1}", adc01Length, adc2Length);
            mAdc01SampleLength = adc01Length;
            mAdc2SampleLength = adc2Length;
        }

        public void SetSamplePeriod(int adc01Period, int adc2Period)
        {
            mAdc01Period = adc01Period;
            mAdc2Period = adc2Period;
        }

        public void SetChannelMask(ushort mask)
        {
            mChannelMask = mask;
        }

        public string WelcomeMessage()
        {
            return mWelcomeMessage;
        }

        public void Initialize()
        {
            bool _continue = true;

            mWelcomeMessage = "";
            mPort.NewLine = "\r";

            // make sure the board is not in acquisition mode
            mPort.WriteLine("$stop");
            mPort.DiscardInBuffer();

            mPort.WriteLine("$version");

            while (_continue)
            {
                string message = "";
                try
                {
                    message = mPort.ReadLine();
                }
                catch (TimeoutException)
                {
                    Debug.WriteLine("Error: Port data read timeout");
                    break;
                }

                if (String.IsNullOrEmpty(message))
                    continue;

                Debug.WriteLine(message.Trim());

                // gather the welcome message and add newline characters
                if (message.Trim().StartsWith("-- $MSG:"))
                {
                    mWelcomeMessage += message.Substring(9).Trim(' ') + Environment.NewLine;
                }
            }
        }

        private void OnDataReady(object sender, SerialDataReceivedEventArgs e)
        {
            SerialPort sp = (SerialPort)sender;

            var bytes = sp.ReadExisting();
            mResponse += bytes;

            while (mResponse.Contains("\r\n"))
            {
                string mLine = mResponse.Substring(0, mResponse.IndexOf('\r'));

                // pass new line via the queue to the GUI thread
                mMessages.Enqueue(mLine);

                if (mLine.StartsWith("$ERROR"))
                    Console.WriteLine("board responded with error: {0}", mLine);

                mResponse = mResponse.Substring(mResponse.IndexOf('\r') + 2);
            }
        }

        private void OnSingleDataReady(object sender, SerialDataReceivedEventArgs e)
        {
            SerialPort sp = (SerialPort)sender;

            var bytes = sp.ReadExisting();
            mResponse += bytes;

            while (mResponse.Contains("\r\n"))
            {
                string mLine = mResponse.Substring(0, mResponse.IndexOf('\r'));
                Console.WriteLine("new line of length: {0}", mLine.Length);
                mMessages.Enqueue(mLine);
                mResponse = mResponse.Substring(mResponse.IndexOf('\r') + 2);
            }
        }

        private void discardPreviousData()
        {
            // start with clean input data buffer and dequeue all previous messages
            mPort.DiscardInBuffer();

            string discarded;
            while (mMessages.TryDequeue(out discarded))
            {
                // just discard them
            }
        }

        private async void delay(int ms)
        {
            await Task.Delay(ms);
        }

        public int StartAcquisition()
        {
            if (!mPort.IsOpen) return -1;

            delay(200);
            
            discardPreviousData();

            mPort.WriteLine(String.Format("$setexp {0}", mExperimentPeriod));
            mPort.WriteLine(String.Format("$setacq {0} {1} {2} {3} {4}", mAdc01Period, mAdc01SampleLength, mAdc2Period, mAdc2SampleLength, mChannelMask));
            Debug.WriteLine(String.Format("$setacq {0} {1} {2} {3} {4}", mAdc01Period, mAdc01SampleLength, mAdc2Period, mAdc2SampleLength, mChannelMask));
            mPort.WriteLine("$start");

            // connect handler to receive messages from the board
            mPort.DataReceived += new SerialDataReceivedEventHandler(OnDataReady);

            return 0;
        }

        public int SingleShotAcquisition()
        {
            if (!mPort.IsOpen) return -1;

            discardPreviousData();

            mPort.WriteLine(String.Format("$setacq {0} {1} {2} {3} {4}", mAdc01Period, mAdc01SampleLength, mAdc2Period, mAdc2SampleLength, mChannelMask));
            Console.WriteLine(String.Format("$setacq {0} {1} {2} {3} {4}", mAdc01Period, mAdc01SampleLength, mAdc2Period, mAdc2SampleLength, mChannelMask));
            //mPort.ReadLine();

            mPort.WriteLine("$single");
            //mPort.ReadLine();

            // connect handler to receive messages from the board
            mPort.DataReceived += new SerialDataReceivedEventHandler(OnSingleDataReady);
            return 0;

        }

        public int StartContinuous(int period)
        {
            if (!mPort.IsOpen) return -1;

            discardPreviousData();
            mPort.WriteLine(String.Format("$continuous {0}", period));

            // connect handler to receive messages from the board
            mPort.DataReceived += new SerialDataReceivedEventHandler(OnDataReady);
            return 0;
        }

        public int SingleShotFinished()
        {
            mPort.DataReceived -= (SerialDataReceivedEventHandler) OnSingleDataReady;
            return 0;
        }

        public void EndAcquisition()
        {
            mPort.DataReceived -= (SerialDataReceivedEventHandler) OnDataReady;
            Console.WriteLine("stopping");
            mPort.WriteLine("$stop");
        }

        public void SendReboot()
        {
            mPort.WriteLine("$reboot");
        }

        public bool DataReady()
        {
            if (mMessages.IsEmpty)
                return false;
            return true;
        }

        // returns the oldest message from the queue
        public string LastMessage()
        {
            string msg;
            mMessages.TryDequeue(out msg);
            return msg;
        }
    }
}
