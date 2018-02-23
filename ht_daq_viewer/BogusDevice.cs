using System;
using System.Collections.Generic;
using System.Collections.Concurrent;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace DemoViewer
{
    enum DeviceState
    {
        Stopped,
        BurstAcq,
        ContinuousAcq
    }
    class BogusDevice
    {
        //private List<string> ans;
        private ConcurrentStack<string> ans;
        private DeviceState state;
        private bool isRunning;
        private int expPeriod = 1;
        private int sampleSize = 128;
        private Random rnd = new Random();
        private TimeSpan date;
        private string seconds = "0";

        public BogusDevice()
        {
            state = DeviceState.Stopped;

            ans = new ConcurrentStack<string>();
            dataSpitter();

        }

        private void dataSpitter()
        {
            isRunning = true;
            Task.Run(() =>
                {
                    while (isRunning)
                    {
                        if (state == DeviceState.BurstAcq)
                        {
                            for (int ch = 0; ch < 2; ch++)
                            {
                                string response = String.Format("{0},{1},266,{2}", ch, seconds, sampleSize);
                                //string response = String.Format("{0,2X},{1},266,{2}", ch, seconds, sampleSize);
                                for (int s = 0; s < sampleSize; s++)
                                    response += String.Format(",{0}", rnd.Next(ch * 100 + 100, ch * 100 + 150));
                                ans.Push(response + "\r\n");
                            }
                        }
                        System.Threading.Thread.Sleep(expPeriod * 1000);
                    }
                });
        }
        public string readLine()
        {
            string ret;
            while (!ans.TryPop(out ret)) {
                  System.Threading.Thread.Sleep(1);
            }
            return ret;       
        }

        public void write (string sData)
        {
            if (sData.StartsWith("$setexp"))
            {
                int value = int.Parse(sData.Substring(7).Trim());
                if (value >= 1 && value <= 10)
                {
                    expPeriod = value;
                    ans.Push("$OK\r\n");
                }
                else
                    ans.Push("$ERROR\r\n");
                Console.WriteLine(ans.Count);
            }
            else if (sData.StartsWith("$setacq"))
            {
                Console.WriteLine(sData.Substring(7));
                var args = sData.Substring(7).Trim().Split(null);
                Console.WriteLine(args);
                var period = int.Parse(args[0]);

                var size = int.Parse(args[1]);
                var mask = int.Parse(args[2]);

                sampleSize = size;
                ans.Push("OK\r\n");
            }
            else if (sData.StartsWith("$start"))
            {
                state = DeviceState.BurstAcq;
                ans.Clear();
            }
            else if (sData.StartsWith("$stop"))
            {
                state = DeviceState.Stopped;
                ans.Clear();
                ans.Push("$STOP");
            }
            else if (sData.StartsWith("$continuous"))
            {
                state = DeviceState.ContinuousAcq;
            }
            else if (sData.StartsWith("$settime"))
            {
                var args = sData.Substring(8).Trim().Split(null);
                
                if (args.GetLength(0) != 0 && !String.IsNullOrEmpty(args[0]))
                {
                    TimeSpan epochTicks = new TimeSpan(new DateTime(1970, 1, 1).Ticks);
                    TimeSpan newTicks = new TimeSpan(0, 0, int.Parse(args[0]));

                    seconds = args[0];
                    ans.Push("OK\r\n");
                }
                else
                {
                    TimeSpan epochTicks = new TimeSpan(new DateTime(1970, 1, 1).Ticks);
                    TimeSpan unixTicks = new TimeSpan(DateTime.UtcNow.Ticks) - epochTicks;
                    long unixTime = (long)unixTicks.TotalSeconds;

                    ans.Push(String.Format("{0}\r\n", seconds));
                }
            }
        }
        public void write (char[] data, int offset, int count)
        {
            string sData = data.ToString();
            this.write(sData);
        }
    }
}
