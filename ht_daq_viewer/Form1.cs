//#define OLD_TEMP

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.IO;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms.DataVisualization.Charting;
using System.Windows.Forms;
using System.Numerics;
using System.Diagnostics;
using MathNet.Numerics.IntegralTransforms;

namespace HtDaqViewer
{
    public partial class frmMainWindow : Form
    {
        private Device board = new Device("COM1");
        private bool isInAcq;
        private bool isInSingle;
        private int mGotSingles;
        private Queue<string> mData;
        private int mCollected;
        private int mToCollect;
        private ushort[][] mChannelRawData;
        private bool mFftNeedsRefresh;
        private double[][] mChannelContinuousData;
        private int mSelectedChannel = -1;
        private int mSelectedChannels = 1;
        private int mChannelBufferLength = 4096;
        private int mContinuousIndex;
        private int mPrevIndex;
        private const int MAX_RAW_SAMPLES = 8192*2;
        private const int MAX_CONTINUOUS_SAMPLES = 512;
        private double[] fft_signal;
        private int mFftLength;
        private Timer mTimer = new Timer();
        private int mTemperature = 0;
        private int mTemperatureCounter = 0;

        public frmMainWindow()
        {
            InitializeComponent();

            mData = new Queue<string>();
            mChannelRawData = new ushort[10][];
            mChannelContinuousData = new double[10][];
            mContinuousIndex = 0;

            mFftNeedsRefresh = false;

            for (int i = 0; i < 10; i++)
            {
                mChannelRawData[i] = new ushort[MAX_RAW_SAMPLES];
                for (int j = 0; j < MAX_RAW_SAMPLES; j++)
                {
                    mChannelRawData[i][j] = 0;
                }
                mChannelContinuousData[i] = new double[MAX_CONTINUOUS_SAMPLES];
                for (int j = 0; j < MAX_CONTINUOUS_SAMPLES; j++)
                {
                    mChannelContinuousData[i][j] = double.NaN;
                }
            }

            fft_signal = new double[MAX_RAW_SAMPLES];
            mFftLength = 0;

            isInAcq = false;
            isInSingle = false;

            // initialize charts and AC analysis tab
            chartInit();
            clearAcAnalysis();

            // update sample rate information
            Single value = Single.Parse(txtDigitalSamplingPeriod.Text);
            lblRateDigital.Text = "Sample Rate: " + ((double)1000000.0 / value).ToString("0.00") + " (sps)";
            value = Single.Parse(txtAnalSamplingPeriod.Text);
            lblRateAnalog.Text = "Sample Rate: " + ((double)1000000.0 / value).ToString("0.00") + " (sps)";

            updateChannelSelection();

            // start timer for FFT and AC analysis updates
            mTimer.Interval = 500;
            mTimer.Tick += new EventHandler(OnTick);
            mTimer.Start();



            if (File.Exists("dupa.txt")) {

            isInAcq = true;
            mFftNeedsRefresh = true;
            mSelectedChannel = 0;

                StreamReader errFile;

                string filename = "dupa.txt";

                errFile = new StreamReader(filename);

                for (int i = 0; i < mChannelBufferLength; i++)
                {
                    
                    string line = errFile.ReadLine();
                    mChannelRawData[mSelectedChannel][i] = Convert.ToUInt16(line.Substring(0, 5));
                    Console.WriteLine("{0}", mChannelRawData[mSelectedChannel][i]);
                }
                errFile.Close();


                try
                {
                    // this is done upon timer tick because running FFT caused hangs
                    // or other weird issues
                    PerformAnalysis();
                }
                catch (Exception ex)
                {
                    Console.WriteLine("Analysis ERROR: " + ex.Message);
                }

            isInAcq = false;
            }

        }

        private void OnTick(Object sender, EventArgs e)
        {
            try
            {
                // this is done upon timer tick because running FFT caused hangs
                // or other weird issues
                PerformAnalysis();
            }
            catch (Exception ex)
            {
               
                Console.WriteLine("details: " + ex.ToString());
                {     // Get stack trace for the exception with source file information
                    var st = new StackTrace(ex, true);
                    // Get the top stack frame
                    var frame = st.GetFrame(0);
                    // Get the line number from the stack frame
                    var line = frame.GetFileLineNumber();

                    Console.WriteLine(String.Format("Analysis ERROR @ {0}: ", line) + ex.Message);
                }

                // store data to a file
                if (false)
                {
                    StreamWriter errFile;

                    string filename = Environment.GetFolderPath(
                             System.Environment.SpecialFolder.DesktopDirectory);
                    filename += "\\analysis_error_" + DateTime.Now.ToString("yyyyddM-HHmmss") + ".txt";

                    if (File.Exists(filename))
                        File.Delete(filename);

                    errFile = new StreamWriter(filename);

                    for (int i = 0; i < mChannelBufferLength; i++)
                    {
                        errFile.WriteLine(string.Format("{0,5},", mChannelRawData[mSelectedChannel][i]));
                    }
                    errFile.Close();
                }
            }
        }

        private void refreshContinuous(List<string> dataMessages)
        {
            foreach (string data in dataMessages)
            {
                // we only expect CONT messages here
                if (!data.StartsWith("$CONT:")) continue;

                var datum = data.Substring(6).Split(',');

                // no other number of fields allowed (should drop corrupted lines)
                if (datum.Length != 11) continue;

                // convert the gap between shots based on the index
                int index = Convert.ToByte(datum[0], 16);
                int gap = 1;
                if (mPrevIndex != -1)
                {
                    gap = index - mPrevIndex;
                    if (gap < 0)
                        gap += 255;
                }

                // if the gap is greater then 1, we lost some data
                if (gap > 1)
                {
                    for (int i = 0; i < gap; i++)
                    {
                        for (int ch = 0; ch < 10; ch++)
                        {
                            // show NaN then
                            mChannelContinuousData[ch][mContinuousIndex] = double.NaN;
                        }
                        // circular buffer indexing
                        if (++mContinuousIndex >= MAX_CONTINUOUS_SAMPLES)
                            mContinuousIndex = 0;
                    }
                }

                for (int i = 0; i < datum.Length - 1; i++)
                {
                    mChannelContinuousData[i][mContinuousIndex] =  (double) Convert.ToUInt16(datum[1 + i], 16);

                    if (i == 9 && mTemperatureCounter == 0)
                    {
                        UInt32 temp = getTempK10_from_adcCodeSum(Convert.ToUInt16(datum[1+i], 16));
                        mTemperature = (int) temp - 2732;
                        mTemperature /= 10;

                        float fTemp = ((float) temp - 2732.0f) / 10.0f;
                        tempLabel.Text = String.Format("Board Temperature: {0:0}°C", fTemp);
                    }
                    if (i == 8 && mTemperatureCounter == 0)
                    {
                        float voltage = Convert.ToUInt16(datum[i + 1], 16) * (float) 2.5 / UInt16.MaxValue;
                        float vcc = (voltage * 304 - 5) / 100;
                        vccLabel.Text = String.Format("VCC: {0:0.00}V", vcc);
                    }
                }

                // show measurements but not so fast
                if (++mTemperatureCounter >= 100)
                    mTemperatureCounter = 0;

                // circular buffer indexing
                if (++mContinuousIndex >= MAX_CONTINUOUS_SAMPLES)
                    mContinuousIndex = 0;
                // remember current index to calculate sample loss
                mPrevIndex = index;
            }

            // now update all charts
            for (int j = 0; j < 10; j++)
            {
                Series ser = new Series();
                ser.ChartType = SeriesChartType.Line;
                ser.BorderWidth = 2;
                ser.YValueType = ChartValueType.Double;
                ser.Color = System.Drawing.Color.White;
                ser.Name = "Channel " + j.ToString();
                ser.ChartArea = "ch" + j.ToString();
  
                for (int i = 0; i < MAX_CONTINUOUS_SAMPLES; i++)
                {
                    double yval = mChannelContinuousData[j][(i + mContinuousIndex) % MAX_CONTINUOUS_SAMPLES];
                    double xval = (double)i;
                    xval *= (double)nudContSampRate.Value;

                    ser.Points.AddXY(xval, yval * 2.5 / ushort.MaxValue);
                }
                chart.Series[j] = ser;
            }
            chart.Update();
        }

        public static double[] BlackmanHarris7(int width)
        {
            const double a0 =  0.27105140069342;
            const double a1 = -0.43329793923448;
            const double a2 =  0.21812299954311;
            const double a3 = -0.06592544638803;
            const double a4 =  0.01081174209837;
            const double a5 = -0.00077658482522;
            const double a6 =  0.00001388721735;

            double w  = 2.0 * Math.PI / width;
            double w2 = 2.0 * w;
            double w3 = 3.0 * w;
            double w4 = 4.0 * w;
            double w5 = 5.0 * w;
            double w6 = 6.0 * w;

            var result = new double[width];
            for (int i = 0; i < result.Length; i++)
            {
                result[i] = a0
                          + a1 * Math.Cos(w  * i)
                          + a2 * Math.Cos(w2 * i)
                          + a3 * Math.Cos(w3 * i)
                          + a4 * Math.Cos(w4 * i)
                          + a5 * Math.Cos(w5 * i)
                          + a6 * Math.Cos(w6 * i);
            }
            return result;
        }

        private void PerformAnalysis()
        {
            // sampling rate depending on the MUX selection
            double SamplingRate = 1e6 / double.Parse(txtDigitalSamplingPeriod.Text);
            if (mSelectedChannel >= 2)
                SamplingRate = 1e6 / double.Parse(txtAnalSamplingPeriod.Text);

            // do the mumbo jumbo only if refresh is needed
            if (!isInAcq && !mFftNeedsRefresh) return;
            mFftNeedsRefresh = false;


            tempLabel.Text = "Board Temperature: " + mTemperature.ToString() + "°C";
            Debug.WriteLine("analysis for selected channel: {0}", mSelectedChannel);

            // only one channel must be selected
            if (mSelectedChannel == -1)
            {
                fftChart.Titles[0].Text = "FFT is unavailable when multiple channels are selected or in continuous mode.";
                fftChart.Titles[0].ForeColor = Color.Red;
                fftChart.Titles[0].Visible = true;

                HistogramChart.Titles[0].Text = "Histogram is unavailable when multiple channels are selected or in continuous mode.";
                HistogramChart.Titles[0].ForeColor = Color.Red;
                HistogramChart.Titles[0].Visible = true;
                return;
            }

            // set chart titles and make them visible
            fftChart.Titles[0].ForeColor = Color.Black;
            fftChart.Titles[0].Text = "Channel " + mSelectedChannel.ToString();
            HistogramChart.Titles[0].ForeColor = Color.Black;
            HistogramChart.Titles[0].Text = "Channel " + mSelectedChannel.ToString();
            fftChart.Titles[0].Visible = true;
            HistogramChart.Titles[0].Visible = true;

            var Len = (int)mChannelBufferLength;    
            var histLen = Len;

            // sampling frequency depending on MUX
            var Fs = ((double) 1000000 / double.Parse(txtDigitalSamplingPeriod.Text));
            if (mSelectedChannel >= 2)
                Fs = ((double) 10e6 / double.Parse(txtAnalSamplingPeriod.Text));

            double[] uints = new double[Len];
            int[] vals = new int[Len];
            Double DcAmplitude = 0.0f;

            lock (mChannelRawData.SyncRoot)
            {
                for (int i = 0; i < Len; i++)
                {
                    uints[i] = mChannelRawData[mSelectedChannel][i];
                    vals[i] = mChannelRawData[mSelectedChannel][i];
                    DcAmplitude += uints[i];
                }
            }

            DcAmplitude /= Len;

            // we are fixed at 2.5 V
            double MaxAmplitude = uints.Max() * 2.5 / ushort.MaxValue;
            double MinAmplitude = uints.Min() * 2.5 / ushort.MaxValue;

            // get rid of the DC
            for (int i = 0; i < Len; i++)
            {
                uints[i] -= (double) DcAmplitude;
                uints[i] /= (ushort.MaxValue / 2 / Math.Sqrt(2));         
            }

            DcAmplitude *= (double) 2.5 / (double) ushort.MaxValue;

            var buffer = new System.Numerics.Complex[uints.Length];

            var win = BlackmanHarris7(uints.Length);

            for (int i = 0; i < uints.Length; i++)
                buffer[i] = uints[i] * win[i] / 0.27105140069342415;
            
            Fourier.Forward(buffer, FourierOptions.Matlab);

            mFftLength = uints.Length / 2;
 
            for (int i = 0; i < mFftLength; i++)
            {
                fft_signal[i] = buffer[i].Magnitude / uints.Length;
                fft_signal[i] *= Math.Sqrt(2);
            }

            var max = fft_signal.Max();

            int SigLen = mFftLength * 2;
            int DcBins = 6;
            int SignalBins = 6;
            int SignalNeighbourhood = 2 * SignalBins + 1;
            int NoHarmonics = 6;

            // chart updates
            Series fftSeries = new Series();
            fftSeries.Name = "Channel " + mSelectedChannel;
            fftSeries.ChartType = SeriesChartType.Line;
            fftSeries.BorderWidth = 2;
            fftSeries.YValueType = ChartValueType.Double;
            fftSeries.Color = System.Drawing.Color.White;
            
            for (int x = 0; x < mFftLength; x++)
            {
                double value = 20 * Math.Log10(fft_signal[x]);
                // make sure we don't break the chart library with incorrect values
                if (double.IsNegativeInfinity(value))
                    fftSeries.Points.AddXY((double)x * (SamplingRate / 2) / mFftLength, -200.0);
                else
                    fftSeries.Points.AddXY((double) x * (SamplingRate/2) / mFftLength, 20 * Math.Log10(fft_signal[x]));
            }

            fftChart.Series.Clear();
            fftChart.Series.Add(fftSeries);
            fftChart.Update();

#if true
            // now the histogram update
            int noBins = 100;
            int[] histData = new int[noBins];
            int[] histAxis = new int[noBins];

            for (int i = 0; i < noBins; i++)
                histData[i] = 0;

            double delta_x = (vals.Max() - vals.Min()) / noBins;

            if (delta_x <= 1)
                delta_x = 1;

            for (int i = 0; i < histLen; i++)
            {
                //int bucketIndex = (int) Math.Floor(((double)vals[i] - ushort.MinValue) / delta_x);
                int bucketIndex = (int)Math.Floor(((double)vals[i] - (double)vals.Min()) / delta_x);
                if (bucketIndex >= noBins) bucketIndex--;
                //if (bucketIndex == 0) bucketIndex++;
                    
                histData[bucketIndex]++;
            }

            HistogramChart.Series.Clear();
            Series barSeries = new Series();
            barSeries.ChartType = SeriesChartType.Column;
            barSeries.Color = System.Drawing.Color.White;
                
            for (int i = 0; i < noBins; i++)
            {
                histAxis[i] = (int) ( i * (double)ushort.MaxValue / noBins);
                barSeries.Points.AddXY(histAxis[i], histData[i]);
            }
   
            HistogramChart.Series.Add(barSeries);

            var count = 0;
            foreach (CustomLabel lbl in HistogramChart.ChartAreas[0].AxisX.CustomLabels)
            {
                lbl.Text = histAxis[count++].ToString();
            }
                
            HistogramChart.Update();
            HistogramChart.ChartAreas[0].RecalculateAxesScale();
#endif

            // AC anaysis
            for (int i = 0; i < DcBins; i++)
                fft_signal[i] = 0;

            var fund_v = fft_signal.Max();
            var fund_index = fft_signal.ToList().IndexOf(fund_v);


            var fundNeighbourhood = fft_signal.Skip(fund_index - SignalBins).Take(SignalNeighbourhood).ToArray();

            for (int i = 0; i < fundNeighbourhood.Length; i++)
                fundNeighbourhood[i] *= fundNeighbourhood[i];

            var rms_fund = Math.Sqrt(fundNeighbourhood.Sum());

            var bin_width = SamplingRate / (2 * mFftLength);
            var fund_f_khz = bin_width * fund_index / 1000;

            Debug.WriteLine("sr = {0}, fftlen = {1}, bin_width = {2}", SamplingRate, fft_signal.Length, bin_width);
            Debug.WriteLine("fundamental: {0:0,000} kHz @ {1}", fund_f_khz, fund_index);

            double avg_noise_floor = 0.0;
            for (int i = 0; i < mFftLength; i++)
            {
                if (i >= fund_index - SignalBins && i <= fund_index + SignalBins) continue;
                avg_noise_floor += fft_signal[i];
            }
            avg_noise_floor /= (mFftLength - SignalNeighbourhood);

            // critical part that may cause out of bounds exceptions
            try
            {
                double[] fft_input = new double[fft_signal.Length];
                fft_signal.CopyTo(fft_input, 0);

                for (int i = 0; i < SignalNeighbourhood; i++)
                {
                    fft_input[fund_index - SignalBins + i] = avg_noise_floor;
                }
                fft_input[fund_index] = 0;

                var fft_size = mFftLength;

                double[] fft_inputSquared = new double[fft_size];

                for (int i = 0; i < fft_size; i++)
                    fft_inputSquared[i] = fft_input[i] * fft_input[i];

                double Sinad = 20 * Math.Log10(rms_fund / Math.Sqrt(fft_inputSquared.Sum()));

                int[] HarmonicLocation = new int[NoHarmonics];
                double[] HarmonicAmplitude = new double[NoHarmonics];
                double[] HarmonicAmplitudeSquared = new double[NoHarmonics];

                for (int i = 0; i < NoHarmonics; i++)
                {
                    int NextFreqNotWound = (i + 2) * fund_index;
                    int tmp1 = fft_size * 2 - 1;
                    int tmp2 = NextFreqNotWound - tmp1 * (int)Math.Floor((double)NextFreqNotWound / tmp1);

                    int next_index = 0;
                    if (tmp2 > fft_size - 1)
                        next_index = tmp1 - tmp2;
                    else
                        next_index = tmp2;

                    var fn = fft_input.Skip(next_index - 3).Take(6).ToArray();
                    var fh_max = fn.Max();
                    var fh_max_index = fn.ToList().IndexOf(fh_max);

                    next_index += fh_max_index - 3;

                    HarmonicLocation[i] = next_index;

                    if (fft_input[next_index] == 0)
                    {
                        HarmonicAmplitudeSquared[i] = avg_noise_floor * avg_noise_floor;
                        HarmonicAmplitude[i] = avg_noise_floor;
                    }
                    else
                    {
                        HarmonicAmplitudeSquared[i] = fft_input[next_index] * fft_input[next_index];
                        HarmonicAmplitude[i] = fft_input[next_index];
                    }
                }

                double[] HarmonicAmplitudeSquared2 = new double[NoHarmonics];
                double[] HarmonicAmplitude2 = new double[NoHarmonics];
                double[] fft_input2 = new double[fft_input.Length];
                fft_input.CopyTo(fft_input2, 0);

                for (int i = 0; i < NoHarmonics; i++)
                {
                    var harm_index = HarmonicLocation[i];

                    for (int j = 0; j < 9; j++)
                    {
                        fft_input2[harm_index - 4 + j] = avg_noise_floor;
                    }

                    var amplitude = 0.0;

                    for (int j = 0; j < 6; j++)
                        amplitude += fft_input[harm_index - 3 + j] * fft_input[harm_index - 3 + j];

                    HarmonicAmplitudeSquared2[i] = Math.Sqrt(amplitude) * Math.Sqrt(amplitude);
                    HarmonicAmplitude2[i] = Math.Sqrt(amplitude);
                }

                double[] fft_input2Squared = new double[fft_input2.Length];

                for (int i = 0; i < fft_size; i++)
                    fft_input2Squared[i] = fft_input2[i] * fft_input2[i];

                double Thd = 20 * Math.Log10(Math.Sqrt(HarmonicAmplitudeSquared2.Sum()) / rms_fund);
                double Snr = 20 * Math.Log10(rms_fund / Math.Sqrt(fft_input2Squared.Sum()));
                double DynR = 4.48 - 20 * Math.Log10(Math.Sqrt(fft_input2Squared.Sum()));

                labelDynamicRange.Text = String.Format("{0:0.000}  dB", DynR);
                labelSnr.Text = String.Format("{0:0.000}  dB", Snr);
                labelThd.Text = String.Format("{0:0.000}  dB", Thd);
                labelSinad.Text = String.Format("{0:0.000}  dB", Sinad);

                labelFreq2nd.Text = String.Format("{0:0.000}  kHz", bin_width * HarmonicLocation[0] / 1000);
                labelFreq3rd.Text = String.Format("{0:0.000}  kHz", bin_width * HarmonicLocation[1] / 1000);
                labelFreq4th.Text = String.Format("{0:0.000}  kHz", bin_width * HarmonicLocation[2] / 1000);
                labelFreq5th.Text = String.Format("{0:0.000}  kHz", bin_width * HarmonicLocation[3] / 1000);

                labelAmp2nd.Text = String.Format("{0:0.000}  dBFS", 20 * Math.Log10(HarmonicAmplitude[0]));
                labelAmp3rd.Text = String.Format("{0:0.000}  dBFS", 20 * Math.Log10(HarmonicAmplitude[1]));
                labelAmp4th.Text = String.Format("{0:0.000}  dBFS", 20 * Math.Log10(HarmonicAmplitude[2]));
                labelAmp5th.Text = String.Format("{0:0.000}  dBFS", 20 * Math.Log10(HarmonicAmplitude[3]));
            }
            catch (Exception e)
            {
                Console.WriteLine("analysis exception:" + e.Message);

                labelDynamicRange.Text = "-  dB";
                labelSnr.Text = "-  dB";
                labelThd.Text = "-  dB";
                labelSinad.Text = "-  dB";

                labelFreq2nd.Text = "-  kHz";
                labelFreq3rd.Text = "-  kHz";
                labelFreq4th.Text = "-  kHz";
                labelFreq5th.Text = "-  kHz";

                labelAmp2nd.Text = "-  dBFS";
                labelAmp3rd.Text = "-  dBFS";
                labelAmp4th.Text = "-  dBFS";
                labelAmp5th.Text = "-  dBFS";
            }

            labelMaxAmplitude.Text = String.Format("{0:0.000} V", MaxAmplitude);
            labelMinAmplitude.Text = String.Format("{0:0.000} V", MinAmplitude);
            labelPkPkAmplitude.Text = String.Format("{0:0.000} V", MaxAmplitude - MinAmplitude);
            labelDcAmplitude.Text = String.Format("{0:0.000000} V", DcAmplitude);
            labelFundamentalAmplitude.Text = String.Format("{0:0.000} dBFS", 20 * Math.Log10(fund_v));
            labelFundamentalFrequency.Text = String.Format("{0:0.000} Hz", bin_width * fund_index);
            labelRms.Text = String.Format("{0:0.000} V", (MaxAmplitude - MinAmplitude) /Math.Sqrt(2));

            labelNoiseFloor.Text = String.Format("{0:0.000}  dB", 20 * Math.Log10(avg_noise_floor));
            labelFreqFund.Text = String.Format("{0:0.000}  kHz", fund_f_khz);
            labelAmpFund.Text = String.Format("{0:0.000}  dBFS", 20 * Math.Log10(fund_v));
        }

        private void refresh()
        {
            if (cbContinuous.Checked)
            {
                List<string> messages = new List<String>();
                while (mData.Count > 0)
                {
                    messages.Add(mData.Dequeue());
                }
                refreshContinuous(messages);
            }
            else
            {
                while (mData.Count > 0)
                {
                    string data = mData.Dequeue();

                    if (data.StartsWith("$ERROR:"))
                    {
                        Debug.WriteLine("acquisition error: {0}", data);
                        if (isInSingle)
                            singleShotEnd();
                        else
                            stopAcquisition();
                        btnAcquisition.Text = "Start Acquisition";
                        groupBoxAcq.Enabled = true;

                        updateButtons();
                        MessageBox.Show(data.Substring(7), "Acquisition Error");
                        return;
                    }

#if !OLD_TEMP
                    // analyse TEMP line
                    if (data.StartsWith("$TEMP"))
                    {
                        var fields = data.Substring(6).Split(',');
                        mTemperature = Convert.ToInt32(fields[0].Split('.')[0]);
                        //Console.WriteLine("T: {0}", data);
                        //Console.WriteLine("new temperature {0} - {1}", fields[0], mTemperature);
                        var vcc = Convert.ToDouble(fields[1].Split('=')[1].TrimEnd('V'));
                        vccLabel.Text = String.Format("VCC: {0:0.00}V", vcc);
                    }
#endif
                    // not interested with lines not starting with BURST
                    if (!data.StartsWith("$BURST:")) continue;

                    var datum = data.Substring(7).Split(',');
                    var ch = int.Parse(datum[0]);

                    Console.WriteLine("new data in channel {0}, len {1}", ch, datum.Length);                    
#if OLD_TEMP
                    int auxFields = 5;
#else
                    int auxFields = 3;
#endif
                    UInt16[] uints = new UInt16[datum.Length - auxFields];

#if OLD_TEMP
                    Console.WriteLine("temp: {0}", datum[2]);
                    try
                    {
                        mTemperature = Convert.ToUInt16(datum[2],16);
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("BAD LUCK: " + ex.Message);
                    }
                    
                    mTemperature -= 2732;
                    mTemperature /= 10;

                    Debug.WriteLine("new temperature {0}", mTemperature);
#endif

                    for (int i = 0; i < datum.Length - auxFields; i++)
                        uints[i] = Convert.ToUInt16(datum[auxFields + i], 16);

                    Series ser = new Series();
                    ser.Name = "Channel " + datum[0];
                    ser.ChartType = SeriesChartType.Line;
                    ser.BorderWidth = 2;
                    ser.YValueType = ChartValueType.Double;

                    ser.ChartArea = "ch" + datum[0];
                    ser.BorderWidth = 2;
                    ser.Color = System.Drawing.Color.White;

                    mCollected++;

                    lock (mChannelRawData.SyncRoot)
                    {
                        for (int x = 0; x < uints.Length; x++)
                        {
                            Double xt = x;
                            xt *= ((ch < 2) ? double.Parse(txtDigitalSamplingPeriod.Text) : double.Parse(txtAnalSamplingPeriod.Text));
                            xt /= 1000.0f;
                            //Console.WriteLine("{0} {1}", xt, x);
                            ser.Points.AddXY(xt, uints[x] * 2.5 / ushort.MaxValue);
                            mChannelRawData[ch][x] = uints[x];
                        }
                        for (int x = uints.Length; x < MAX_RAW_SAMPLES; x++)
                        {
                            mChannelRawData[ch][x] = 0;
                        }
                    }

                    //chart.Series[ch].Points.Invalidate();
                    chart.Series[ch] = ser;
                    //chart.Series[ch].Points.Invalidate();

                }
                chart.Update();

                if (isInSingle && mCollected >= mToCollect)
                {
                    singleShotEnd();
                    mFftNeedsRefresh = true;
                }
            }
        }

        private void chartInit()
        {
            chart.Series.Clear();
            chart.Titles.Clear();

            foreach (var chArea in chart.ChartAreas)
            {
                chArea.AxisX.Title = "time [ms]";
                chArea.AxisX.IntervalAutoMode = IntervalAutoMode.VariableCount;
                // chArea.AxisX.LabelStyle.Format = "{#.0}";
                chArea.AxisX.LabelStyle.Format = "{0.0}";
                chArea.AxisY.Title = "voltage [V]";

                chArea.AxisY.Minimum = 0;
                chArea.AxisY.Maximum = 2.5;
                chArea.AxisY.Interval = 0.5;
                chArea.AxisY.IntervalAutoMode = IntervalAutoMode.FixedCount;

                chArea.BackColor = Color.Black;
                chArea.AxisX.MajorGrid.LineColor = Color.ForestGreen;
                chArea.AxisY.MajorGrid.LineColor = Color.ForestGreen;

                chArea.AxisX.MinorGrid.LineColor = Color.DarkGreen;
                chArea.AxisY.MinorGrid.LineColor = Color.DarkGreen;

                chArea.AxisX.MajorGrid.LineDashStyle = ChartDashStyle.Dash;
                chArea.AxisY.MajorGrid.LineDashStyle = ChartDashStyle.Dash;

                chArea.Visible = false;
            }

            for (int i = 0; i < chart.ChartAreas.Count; i++)
            {
                chart.Titles.Add("Channel " + i);
                
                chart.Titles[i].DockedToChartArea = "ch" + i;
                chart.Titles[i].ForeColor = Color.White;
            }
          
            for (int ch = 0; ch < 10; ch++)
            {
                Series ser = new Series();

                ser.Name = "Channel " + ch.ToString();
                ser.ChartType = SeriesChartType.Line;
                ser.ChartArea = "ch" + ch;
                   
                ser.BorderWidth = 2;
                ser.YValueType = ChartValueType.UInt32;

                int sampleCount = mChannelBufferLength;
                for (int x = 0; x < sampleCount - 1; x++)
                {
                    ser.Points.AddXY(x, 0);
                }

                chart.Series.Add(ser);
            }

            double SamplingRate = 1e6 / double.Parse(txtDigitalSamplingPeriod.Text);
            if (mSelectedChannel >= 2)
                SamplingRate = 1e6 / double.Parse(txtAnalSamplingPeriod.Text);

            fftChart.ChartAreas[0].AxisY.Minimum = -140;
            fftChart.ChartAreas[0].AxisY.Maximum = 0;
            fftChart.ChartAreas[0].AxisY.Interval = 20;
            fftChart.ChartAreas[0].AxisY.Title = "Amplitude [dBFS]";
            fftChart.ChartAreas[0].AxisY.IntervalAutoMode = IntervalAutoMode.FixedCount;
            fftChart.ChartAreas[0].AxisY.MajorGrid.LineDashStyle = ChartDashStyle.Dash;
            fftChart.ChartAreas[0].AxisY.MajorGrid.LineColor = Color.ForestGreen;
            fftChart.ChartAreas[0].AxisY.MinorGrid.LineColor = Color.DarkGreen;
            fftChart.ChartAreas[0].AxisY.MinorGrid.Interval = 5;
                
            fftChart.ChartAreas[0].AxisX.Minimum = 0;
            fftChart.ChartAreas[0].AxisX.Maximum = SamplingRate / 2;
            fftChart.ChartAreas[0].AxisX.Interval = SamplingRate / 10;
            fftChart.ChartAreas[0].AxisX.Title = "Frequency [Hz]";
            fftChart.ChartAreas[0].AxisX.IntervalAutoMode = IntervalAutoMode.FixedCount;
            fftChart.ChartAreas[0].AxisX.MajorGrid.LineDashStyle = ChartDashStyle.Dash;
            fftChart.ChartAreas[0].AxisX.MajorGrid.LineColor = Color.ForestGreen;
            fftChart.ChartAreas[0].AxisX.MinorGrid.LineColor = Color.DarkGreen;

            fftChart.Update();

            HistogramChart.ChartAreas[0].AxisX.Minimum = -10000;
            HistogramChart.ChartAreas[0].AxisX.Maximum =  70000;
            HistogramChart.ChartAreas[0].AxisX.Interval = 10000;

            HistogramChart.Update();
        }

        private void singleShotAcquisition()
        {
            ushort mask = 0x0000;
            mask |= cbDigitalCh1.Checked ? (ushort)0x0001 : (ushort)0x0;
            mask |= cbDigitalCh2.Checked ? (ushort)0x0002 : (ushort)0x0;
            mask |= cbAnalCh1.Checked ? (ushort)0x0004 : (ushort)0x0;
            mask |= cbAnalCh2.Checked ? (ushort)0x0008 : (ushort)0x0;
            mask |= cbAnalCh3.Checked ? (ushort)0x0010 : (ushort)0x0;
            mask |= cbAnalCh4.Checked ? (ushort)0x0020 : (ushort)0x0;
            mask |= cbAnalCh5.Checked ? (ushort)0x0040 : (ushort)0x0;
            mask |= cbAnalCh6.Checked ? (ushort)0x0080 : (ushort)0x0;
            mask |= cbAnalCh7.Checked ? (ushort)0x0100 : (ushort)0x0;
            mask |= cbAnalCh8.Checked ? (ushort)0x0200 : (ushort)0x0;

            chartInit();

            int selectedChannels = 0;


            ushort maskCopy = mask;
            for (int i = 0; i < 10; i++)
            {
                if ((maskCopy & 0x1) == 0x1)
                {
                    mSelectedChannel = i;
                    selectedChannels++;
                }
                maskCopy >>= 1;
            }
            if (selectedChannels != 1)
                mSelectedChannel = -1;

            mChannelBufferLength = (int)Math.Floor(8192.0 / selectedChannels);

            // set acquisition parameters
            board.SetChannelMask(mask);
            board.SetExperimentPeriod((int)nudExperimentPeriod.Value);
            board.SetSampleLength(mChannelBufferLength, mChannelBufferLength);
            board.SetSamplePeriod((int)(10*double.Parse(txtDigitalSamplingPeriod.Text)), (int)(10*double.Parse(txtAnalSamplingPeriod.Text)));

            // enable visible channels and hide the others
            chart.ChartAreas[0].Visible = cbDigitalCh1.Checked;
            chart.ChartAreas[1].Visible = cbDigitalCh2.Checked;
            chart.ChartAreas[2].Visible = cbAnalCh1.Checked;
            chart.ChartAreas[3].Visible = cbAnalCh2.Checked;
            chart.ChartAreas[4].Visible = cbAnalCh3.Checked;
            chart.ChartAreas[5].Visible = cbAnalCh4.Checked;
            chart.ChartAreas[6].Visible = cbAnalCh5.Checked;
            chart.ChartAreas[7].Visible = cbAnalCh6.Checked;
            chart.ChartAreas[8].Visible = cbAnalCh7.Checked;
            chart.ChartAreas[9].Visible = cbAnalCh8.Checked;

            mToCollect = 0;
            mToCollect += cbDigitalCh1.Checked ? 1 : 0;
            mToCollect += cbDigitalCh2.Checked ? 1 : 0;
            mToCollect += cbAnalCh1.Checked ? 1 : 0;
            mToCollect += cbAnalCh2.Checked ? 1 : 0;
            mToCollect += cbAnalCh3.Checked ? 1 : 0;
            mToCollect += cbAnalCh4.Checked ? 1 : 0;
            mToCollect += cbAnalCh5.Checked ? 1 : 0;
            mToCollect += cbAnalCh6.Checked ? 1 : 0;
            mToCollect += cbAnalCh7.Checked ? 1 : 0;
            mToCollect += cbAnalCh8.Checked ? 1 : 0;
            mCollected = 0;

            isInSingle = true;
            groupBoxAcq.Enabled = false;
            board.SingleShotAcquisition();

            Task.Run(() =>
            {
                while (isInSingle)
                {

                    if (board.DataReady())
                    {
                        mData.Enqueue(board.LastMessage());
                        this.Invoke((MethodInvoker)refresh);
                    }
                    else
                    {
                        System.Threading.Thread.Sleep(1);
                    }
                }
            });
        }

        private void singleShotEnd()
        {
            Console.WriteLine("single ends");
            try
            {
                board.SingleShotFinished();
            }
            catch (Exception ex)
            {
                MessageBox.Show("No answer from board, " + ex.Message, "Warning");

            }
            isInSingle = false;
            groupBoxAcq.Enabled = true;
        }

        private void startAcquisition(bool continuous)
        {
            isInAcq = true;

            while (board.DataReady())
                board.LastMessage();

            if (continuous)
            {
                mTemperatureCounter = 0;

                // in continuous mode make all channels visible except temp and VCC
                chart.ChartAreas[0].Visible = true;
                chart.ChartAreas[1].Visible = true;
                chart.ChartAreas[2].Visible = true;
                chart.ChartAreas[3].Visible = true;
                chart.ChartAreas[4].Visible = true;
                chart.ChartAreas[5].Visible = true;
                chart.ChartAreas[6].Visible = true;
                chart.ChartAreas[7].Visible = true;
                chart.ChartAreas[8].Visible = false;
                chart.ChartAreas[9].Visible = false;

                mPrevIndex = -1;
                board.StartContinuous((int)(nudContSampRate.Value * 1000));
            }
            else
                board.StartAcquisition();

            Console.WriteLine("starting acquisition");
            Task.Run(() =>
            {
                while (isInAcq) 
                {
                    if (board.DataReady())
                    {
                        while (board.DataReady())
                        {
                            // enqueue the line from the second thread...
                            mData.Enqueue(board.LastMessage());
                        }
                        // ...and invoke refresh to update GUI
                        this.Invoke((MethodInvoker)refresh);
                    }
                    else
                    {
                        // wait 1 ms in case there is no traffic
                        System.Threading.Thread.Sleep(1);
                    }
                }
            });
        }

        private void stopAcquisition(bool needsReboot = false)
        {
            try
            {
                board.EndAcquisition();
                if (needsReboot)
                    board.SendReboot();
            }
            catch (Exception ex)
            {
                MessageBox.Show("No answer from board, " + ex.Message, "Warning");

            }
            tempLabel.Text = "Board Temperature: -";
            vccLabel.Text = "VCC: -";
            isInAcq = false;
        }

        private async void btnAcquisition_Click(object sender, EventArgs e)
        {
            if (isInAcq)
            {
                stopAcquisition();
                btnAcquisition.Text = "Start Acquisition";
                groupBoxAcq.Enabled = true;
                updateButtons();
            }
            else
            {
                {
                    fftChart.Titles[0].ForeColor = Color.Black;
                    fftChart.Titles[0].Text = "Acquisition is stopped";
                    fftChart.Series.Clear();
                    fftChart.Update();

                    clearAcAnalysis();


                    HistogramChart.Titles[0].ForeColor = Color.Black;
                    HistogramChart.Titles[0].Text = "Acquisition is stopped";
                    HistogramChart.Series.Clear();
                    HistogramChart.Update();
                }

                ushort mask = 0x0000;
                mask |= cbDigitalCh1.Checked ? (ushort) 0x0001 : (ushort) 0x0;
                mask |= cbDigitalCh2.Checked ? (ushort) 0x0002 : (ushort) 0x0;
                mask |= cbAnalCh1.Checked ? (ushort)0x0004 : (ushort) 0x0;
                mask |= cbAnalCh2.Checked ? (ushort)0x0008 : (ushort)0x0;
                mask |= cbAnalCh3.Checked ? (ushort)0x0010 : (ushort)0x0;
                mask |= cbAnalCh4.Checked ? (ushort)0x0020 : (ushort)0x0;
                mask |= cbAnalCh5.Checked ? (ushort)0x0040 : (ushort)0x0;
                mask |= cbAnalCh6.Checked ? (ushort)0x0080 : (ushort)0x0;
                mask |= cbAnalCh7.Checked ? (ushort)0x0100 : (ushort)0x0;
                mask |= cbAnalCh8.Checked ? (ushort)0x0200 : (ushort)0x0;

                int selectedChannels = 0;

                if (cbContinuous.Checked)
                {
                    mSelectedChannel = -1;
                }
                else
                {
                    ushort maskCopy = mask;
                    for (int i = 0; i < 10; i++)
                    {
                        if ((maskCopy & 0x1) == 0x1)
                        {
                            mSelectedChannel = i;
                            selectedChannels++;
                        }
                        maskCopy >>= 1;
                    }
                    if (selectedChannels != 1)
                        mSelectedChannel = -1;

                    int channelBuffer = (int)Math.Floor(8192.0 / selectedChannels);

                    mChannelBufferLength = channelBuffer;
                }

                chartInit();

                // set acquisition parameters
                board.SetChannelMask(mask);
                board.SetExperimentPeriod((int) nudExperimentPeriod.Value);
                //board.SetSampleLength((int) nudDigitalSampLength.Value, (int) nudAnalogSampLength.Value);
                board.SetSampleLength(mChannelBufferLength, mChannelBufferLength);
                board.SetSamplePeriod((int)(10*double.Parse(txtDigitalSamplingPeriod.Text)), (int)(10*double.Parse(txtAnalSamplingPeriod.Text)));
                
                // enable visible channels and hide the others
                chart.ChartAreas[0].Visible = cbDigitalCh1.Checked;
                chart.ChartAreas[1].Visible = cbDigitalCh2.Checked;
                chart.ChartAreas[2].Visible = cbAnalCh1.Checked;
                chart.ChartAreas[3].Visible = cbAnalCh2.Checked;
                chart.ChartAreas[4].Visible = cbAnalCh3.Checked;
                chart.ChartAreas[5].Visible = cbAnalCh4.Checked;
                chart.ChartAreas[6].Visible = cbAnalCh5.Checked;
                chart.ChartAreas[7].Visible = cbAnalCh6.Checked;
                chart.ChartAreas[8].Visible = cbAnalCh7.Checked;
                chart.ChartAreas[9].Visible = cbAnalCh8.Checked;

                for (int ch = 0; ch < 10; ch++)
                {
                    chart.ChartAreas[ch].AxisX.Minimum = 0;
                    chart.ChartAreas[ch].AxisX.Crossing = 0;
                    for (int i = 0; i < MAX_CONTINUOUS_SAMPLES; i++)
                    {
                        mChannelContinuousData[ch][i] = double.NaN;
                    }
                }
                
                startAcquisition(cbContinuous.Checked);
                btnAcquisition.Text = "Stop Acquisition";
                groupBoxAcq.Enabled = false;
                updateButtons();
            }
        }

        private void btnSetTime_Click(object sender, EventArgs e)
        {
            TimeSpan epochTicks = new TimeSpan(new DateTime(1970, 1, 1).Ticks);
            TimeSpan unixTicks = new TimeSpan(DateTime.UtcNow.Ticks) - epochTicks;
            long unixTime = (long)unixTicks.TotalSeconds;

            //board.write("$settime\r\n");
            //Console.WriteLine(board.readLine());

            //board.write(String.Format("$settime {0}\r\n", unixTime));
            //Console.WriteLine(board.readLine());
            // TODO check output
        }

        private void btnConnect_Click(object sender, EventArgs e)
        {
            if (board.IsConnected())
                board.Disconnect();
            else
            {
                try
                {
                    board.Connect(tbSerialPort.Text);
                    board.Initialize();
                }
                catch (Exception ex)
                {
                    MessageBox.Show(ex.Message, "Connection Error");
                    return;
                }
            }

            if (board.IsConnected())
            {
                // get rid of any duplications
                string welcomeMsg = board.WelcomeMessage();
                if (!String.IsNullOrWhiteSpace(welcomeMsg))
                {
                    int i = welcomeMsg.IndexOf("High", 1);

                    if (i >= 0)
                         welcomeMsg = welcomeMsg.Substring(0, i);
                }
                lblBoardDetails.Text = welcomeMsg;
            }
            else
            {
                lblBoardDetails.Text = "No Board Connected";
            }
            updateButtons();
        }

        private void updateButtons()
        {
            // set connect button state depending on the connection to the device
            if (board.IsConnected())
            {
                tbSerialPort.Enabled = false;
                btnConnect.Text = "Disconnect";
            }
            else
            {
                tbSerialPort.Enabled = true;
                btnConnect.Text = "Connect";
            }

            // board connected and at least one channel is selected
            if (board.IsConnected() && mSelectedChannels > 0)
            {
                btnAcquisition.Enabled = true;
                // single shot disabled if continuous mode is selected
                if (isInAcq || cbContinuous.Checked)
                    btnSingleAcquisition.Enabled = false;
                else
                    btnSingleAcquisition.Enabled = true;
            }
            else // no channels selected
            {
                if (cbContinuous.Checked)
                    btnAcquisition.Enabled = true;
                else
                    btnAcquisition.Enabled = false;
                btnSingleAcquisition.Enabled = false;
            }
        }

        private void btnSingleAcquisition_Click(object sender, EventArgs e)
        {
            singleShotAcquisition();
        }

        private void btnCsvExport_Click(object sender, EventArgs e)
        {
            StreamWriter csvFile;

            // always export 0th channel in case none is selected
            int channel = mSelectedChannel == -1 ? 0 : mSelectedChannel;

            if (saveCsvDialog.ShowDialog() != DialogResult.OK)
                return;
            
            string filename = saveCsvDialog.FileName;
            if (File.Exists(filename))
                File.Delete(filename);

            csvFile = new StreamWriter(filename);

#if false
            csvFile.WriteLine("{0:000000.000000}", (double)1000000 / double.Parse(txtDigitalSamplingPeriod.Text));
            csvFile.WriteLine("16.000000");
            csvFile.WriteLine("0.000000");
            csvFile.WriteLine("2.500000");
            csvFile.WriteLine("0.000000");

            for (int i = 0; i < mChannelBufferLength; i++)
                csvFile.WriteLine(string.Format("{0,5}.000000,", mChannelRawData[channel][i]));
#endif
            for (int i = 0; i < 10; i++)
            {
                string line = String.Format("{0},", i);
                for (int j = 0; j < mChannelBufferLength; j++)
                {
                    line += string.Format("{0}", mChannelRawData[i][j]);
                    if (j < mChannelBufferLength - 1)
                        line += ",";
                }
                csvFile.WriteLine(line);
             }


            csvFile.Close();
            return;
        }

        private void labelFreq1st_Click(object sender, EventArgs e)
        {
            // do nothing
        }

        private void clearAcAnalysis()
        {
            labelMinAmplitude.Text = "";
            labelMaxAmplitude.Text = "";
            labelPkPkAmplitude.Text = "";
            labelDcAmplitude.Text = "";
            labelFundamentalAmplitude.Text = "";
            labelFundamentalFrequency.Text = "";
            labelRms.Text = "";

            labelFreqFund.Text = "";
            labelFreq2nd.Text = "";
            labelFreq3rd.Text = "";
            labelFreq4th.Text = "";
            labelFreq5th.Text = "";
            labelAmpFund.Text = "";
            labelAmp2nd.Text = "";
            labelAmp3rd.Text = "";
            labelAmp4th.Text = "";
            labelAmp5th.Text = "";

            labelSinad.Text = "";
            labelThd.Text = "";
            labelSnr.Text = "";
            labelDynamicRange.Text = "";
            labelNoiseFloor.Text = "";
        }

        private void comboBox1_SelectionChangeCommitted(object sender, EventArgs e)
        {
            clearAcAnalysis();
        }

        private void frmMainWindow_KeyDown(object sender, KeyEventArgs e)
        {
            if (tabControl1.SelectedIndex != 1) return;

            // catch ctrl+c key combination
            if (e.Control && e.KeyCode == Keys.C)
            {
                string xlsData = "";
                string cpuid = "0x00000000";

                string welcomeMsg = board.WelcomeMessage();

                if (!String.IsNullOrWhiteSpace(welcomeMsg))
                {
                    int cpuidPos = welcomeMsg.IndexOf("CPUID");

                    if (cpuidPos > 0)
                        cpuid = welcomeMsg.Substring(cpuidPos + 8, 10);
                }

                xlsData += cpuid + '\t';

                xlsData += mSelectedChannel.ToString() + '\t';
                xlsData += mTemperature.ToString() + '\t';

                if (vccLabel.Text == "VCC: -")
                    xlsData += "0\t";
                else
                    xlsData += vccLabel.Text.Substring(5, 4) + '\t';

                xlsData += labelMaxAmplitude.Text.Split(' ')[0] + '\t';
                xlsData += labelMinAmplitude.Text.Split(' ')[0] + '\t';
                xlsData += labelPkPkAmplitude.Text.Split(' ')[0] + '\t';
                xlsData += labelDcAmplitude.Text.Split(' ')[0] + '\t';
                xlsData += labelFundamentalAmplitude.Text.Split(' ')[0] + '\t';
                xlsData += labelFundamentalFrequency.Text.Split(' ')[0] + '\t';
                xlsData += labelRms.Text + '\t';

                xlsData += labelFreqFund.Text.Split(' ')[0] + '\t';
                xlsData += labelFreq2nd.Text.Split(' ')[0] + '\t';
                xlsData += labelFreq3rd.Text.Split(' ')[0] + '\t';
                xlsData += labelFreq4th.Text.Split(' ')[0] + '\t';
                xlsData += labelFreq5th.Text.Split(' ')[0] + '\t';

                xlsData += labelAmpFund.Text.Split(' ')[0] + '\t';
                xlsData += labelAmp2nd.Text.Split(' ')[0] + '\t';
                xlsData += labelAmp3rd.Text.Split(' ')[0] + '\t';
                xlsData += labelAmp4th.Text.Split(' ')[0] + '\t';
                xlsData += labelAmp5th.Text.Split(' ')[0] + '\t';

                xlsData += labelDynamicRange.Text.Split(' ')[0] + '\t';
                xlsData += labelSnr.Text.Split(' ')[0] + '\t';
                xlsData += labelThd.Text.Split(' ')[0] + '\t';
                xlsData += labelSinad.Text.Split(' ')[0] + '\t';
                xlsData += labelNoiseFloor.Text.Split(' ')[0] + '\t';
                
                Clipboard.SetText(xlsData);
                Debug.WriteLine("ctrl+c hijacked");
            }
        }

        private void cbContinuous_CheckedChanged(object sender, EventArgs e)
        {
            updateButtons();
            groupBox1.Enabled = !cbContinuous.Checked;
        }

        // digital sampling period control validation
        private void txtDigitalSamplingPeriod_Leave(object sender, EventArgs e)
        {
            try
            {
                var value = Single.Parse(txtDigitalSamplingPeriod.Text);
                if (value < 2.4) value = 2.4f;
                txtDigitalSamplingPeriod.Text = value.ToString("0.0");
            }
            catch (Exception ex)
            {
                txtDigitalSamplingPeriod.Text = "2.4";
            }
            var newValue = Single.Parse(txtDigitalSamplingPeriod.Text);
            lblRateDigital.Text = "Sample Rate: " + ((double)1000000.0 / newValue).ToString("0.00") + " (sps)"; 
            updateChannelSelection();
        }

        // analogue sampling period control validation
        private void txtAnalSamplingPeriod_Leave(object sender, EventArgs e)
        {
            try
            {
                var newValue = Single.Parse(txtAnalSamplingPeriod.Text);
                if (newValue < 2.4) newValue = 2.4f;
                txtAnalSamplingPeriod.Text = newValue.ToString("0.0");    
            }
            catch (Exception ex)
            {
                txtAnalSamplingPeriod.Text = "2.4";
            }
            var value = Single.Parse(txtAnalSamplingPeriod.Text);
            lblRateAnalog.Text = "Sample Rate: " + ((double)1000000.0 / value).ToString("0.00") + " (sps)"; 
            updateChannelSelection();
        }

        private void updateChannelSelection()
        {
            // get channel mask
            ushort mask = 0x0000;
            mask |= cbDigitalCh1.Checked ? (ushort)0x0001 : (ushort)0x0;
            mask |= cbDigitalCh2.Checked ? (ushort)0x0002 : (ushort)0x0;
            mask |= cbAnalCh1.Checked ? (ushort)0x0004 : (ushort)0x0;
            mask |= cbAnalCh2.Checked ? (ushort)0x0008 : (ushort)0x0;
            mask |= cbAnalCh3.Checked ? (ushort)0x0010 : (ushort)0x0;
            mask |= cbAnalCh4.Checked ? (ushort)0x0020 : (ushort)0x0;
            mask |= cbAnalCh5.Checked ? (ushort)0x0040 : (ushort)0x0;
            mask |= cbAnalCh6.Checked ? (ushort)0x0080 : (ushort)0x0;
            mask |= cbAnalCh7.Checked ? (ushort)0x0100 : (ushort)0x0;
            mask |= cbAnalCh8.Checked ? (ushort)0x0200 : (ushort)0x0;

            int selectedChannels = 0;
            int selectedAnalogChannels = 0;

            // count selected channels
            ushort maskCopy = mask;
            for (int i = 0; i < 10; i++)
            {
                if ((maskCopy & 0x1) == 0x1)
                {
                    selectedChannels++;
                    if (i >= 2)
                        selectedAnalogChannels++;
                }
                maskCopy >>= 1;
            }
            mSelectedChannels = selectedChannels;

            // if 0, for the calculation process we round these to 1
            if (selectedChannels == 0)
                selectedChannels = 1;
            if (selectedAnalogChannels == 0)
                selectedAnalogChannels = 1;

            int channelBuffer = (int)Math.Floor(8192.0 / selectedChannels);

            lblBufSizePerChannel.Text = "Buffer Size per Channel: "+ channelBuffer.ToString();

            var value = Single.Parse(txtAnalSamplingPeriod.Text);
            lblRateAnalog.Text = "Sample Rate: " + ((double)1000000.0 / value).ToString("0.00") + " (sps)";

            lblRatePerChannel.Text = "Sample Rate per Channel: " + (1e6 / (selectedAnalogChannels *value)).ToString("0.00") +" (sps)";

            // changing selection needs us to update buttons
            updateButtons();
        }

        private void cbDigitalCh1_CheckedChanged(object sender, EventArgs e)
        {
            updateChannelSelection();
        }

        private void cbDigitalCh2_CheckedChanged(object sender, EventArgs e)
        {
            updateChannelSelection();
        }

        private void cbAnalCh1_CheckedChanged(object sender, EventArgs e)
        {
            updateChannelSelection();
        }

        private void cbAnalCh2_CheckedChanged(object sender, EventArgs e)
        {
            updateChannelSelection();
        }

        private void cbAnalCh3_CheckedChanged(object sender, EventArgs e)
        {
            updateChannelSelection();
        }

        private void cbAnalCh4_CheckedChanged(object sender, EventArgs e)
        {
            updateChannelSelection();
        }

        private void cbAnalCh5_CheckedChanged(object sender, EventArgs e)
        {
            updateChannelSelection();
        }

        private void cbAnalCh6_CheckedChanged(object sender, EventArgs e)
        {
            updateChannelSelection();
        }

        private void cbAnalCh7_CheckedChanged(object sender, EventArgs e)
        {
            updateChannelSelection();
        }

        private void cbAnalCh8_CheckedChanged(object sender, EventArgs e)
        {
            updateChannelSelection();
        }

        private double parallel(double x, double y)
        {
            return 1.0f / (1.0f / x + 1.0f / y);
        }

        private double series(double x, double y)
        {
            return y / (x + y);
        }
        
        private UInt32 rtdOhmsToAdcCodeSum(UInt32 x)
        {
            UInt32 ADC_FULLSCALE  = 65535;
            UInt32 RTD_R_PULLUP   = 1000;  //Value of onboard pullup resistor
            UInt32 RTD_R_PULLDOWN = 1000;  //Value of onboard pulldown resistor
            float  ADC_R_BIAS_LOWER = 100000.0f;
            float  ADC_R_BIAS_UPPER = 100000.0f;
            UInt32 configRTD_SAMPLES = 1; // ?

            // TODO konwersja?
            return (UInt32) ((configRTD_SAMPLES * (double) ADC_FULLSCALE)
                                       * series (
                                            parallel(ADC_R_BIAS_UPPER, RTD_R_PULLUP),
                                            parallel(ADC_R_BIAS_LOWER, (double)x + RTD_R_PULLDOWN)));
        }

        private UInt32 getTempK10_from_adcCodeSum(UInt32 adcCodeSum)
        {
            //in ohms
            UInt32 RTD_RES_N40 =  843;
            UInt32 RTD_RES_0   = 1000;
            UInt32 RTD_RES_40  = 1155;
            UInt32 RTD_RES_80  = 1309;
            UInt32 RTD_RES_120 = 1461;
            UInt32 RTD_RES_160 = 1610;
            UInt32 RTD_RES_200 = 1759;
            UInt32 RTD_RES_240 = 1905;

            UInt32 RTD_ADCCODESUM_N40 = rtdOhmsToAdcCodeSum(RTD_RES_N40);
            UInt32 RTD_ADCCODESUM_0   = rtdOhmsToAdcCodeSum(RTD_RES_0);
            UInt32 RTD_ADCCODESUM_40  = rtdOhmsToAdcCodeSum(RTD_RES_40);
            UInt32 RTD_ADCCODESUM_80  = rtdOhmsToAdcCodeSum(RTD_RES_80);
            UInt32 RTD_ADCCODESUM_120 = rtdOhmsToAdcCodeSum(RTD_RES_120);
            UInt32 RTD_ADCCODESUM_160 = rtdOhmsToAdcCodeSum(RTD_RES_160);
            UInt32 RTD_ADCCODESUM_200 = rtdOhmsToAdcCodeSum(RTD_RES_200);
            UInt32 RTD_ADCCODESUM_240 = rtdOhmsToAdcCodeSum(RTD_RES_240); 
            
            //slopes from low to high
            UInt32 RTD_C10_PER_CODESUM_N40 = (65536*400/(RTD_ADCCODESUM_0  -RTD_ADCCODESUM_N40));
            UInt32 RTD_C10_PER_CODESUM_0   = (65536*400/(RTD_ADCCODESUM_40 -RTD_ADCCODESUM_0  ));
            UInt32 RTD_C10_PER_CODESUM_40  = (65536*400/(RTD_ADCCODESUM_80 -RTD_ADCCODESUM_40 ));
            UInt32 RTD_C10_PER_CODESUM_80  = (65536*400/(RTD_ADCCODESUM_120-RTD_ADCCODESUM_80 ));
            UInt32 RTD_C10_PER_CODESUM_120 = (65536*400/(RTD_ADCCODESUM_160-RTD_ADCCODESUM_120));
            UInt32 RTD_C10_PER_CODESUM_160 = (65536*400/(RTD_ADCCODESUM_200-RTD_ADCCODESUM_160));
            UInt32 RTD_C10_PER_CODESUM_200 = (65536*400/(RTD_ADCCODESUM_240-RTD_ADCCODESUM_200));


            UInt32 tempK10;
            UInt32 deltaCodeSum;
            UInt32 slope;

            //adcCodeSum *= 4;

            if (adcCodeSum < RTD_ADCCODESUM_N40)
                return 0;
            if (adcCodeSum > RTD_ADCCODESUM_240)
                return 0;
            if (adcCodeSum < RTD_ADCCODESUM_0)
            {
                tempK10 = -400 + 2732;
                deltaCodeSum = adcCodeSum - RTD_ADCCODESUM_N40;
                //25C,deltaCodeSum=993
                slope = RTD_C10_PER_CODESUM_N40;
            }
            else if (adcCodeSum < RTD_ADCCODESUM_40)
            {
                tempK10 = 0 + 2732;
                deltaCodeSum = adcCodeSum - RTD_ADCCODESUM_0;
                //25C,deltaCodeSum=993
                slope = RTD_C10_PER_CODESUM_0;
            }
            else if (adcCodeSum < RTD_ADCCODESUM_80)
            {
                tempK10 = 400 + 2732;
                deltaCodeSum = adcCodeSum - RTD_ADCCODESUM_40;
                slope = RTD_C10_PER_CODESUM_40;
            }
            else if (adcCodeSum < RTD_ADCCODESUM_120)
            {
                tempK10 = 800 + 2732;
                deltaCodeSum = adcCodeSum - RTD_ADCCODESUM_80;
                slope = RTD_C10_PER_CODESUM_80;
            }
            else if (adcCodeSum < RTD_ADCCODESUM_160)
            {
                tempK10 = 1200 + 2732;
                deltaCodeSum = adcCodeSum - RTD_ADCCODESUM_120;
                slope = RTD_C10_PER_CODESUM_120;
            }
            else if (adcCodeSum < RTD_ADCCODESUM_200)
            {
                tempK10 = 1600 + 2732;
                deltaCodeSum = adcCodeSum - RTD_ADCCODESUM_160;
                slope = RTD_C10_PER_CODESUM_160;
            }
            else
            {
                tempK10 = 2000 + 2732;
                deltaCodeSum = adcCodeSum - RTD_ADCCODESUM_200;
                slope = RTD_C10_PER_CODESUM_200;
            }

            tempK10 += (deltaCodeSum * slope) >> 16;
            return tempK10;
        }

        private void frmMainWindow_FormClosing(object sender, FormClosingEventArgs e)
        {
            // stop the acquisition before leaving
            if (isInAcq)
                stopAcquisition(true);
        }

        private void tabPage3_Click(object sender, EventArgs e)
        {

        }

        private void nudContSampRate_ValueChanged(object sender, EventArgs e)
        {
            if ((float)nudContSampRate.Value < 1.0)
                nudContSampRate.Value = 0;
            else if ((float)nudContSampRate.Value < 2.0)
                nudContSampRate.Value = 2;
        }

        private void label2_Click(object sender, EventArgs e)
        {

        }
    }
}
