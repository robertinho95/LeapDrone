using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Drawing.Imaging;
using Leap;
using System.IO;

using vJoyInterfaceWrap;
using System.Threading;

namespace LeapDrone
{
    public partial class Form1 : Form
    {
        // ******************************************* vJOY *********************************************

        // Declaring one joystick (Device id 1) and a position structure. 
        static public vJoy joystick;
        static public vJoy.JoystickState iReport;
        static public uint id = 1; // check ID!!!
        static public long maxvalue = 0;
        static public long minvalue = 0;
        static int X, Y, RX, RY;
        static Thread t;


        // ******************************************* **** *********************************************

        static public Hand myHand;
        static public CircularBuffer<HandValue> myCircularBuff;
        static public Logger myLog;

        private byte[] imagedata = new byte[1];
        private Controller controller = new Controller();
        Bitmap bitmap = new Bitmap(640, 480, System.Drawing.Imaging.PixelFormat.Format8bppIndexed);

        public Form1()
        {
            InitializeComponent();

            // ******************************************* **** *********************************************
            // **************************************** start_Vjoy ******************************************
            // ******************************************* **** *********************************************

            // Create one joystick object and a position structure.
            joystick = new vJoy();
            iReport = new vJoy.JoystickState();

            // Get the driver attributes (Vendor ID, Product ID, Version Number)
            if (!joystick.vJoyEnabled())
            {
                Console.WriteLine("vJoy driver not enabled: Failed Getting vJoy attributes.\n");
                return;
            }
            else
                Console.WriteLine("Vendor: {0}\nProduct :{1}\nVersion Number:{2}\n", joystick.GetvJoyManufacturerString(), joystick.GetvJoyProductString(), joystick.GetvJoySerialNumberString());

            // Get the state of the requested device
            VjdStat status = joystick.GetVJDStatus(id);
            switch (status)
            {
                case VjdStat.VJD_STAT_OWN:
                    Console.WriteLine("vJoy Device {0} is already owned by this feeder\n", id);
                    break;
                case VjdStat.VJD_STAT_FREE:
                    Console.WriteLine("vJoy Device {0} is free\n", id);
                    break;
                case VjdStat.VJD_STAT_BUSY:
                    Console.WriteLine("vJoy Device {0} is already owned by another feeder\nCannot continue\n", id);
                    return;
                case VjdStat.VJD_STAT_MISS:
                    Console.WriteLine("vJoy Device {0} is not installed or disabled\nCannot continue\n", id);
                    return;
                default:
                    Console.WriteLine("vJoy Device {0} general error\nCannot continue\n", id);
                    return;
            };

            // Check which axes are supported
            bool AxisX = joystick.GetVJDAxisExist(id, HID_USAGES.HID_USAGE_X);
            bool AxisY = joystick.GetVJDAxisExist(id, HID_USAGES.HID_USAGE_Y);
            bool AxisZ = joystick.GetVJDAxisExist(id, HID_USAGES.HID_USAGE_Z);
            bool AxisRX = joystick.GetVJDAxisExist(id, HID_USAGES.HID_USAGE_RX);
            bool AxisRY = joystick.GetVJDAxisExist(id, HID_USAGES.HID_USAGE_RY);
            bool AxisRZ = joystick.GetVJDAxisExist(id, HID_USAGES.HID_USAGE_RZ);
            // Get the number of buttons and POV Hat switchessupported by this vJoy device
            int nButtons = joystick.GetVJDButtonNumber(id);
            int ContPovNumber = joystick.GetVJDContPovNumber(id);
            int DiscPovNumber = joystick.GetVJDDiscPovNumber(id);

            // Print results
            Console.WriteLine("\nvJoy Device {0} capabilities:\n", id);
            Console.WriteLine("Number of buttons\t\t{0}\n", nButtons);
            Console.WriteLine("Number of Continuous POVs\t{0}\n", ContPovNumber);
            Console.WriteLine("Number of Descrete POVs\t\t{0}\n", DiscPovNumber);
            Console.WriteLine("Axis X\t\t{0}\n", AxisX ? "Yes" : "No");
            Console.WriteLine("Axis Y\t\t{0}\n", AxisY ? "Yes" : "No");
            Console.WriteLine("Axis Z\t\t{0}\n", AxisZ ? "Yes" : "No");
            Console.WriteLine("Axis Rx\t\t{0}\n", AxisRX ? "Yes" : "No");
            Console.WriteLine("Axis Rx\t\t{0}\n", AxisRY ? "Yes" : "No");
            Console.WriteLine("Axis Rz\t\t{0}\n", AxisRZ ? "Yes" : "No");

            // Test if DLL matches the driver
            UInt32 DllVer = 0, DrvVer = 0;
            bool match = joystick.DriverMatch(ref DllVer, ref DrvVer);
            if (match)
                Console.WriteLine("Version of Driver Matches DLL Version ({0:X})\n", DllVer);
            else
                Console.WriteLine("Version of Driver ({0:X}) does NOT match DLL Version ({1:X})\n", DrvVer, DllVer);


            // Acquire the target
            if ((status == VjdStat.VJD_STAT_OWN) || ((status == VjdStat.VJD_STAT_FREE) && (!joystick.AcquireVJD(id))))
            {
                Console.WriteLine("Failed to acquire vJoy device number {0}.\n", id);
                return;
            }
            else
                Console.WriteLine("Acquired: vJoy device number {0}.\n", id);

            joystick.GetVJDAxisMax(id, HID_USAGES.HID_USAGE_X, ref maxvalue);
            Console.WriteLine("Max value X: {0}.\n", maxvalue);
            joystick.GetVJDAxisMax(id, HID_USAGES.HID_USAGE_Y, ref maxvalue);
            Console.WriteLine("Max value Y: {0}.\n", maxvalue);
            joystick.GetVJDAxisMax(id, HID_USAGES.HID_USAGE_RX, ref maxvalue);
            Console.WriteLine("Max value RX: {0}.\n", maxvalue);
            joystick.GetVJDAxisMax(id, HID_USAGES.HID_USAGE_RY, ref maxvalue);
            Console.WriteLine("Max value RY: {0}.\n", maxvalue);

            joystick.GetVJDAxisMin(id, HID_USAGES.HID_USAGE_X, ref minvalue);
            Console.WriteLine("Min value X: {0}.\n", minvalue);
            joystick.GetVJDAxisMin(id, HID_USAGES.HID_USAGE_Y, ref minvalue);
            Console.WriteLine("Min value Y: {0}.\n", minvalue);
            joystick.GetVJDAxisMin(id, HID_USAGES.HID_USAGE_RX, ref minvalue);
            Console.WriteLine("Min value RX: {0}.\n", minvalue);
            joystick.GetVJDAxisMin(id, HID_USAGES.HID_USAGE_RY, ref minvalue);
            Console.WriteLine("Min value RY: {0}.\n", minvalue);

            this.minValJOY.Text = minvalue.ToString();
            this.maxValJOY.Text = maxvalue.ToString();

            // Reset this device to default values
            joystick.ResetVJD(id);

            yawProg.Minimum = (int)minvalue;
            yawProg.Maximum = (int)maxvalue;
            rollProg.Minimum = (int)minvalue;
            rollProg.Maximum = (int)maxvalue;
            pitchProg.Minimum = (int)minvalue;
            pitchProg.Maximum = (int)maxvalue;
            heightProg.Minimum = (int)minvalue;
            heightProg.Maximum = (int)maxvalue;

            rollChart.ChartAreas[0].AxisY.Maximum = maxvalue;
            rollChart.ChartAreas[0].AxisY.Minimum = minvalue;
            pitchChart.ChartAreas[0].AxisY.Maximum = maxvalue;
            pitchChart.ChartAreas[0].AxisY.Minimum = minvalue;
            heightChart.ChartAreas[0].AxisY.Maximum = maxvalue;
            heightChart.ChartAreas[0].AxisY.Minimum = minvalue;
            yawChart.ChartAreas[0].AxisY.Maximum = maxvalue;
            yawChart.ChartAreas[0].AxisY.Minimum = minvalue;

            myCircularBuff = new CircularBuffer<HandValue>(2000);

            myLog = new Logger();

            t = new Thread(joyThread);
            t.Start();

            // ******************************************* **** *********************************************
            // ***************************************** end_Vjoy *******************************************
            // ******************************************* **** *********************************************

            controller.EventContext = WindowsFormsSynchronizationContext.Current;
            controller.FrameReady += newFrameHandler;
            controller.ImageReady += onImageReady;
            controller.ImageRequestFailed += onImageRequestFailed;

            //set greyscale palette for image Bitmap object
            ColorPalette grayscale = bitmap.Palette;
            for (int i = 0; i < 256; i++)
            {
                grayscale.Entries[i] = Color.FromArgb((int)255, i, i, i);
            }
            bitmap.Palette = grayscale;
        }

        static void vJoyDataPack(Frame thisFrame, Form1 sender)
        {
            float ROLL = -99; // radinas
            float PITCH = -99; // radinas
            float YAW = -99; // radinas
            float THROTTLE = -99;
            // PACK data for Vjoy
            if (thisFrame.Hands.Count > 0)
            {
                myHand = thisFrame.Hands.ElementAt(0);
                ROLL = myHand.PalmNormal.Roll; // radinas
                PITCH = myHand.Direction.Pitch; // radinas
                YAW = myHand.Direction.Yaw; // radinas
                THROTTLE = myHand.PalmPosition.z; // millimeters
                if (THROTTLE < 200.0f) THROTTLE = 200.0f;
                else if (THROTTLE > 400.0f) THROTTLE = 400.0f;

                float costant_angle = (float)maxvalue / (1.0f * (float)Math.PI);
                float costant_distance = (float)maxvalue / (200.0f);
                float pi_half = (float)Math.PI / 2.0f;

                X = (int)((YAW + pi_half) * costant_angle);
                Y = (int)((THROTTLE - 200.0f) * costant_distance);
                RX = (int)((ROLL + pi_half) * costant_angle);
                RY = (int)((PITCH + pi_half) * costant_angle);

                if (X < minvalue) X = (int)minvalue; else if (X > maxvalue) X = (int)maxvalue;
                if (Y < minvalue) Y = (int)minvalue; else if (Y > maxvalue) Y = (int)maxvalue;
                if (RX < minvalue) RX = (int)minvalue; else if (RX > maxvalue) RX = (int)maxvalue;
                if (RY < minvalue) RY = (int)minvalue; else if (RY > maxvalue) RY = (int)maxvalue;

            }
            else
            {
                X =  (int)(minvalue + ((maxvalue - minvalue) / 2));
                RX = (int)(minvalue + ((maxvalue - minvalue) / 2));
                RY = (int)(minvalue + ((maxvalue - minvalue) / 2));
                Y =  (int)(minvalue);
            }
            //Display raw values
            sender.rollTXT.Text = ROLL.ToString();
            sender.pitchTXT.Text = PITCH.ToString();
            sender.yawTXT.Text = YAW.ToString();
            sender.heightTXT.Text = THROTTLE.ToString();

            //Display raw values
            sender.rollJOY.Text = RX.ToString();
            sender.pitchJOY.Text = RY.ToString();
            sender.yawJOY.Text = X.ToString();
            sender.heightJOY.Text = Y.ToString();

            sender.yawProg.Value = (int)X;
            sender.rollProg.Value = (int)RX;
            sender.pitchProg.Value = (int)RY;
            sender.heightProg.Value = (int)Y;

            HandValue thisHand = new HandValue();
            thisHand.Roll = RX;
            thisHand.Pitch = RY;
            thisHand.Yaw = X;
            thisHand.Height = Y;
            thisHand.Timestamp = thisFrame.Timestamp;

            myLog.LogHand(thisHand);

            if (sender.chartCheckBox.Checked)
                if ((thisFrame.Id % 3) == 0)
                    myCircularBuff.Add(thisHand);
        }

        void newFrameHandler(object sender, FrameEventArgs eventArgs)
        {
            Frame frame = eventArgs.frame;

            if(frameCheckBox.Checked)
                controller.RequestImages(frame.Id, Leap.Image.ImageType.DEFAULT, imagedata);

            //The following are Label controls added in design view for the form
            this.displayID.Text = frame.Id.ToString();
            this.displayTimestamp.Text = frame.Timestamp.ToString();
            this.displayFPS.Text = frame.CurrentFramesPerSecond.ToString();
            this.displayHandCount.Text = frame.Hands.Count.ToString();

            

            vJoyDataPack(frame, this);
        }

        void onImageRequestFailed(object sender, ImageRequestFailedEventArgs e)
        {
            if (e.reason == Leap.Image.RequestFailureReason.Insufficient_Buffer)
            {
                imagedata = new byte[e.requiredBufferSize];
            }
            Console.WriteLine("Image request failed: " + e.message);
        }

        private void chartRadioButton_CheckedChanged(object sender, EventArgs e)
        {
            if (linesRadioButton.Checked) {
                rollChart.Series["rollSeries"].ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Line;
                pitchChart.Series["pitchSeries"].ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Line;
                yawChart.Series["yawSeries"].ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Line;
                heightChart.Series["heightSeries"].ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Line;
            }
            else if (splineRadioButton.Checked) {
                rollChart.Series["rollSeries"].ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Line;
                pitchChart.Series["pitchSeries"].ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Line;
                yawChart.Series["yawSeries"].ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Line;
                heightChart.Series["heightSeries"].ChartType = System.Windows.Forms.DataVisualization.Charting.SeriesChartType.Line;
            }
        }

        private void loggerButton_Click(object sender, EventArgs e)
        {
            if (myLog.Enabled)
                myLog.stopLog();
            else
                myLog.startLog();

            if (myLog.Enabled)
                loggerButton.Text = "Stop logger";
            else
                loggerButton.Text = "Start logger";
        }

        private void onClose(object sender, FormClosingEventArgs e)
        {
            controller.FrameReady -= newFrameHandler;
            controller.ImageReady -= onImageReady;
            controller.ImageRequestFailed -= onImageRequestFailed;
            controller.Dispose();
            t.Abort();
            myLog.stopLog();
            Application.Exit();
        }

        void onImageReady(object sender, ImageEventArgs e)
        {
            Rectangle lockArea = new Rectangle(0, 0, bitmap.Width, bitmap.Height);
            BitmapData bitmapData = bitmap.LockBits(lockArea, ImageLockMode.WriteOnly, PixelFormat.Format8bppIndexed);
            byte[] rawImageData = imagedata;
            System.Runtime.InteropServices.Marshal.Copy(rawImageData, 0, bitmapData.Scan0, e.image.Width * e.image.Height * 2 * e.image.BytesPerPixel);
            bitmap.UnlockBits(bitmapData);
            displayImages.Image = bitmap;
        }

        public void joyThread()
        {
            while (true)
            {
                iReport.bDevice = (byte)id;
                iReport.AxisX = X;
                iReport.AxisY = Y;
                iReport.AxisXRot = RX;
                iReport.AxisYRot = RY;

                /*** Feed the driver with the position packet - is fails then wait for input then try to re-acquire device ***/
                if (!joystick.UpdateVJD(id, ref iReport))
                {
                    Console.WriteLine("Feeding vJoy device number {0} failed - try to enable device then press enter\n", id);
                    System.Threading.Thread.Sleep(40);
                    joystick.AcquireVJD(id);
                }
                
                if (IsHandleCreated && chartCheckBox.Checked)
                {
                    long from = myCircularBuff.FindIndex(5000000);
                    long to = myCircularBuff.buffLenght();

                    rollChart.Invoke((MethodInvoker)delegate
                    {
                        // Running on the UI thread
                        rollChart.Series["rollSeries"].Points.Clear();
                        if (from != -1)
                        {
                            for (long i = from; i < to; i++)
                            {
                                rollChart.Series["rollSeries"].Points.AddXY(myCircularBuff.Get(i).Timestamp, myCircularBuff.Get(i).Roll);
                            }
                        }
                    });
                    pitchChart.Invoke((MethodInvoker)delegate
                    {
                        // Running on the UI thread
                        pitchChart.Series["pitchSeries"].Points.Clear();
                        if (from != -1)
                        {
                            for (long i = from; i < to; i++)
                            {
                                pitchChart.Series["pitchSeries"].Points.AddXY(myCircularBuff.Get(i).Timestamp, myCircularBuff.Get(i).Pitch);
                            }
                        }
                    });
                    yawChart.Invoke((MethodInvoker)delegate
                    {
                        // Running on the UI thread
                        yawChart.Series["yawSeries"].Points.Clear();
                        if (from != -1)
                        {
                            for (long i = from; i < to; i++)
                            {
                                yawChart.Series["yawSeries"].Points.AddXY(myCircularBuff.Get(i).Timestamp, myCircularBuff.Get(i).Yaw);
                            }
                        }
                    });
                    heightChart.Invoke((MethodInvoker)delegate
                    {
                        // Running on the UI thread
                        heightChart.Series["heightSeries"].Points.Clear();
                        if (from != -1)
                        {
                            for (long i = from; i < to; i++)
                            {
                                heightChart.Series["heightSeries"].Points.AddXY(myCircularBuff.Get(i).Timestamp, myCircularBuff.Get(i).Height);
                            }
                        }
                    });
                }


                System.Threading.Thread.Sleep(40);


            }
        }
    }
    public class HandValue
    {
        private int yaw;
        private int height;
        private int roll;
        private int pitch;
        private long timestamp;

        public HandValue()
        {
            Roll = 0;
            Pitch = 0;
            Yaw = 0;
            Height = 0;
            Timestamp = 0;
        }

        public int Roll { get => roll; set => roll = value; }
        public int Pitch { get => pitch; set => pitch = value; }
        public int Yaw { get => yaw; set => yaw = value; }
        public int Height { get => height; set => height = value; }
        public long Timestamp { get => timestamp; set => timestamp = value; }
    } 

    public class CircularBuffer<T> where T : HandValue, new()
    {
        private T[] buffer;
        private int nextFree;

        public long buffLenght()
        {
            return buffer.Length;
        }

        public CircularBuffer(long lenght)
        {
            this.buffer = new T[lenght];
            for(long i = 0; i < lenght; i++)
            {
                buffer[i] = new T();
            }
            nextFree = 0;
        }

        public void Add(T o)
        {
            buffer[nextFree] = o;
            nextFree = (nextFree + 1) % buffer.Length;
        }

        public T Get(long index)
        {
            long lastindex = (buffer.Length + (nextFree - 1)) % buffer.Length;
            long rightIndex = lastindex + (index - (buffer.Length - 1));
            if (rightIndex < 0) rightIndex += buffer.Length;
            return buffer[rightIndex];
        }

        public long FindIndex(long time)
        {
            long rightIndex = -1;
            long rightIndexConv = -1;
            long lastindex = (buffer.Length + (nextFree - 1)) % buffer.Length;
            long timestamp = (buffer[lastindex].Timestamp - time);

            if (buffer[0].Timestamp <= timestamp)
            {
                for (long i = 0; i < buffer.Length; i++)
                {
                    if (buffer[i].Timestamp == timestamp) { rightIndex = i; break; }
                    else if (buffer[i].Timestamp > timestamp) { rightIndex = i - 1; break; }
                }
            }
            else // (buffer[0].Timestamp > timestamp)
            {
                for (long i = (buffer.Length - 1); i >= 0; i--)
                {
                    if (buffer[i].Timestamp < timestamp) { rightIndex = i; break; }
                }
            }

            if (rightIndex == -1)
                return -1;
            else
            {
                rightIndexConv = (rightIndex - lastindex + (buffer.Length - 1)) % buffer.Length;
                return rightIndexConv;
            }
            
        }
    }

    public class Logger
    {
        private StreamWriter sw;
        private bool enabled;

        public Logger() // file handler
        {

        }

        public void LogHand(HandValue h)
        {
            if(enabled)
                sw.WriteLine(String.Format("{0},{1},{2},{3}", h.Timestamp, h.Roll, h.Pitch, h.Yaw, h.Height));
        }
        public void startLog()
        {
            DateTime now = DateTime.Now;
            String filename = String.Format("{0}_{1}{2}{3}_{4}{5}_{6}.csv", "LeapDroneLog", now.Year, now.Month, now.Day, now.Hour, now.Minute, now.Second);
            String path = "";
            sw = new StreamWriter(path + filename);
            enabled = true;
        }
        public void stopLog()
        {
            if (enabled)
            {
                enabled = false;
                sw.Close();
                sw.Dispose();
            }
        }

        public bool Enabled { get => enabled; }

    }
}