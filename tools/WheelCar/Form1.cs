using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO.Ports;
using System.Threading;

namespace WheelCar
{
    public partial class Form1 : Form
    {
        public Form1()
        {
            InitializeComponent();
        }

        private void button1_Click(object sender, EventArgs e)
        {
            serialPort1.PortName = textBox1.Text;
            serialPort1.DataReceived += new
                SerialDataReceivedEventHandler(port_DataReceived);
            serialPort1.Open();

            textBox2.AppendText("Connected");
            textBox2.AppendText(Environment.NewLine);
        }

        private void port_DataReceived(object sender,
                                    SerialDataReceivedEventArgs e)
        {
            // Show all the incoming data in the port's buffer
            //Console.WriteLine(serialPort1.ReadLine());

            string s = serialPort1.ReadLine();

            /*textBox2.Invoke((Action)delegate
            {
                textBox2.AppendText(s);
                textBox2.AppendText(Environment.NewLine);
            });*/

            var parts = s.Split(' ');
            string cmd = parts[0];
            if (cmd == "imu")
            {
                /*float roll = Convert.ToInt32(parts[1]);
                string pitch = Convert.ToInt32(parts[2]);
                string heading = Convert.ToInt32(parts[3]);*/

                textBox3.Invoke((Action)delegate
                {
                    textBox3.Text = parts[1];
                });
                textBox4.Invoke((Action)delegate
                {
                    textBox4.Text = parts[2];
                });
                textBox5.Invoke((Action)delegate
                {
                    textBox5.Text = parts[3];
                });
            }
        }

        private void button2_Click(object sender, EventArgs e)
        {
            serialPort1.Close();
        }

        private void textBox6_TextChanged(object sender, EventArgs e)
        {

        }

        private void button3_Click(object sender, EventArgs e)
        {
            string s = "1 " + textBox6.Text + " " + textBox7.Text;
            serialPort1.WriteLine(s);
            textBox2.Invoke((Action)delegate
            {
                textBox2.AppendText(s);
                textBox2.AppendText(Environment.NewLine);
            });
        }
    }
}
