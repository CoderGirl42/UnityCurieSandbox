using UnityEngine;
using System.IO.Ports;
using System.Threading;

public class SerialGyro : MonoBehaviour {

    private SerialPort serial;

    // Use this for initialization
    void Start () {
        // Create the Serial Port - Currently uses first port may need to change for other implimentations.
        serial = new SerialPort(SerialPort.GetPortNames()[0], 9600, Parity.None, 8, StopBits.One);

        // Setup Serial Port Settings.
        serial.DtrEnable = true;
        serial.RtsEnable = true;  
        serial.ReadTimeout = 1000;

        Thread.Sleep(100); //allow time for settings to be set. Unity has issues without this.

        // Open the port.
        serial.Open();

        // Calibrate the device.
        serial.Write("c");
    }

    void OnDestroy() {
        // Cleanup serial port.
        if(serial.IsOpen) {
            serial.Close();
        }
    }

    // Update is called once per frame
    void Update() {
        float yaw = 0, pitch = 0, roll = 0;

        // Send a command to the Curie
        serial.Write("s");

        // Recieve the response from Curie
        var msg = serial.ReadLine();
        
        // Parse the Values from the response message.
        if (msg != string.Empty) {
            string[] ypr = msg.Split(',');

            yaw = float.Parse(ypr[0]); // convert to float yaw
            pitch = float.Parse(ypr[1]); // convert to float pitch
            roll = float.Parse(ypr[2]); // convert to float roll

            transform.eulerAngles += new Vector3(pitch, yaw, roll);

            //transform.Rotate(new Vector3(1, 0, 0), pitch);
            //transform.Rotate(new Vector3(0, 1, 0), yaw);
            //transform.Rotate(new Vector3(0, 0, 1), roll);

            //transform.Rotate(Vector3.right * pitch);
            //transform.Rotate(Vector3.up * yaw);
            //transform.Rotate(Vector3.forward * roll);
        }
    }
}
