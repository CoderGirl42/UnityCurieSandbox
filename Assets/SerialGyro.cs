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
        serial.Write("r");
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

            float yaw_degrees = yaw * 180.0f / 3.14159f; // conversion to degrees
            if (yaw_degrees < 0) yaw_degrees += 360.0f; // convert negative to positive angles

            float pitch_degrees = pitch * 180.0f / 3.14159f; // conversion to degrees
            if (pitch_degrees < 0) pitch_degrees += 360.0f; // convert negative to positive angles

            float roll_degrees = roll * 180.0f / 3.14159f; // conversion to degrees
            if (roll_degrees < 0) roll_degrees += 360.0f; // convert negative to positive angles

            Quaternion fromRotation = transform.rotation;
            Quaternion toRotation = Quaternion.Euler(pitch_degrees, yaw_degrees, roll_degrees);
            transform.rotation = Quaternion.Lerp(fromRotation, toRotation, Time.deltaTime * 20);

        }
    }
}
