using UnityEngine;
using System.IO.Ports;
using System.Threading;
using System.Collections;

public class GyroCamera : MonoBehaviour
{
    private float yaw = 0, pitch = 0, roll = 0;
    private float yaw_degrees = 0, pitch_degrees = 0, roll_degrees = 0;

    private SerialPort serial;

    // Use this for initialization
    void Start()
    {
        // Create the Serial Port - Currently uses first port may need to change for other implimentations.
        serial = new SerialPort(SerialPort.GetPortNames()[0], 9600, Parity.None, 8, StopBits.One);

        // Setup Serial Port Settings.
        serial.DtrEnable = true;
        serial.RtsEnable = true;
        serial.ReadTimeout = 1000;

        Thread.Sleep(100); //allow time for settings to be set. Unity has issues without this.

        // Open the port.
        serial.Open();

        // recalibrate the device.
        serial.Write("r");
    }

    void OnDestroy()
    {
        // Cleanup serial port.
        if (serial.IsOpen)
        {
            serial.Close();
        }
    }

    // Update is called once per frame
    void Update()
    {
        // Send a command to the Curie
        serial.Write("s");

        // Recieve the response from Curie
        var msg = serial.ReadLine();

        // Parse the Values from the response message.
        if (msg != string.Empty)
        {
            string[] ypr = msg.Split(',');

            float.TryParse(ypr[0], out yaw); // convert to float yaw
            float.TryParse(ypr[1], out pitch); // convert to float pitch
            float.TryParse(ypr[2], out roll); // convert to float roll

            yaw_degrees = yaw * 180.0f / 3.14159f; // conversion to degrees
            if (yaw_degrees < 0) yaw_degrees += 360.0f; // convert negative to positive angles

            pitch_degrees = pitch * 180.0f / 3.14159f; // conversion to degrees
            if (pitch_degrees < 0) pitch_degrees += 360.0f; // convert negative to positive angles

            roll_degrees = roll * 180.0f / 3.14159f; // conversion to degrees
            if (roll_degrees < 0) roll_degrees += 360.0f; // convert negative to positive angles

            Quaternion fromRotation = transform.rotation;
            Quaternion toRotation = Quaternion.Euler(pitch_degrees, yaw_degrees, -roll_degrees);

            transform.rotation = Quaternion.Slerp(fromRotation, toRotation, Time.deltaTime * 10);

        }
    }

    void OnGUI()
    {
        GUI.Label(new Rect(0, 0, 100, 100), "Raw Yaw:" + yaw.ToString());
        GUI.Label(new Rect(100, 0, 100, 100), "Raw Pitch:" + pitch.ToString());
        GUI.Label(new Rect(200, 0, 100, 100), "Raw Roll:" + roll.ToString());

        GUI.Label(new Rect(0, 20, 100, 100), "Yaw:" + yaw_degrees.ToString());
        GUI.Label(new Rect(100, 20, 100, 100), "Pitch:" + pitch_degrees.ToString());
        GUI.Label(new Rect(200, 20, 100, 100), "Roll:" + roll_degrees.ToString());
    }
}
