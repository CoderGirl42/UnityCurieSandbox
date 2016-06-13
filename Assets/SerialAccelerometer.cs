using UnityEngine;
using System.IO.Ports;
using System.Threading;

[RequireComponent(typeof(Rigidbody))]
public class SerialAccelerometer : MonoBehaviour {

    private SerialPort serial;

	// Use this for initialization
	void Start () {
        // Create the Serial Port - Currently uses first port may need to change for other implimentations.
        serial = new SerialPort(SerialPort.GetPortNames()[0], 9600, Parity.None, 8, StopBits.One);

        // Setup Serial Port Settings
        serial.DtrEnable = true;
        serial.RtsEnable = true;
        serial.ReadTimeout = 1000;

        Thread.Sleep(100); //allow time for settings to be set. Unity has issues without this.

        // Open the Port
        serial.Open();

        // Calibrate the Curie.
        serial.Write("c");
    }

    void OnDestroy() {
        // Cleanup Serial connection.
        if (serial.IsOpen) {
            serial.Close();
        }
    }

    // Update is called once per frame
    void Update() {
        float g = 9.8f;
        float ax = 0, ay = 0, az = 0;

        // Send command to Curie
        serial.Write("s");

        // Receive Response from Curie
        string msg = serial.ReadLine();

        // Parse Message.
        if (msg != string.Empty) {
            string[] ypr = msg.Split(',');

            float.TryParse(ypr[3], out ax);
            float.TryParse(ypr[4], out ay);
            float.TryParse(ypr[5], out az);
        }

        // Create gravity vector
        var gravity = new Vector3(-ax, 0, -ay) * g;

        // Add force from accelerometer to object.
        GetComponent<Rigidbody>().AddForce(gravity, ForceMode.Acceleration);
    }
}
