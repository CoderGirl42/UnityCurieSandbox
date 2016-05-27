using UnityEngine;
using System.IO.Ports;
using System.Threading;

public class SerialPotentiometer : MonoBehaviour {

    private SerialPort serial;
    private float rotation;
    public bool invert = false;
    private int go;
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

    void OnDestroy()
    {
        // Cleanup Serial connection.
        if (serial.IsOpen)
        {
            serial.Close();
        }
    }

    public void Update()
    {
        var msg = serial.ReadLine();

        if (msg != string.Empty)
        {
            string[] vars = msg.Split(',');
            rotation = float.Parse(vars[0]);

            if(invert)
            {
                rotation *= -1;
            }

            Quaternion fromRotation = transform.rotation;
            Quaternion toRotation = Quaternion.Euler(0, rotation, 0);
            transform.rotation = Quaternion.Lerp(fromRotation, toRotation, Time.deltaTime * 2);

            go = int.Parse(vars[1]);

            if (go == 1)
            {
                transform.Translate(Vector3.forward * Time.deltaTime * 10);
            }
        }
    }

    public void OnGUI()
    {
        GUI.Label(new Rect(10, 10, 100, 20), go == 1 ? "LED ON" : "LED OFF");
    }
}
