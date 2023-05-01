using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading;
using System.IO.Ports;

public class TestConnction : MonoBehaviour
{ 
    Thread IOThread = new Thread(DataThread);
    private static SerialPort pulseStream;
    private static SerialPort gyroStream;
    private static string incomingPulseMsg = "";
    private static string outgoingPulseMsg = "";
    private static string incomingGyroMsg = "";
    private static string outgoingGyroMsg = "";
    private static string pulseMessage = "";

    public float BPM;
    /*public string receivestring;
    public GameObject test_data;
    public Rigidbody rb;
    public float sensitivity = 0.01f;
    public string[] datas;
    */

    private static void DataThread()
    {
        pulseStream = new SerialPort("COM6", 115200);
        // gyroStream = new SerialPort("COM5", 115200);
        pulseStream.Open();
        // gyroStream.Open();

        while(true)
        {
            if (outgoingPulseMsg != "")
            {
                pulseStream.Write(outgoingPulseMsg);
                outgoingPulseMsg = "";
            }
            incomingPulseMsg = pulseStream.ReadExisting();
            //Debug.Log($"incoming pulse {incomingPulseMsg}");
            if (outgoingGyroMsg != "")
            {
                gyroStream.Write(outgoingGyroMsg);
                outgoingGyroMsg = "";
            }
            // incomingGyroMsg = gyroStream.ReadExisting();
            Thread.Sleep(200);
        }
    }

    void OnDestroy()
    {
        IOThread.Abort();
        pulseStream.Close();
        // gyroStream.Close();
    }

    // Start is called before the first frame update
    void Start()
    {
        IOThread.Start();
        //data_stream.Open(); //Initiate the Serial stream
    }

    // Update is called once per frame
    void Update()
    {
        if (incomingPulseMsg != "")
        {
            //Debug.Log($"Pulse signal: {incomingPulseMsg}");
            pulseMessage = incomingPulseMsg.Substring("<START>".Length, incomingPulseMsg.Length - "<START>".Length - "<END>".Length);
            BPM = float.Parse(pulseMessage);
        }

        if (incomingGyroMsg != "")
        {
            Debug.Log($"Gyro signal: {incomingGyroMsg}");
        }
    }
}
