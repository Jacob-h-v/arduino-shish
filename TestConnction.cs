using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Threading;
using System.IO.Ports;

public class TestConnction : MonoBehaviour
{ 
    Thread IOThread = new Thread(DataThread);
    private static SerialPort pulseStream;
    private static SerialPort gyroStream
    private static string incomingPulseMsg = "";
    private static string outgoingPulseMsg = "";
    private static string incomingGyroMsg = "";
    private static string outgoingGyroMsg = "";

    private static void DataThread()
    {
        pulseStream = new SerialPort("COM3", 115200);
        gyroStream = new SerialPort("COM5", 115200);
        pulseStream.Open();
        gyroStream.Open();

        while(true)
        {
            if (outgoingPulseMsg != "")
            {
                pulseStream.Write(outgoingPulseMsg);
                outgoingPulseMsg = "";
            }
            incomingPulseMsg = pulseStream.ReadExisting();
            if (outgoingGyroMsg != "")
            {
                gyroStream.Write(outgoingGyroMsg);
                outgoingGyroMsg = "";
            }
            incomingGyroMsg = gyroStream.ReadExisting();
            Thread.Sleep(200);
        }
    }

    private void OnDestroy()
    {
        IOThread.Abort();
        pulseStream.Close();
        gyroStream.Close();
    }

    // Start is called before the first frame update
    void Start()
    {
        IOThread.Start();
    }

    // Update is called once per frame
    void Update()
    {
        if (incomingPulseMsg != "")
        {
            Debug.Log($"Pulse signal: {incomingPulseMsg}");
        }

        if (incomingGyroMsg != "")
        {
            Debug.Log($"Gyro signal: {incomingGyroMsg}");
        }

    }
}
