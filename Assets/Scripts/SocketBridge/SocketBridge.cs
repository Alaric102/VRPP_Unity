using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System;
using System.Net;
using System.Net.Sockets;
using System.Threading;
using System.Text; 

public class SocketBridge : MonoBehaviour
{
    private TcpListener tcpReceiver;
    private Thread tcpReceiverThread;
    // private ManualResetEvent receiverConnected = null;
    private Socket recvSocket = null;
    private TcpListener tcpSender;
    private Thread tcpSenderThread;
    // private ManualResetEvent senderConnected = null;
    private Socket sendSocket = null;
    private const uint clientBufferSize = 1024;
    private byte[] clientBuffer = new byte[clientBufferSize];
    private CircleBuffer circleBuffer = null;
    enum UnityCommands {
        setStartPoint = 1,
        setGoalPoint,
        startPlanning,
        sendRequestedState
    }
    void Awake()
    {
        circleBuffer = transform.GetComponent<CircleBuffer>();

        tcpReceiver = new TcpListener(IPAddress.Parse("127.0.0.1"), 12345);
        // receiverConnected = new ManualResetEvent(false);
        tcpReceiverThread = new Thread( new ThreadStart(StartReceiver) );

        tcpSender = new TcpListener(IPAddress.Parse("127.0.0.1"), 12344);
        // senderConnected = new ManualResetEvent(false);
        tcpSenderThread = new Thread( new ThreadStart(StartSender) );
        
        tcpReceiverThread.Start();
        tcpSenderThread.Start();
    }

    void Update(){

    }
    void OnApplicationQuit() {
        try {
            tcpReceiverThread.Abort();
            tcpReceiver.Stop();
            recvSocket.Close();
        } catch (NullReferenceException) {
            
        }

        try {
            tcpSenderThread.Abort();
            tcpSender.Stop();
            sendSocket.Close();
        } catch (NullReferenceException) {
            
        }
    }
    private void StartReceiver(){
        Debug.Log("Reseiver is started.");
        while(true){
            tcpReceiver.Start();
            recvSocket = tcpReceiver.AcceptSocketAsync().Result;
            Debug.Log("Reseiver socket connected. Will wait for next connection.");

            recvSocket.BeginReceive(clientBuffer, 0, clientBuffer.Length, 0,  
                new AsyncCallback(ReceiveCallback), circleBuffer);
        }
    }
    private void ReceiveCallback(IAsyncResult ar){
        int recvBytesNum = recvSocket.EndReceive(ar);
        circleBuffer.AddData(clientBuffer, ((uint)recvBytesNum));

        string s = "Received " + recvBytesNum + " bytes: ";
        for (int i = 0; i < recvBytesNum; ++i)
            s += clientBuffer[i].ToString() + " ";
        Debug.Log(s);

        Array.Clear(clientBuffer, 0, clientBuffer.Length);
        recvSocket.BeginReceive(clientBuffer, 0, clientBuffer.Length, 0,  
            new AsyncCallback(ReceiveCallback), circleBuffer);
    }
    private void StartSender(){
        Debug.Log("Sender is started.");
        while(true){
            tcpSender.Start();
            sendSocket = tcpSender.AcceptSocketAsync().Result;
            Debug.Log("Sender socket connected. Will wait for next connection.");
        }
    }
    private void StartSend(ref byte[] data){
        byte[] prefix = {255, 255};
        byte[] dataLen = System.BitConverter.GetBytes(Convert.ToUInt16(data.Length));
        byte[] byteData = CombineBytes(CombineBytes(prefix, dataLen), data);
        
        string s = "Send " + byteData.Length.ToString() + " bytes: ";
        for (int i = 0; i < byteData.Length; ++i)
            s += byteData[i].ToString() + " ";
        Debug.Log(s);

        try {
            sendSocket.BeginSend(byteData, 0, byteData.Length, 0,
                new AsyncCallback(SendCallback), null);
        } catch (NullReferenceException) {
            Debug.Log("Sender error");
            return;
        }
    }
    private void SendCallback(IAsyncResult ar){
        int bytesSent = sendSocket.EndSend(ar);
        string s = "Send " + bytesSent.ToString() + " bytes: ";
    }
    public bool SendStartPoint(Vector3 pos, Vector3 rot){
        byte[] cmdBytes = System.BitConverter.GetBytes(Convert.ToUInt16(UnityCommands.setStartPoint));
        byte[] posBytes = GetBytesVector3(ref pos);
        byte[] rotBytes = GetBytesVector3(ref rot);
        byte[] msg = CombineBytes(CombineBytes(cmdBytes, posBytes), rotBytes);
        StartSend(ref msg);
        return true;
    } // Send start state message
    public bool SendGoalPoint(Vector3 pos, Vector3 rot){
        byte[] cmdBytes = System.BitConverter.GetBytes(Convert.ToUInt16(UnityCommands.setGoalPoint));
        byte[] posBytes = GetBytesVector3(ref pos);
        byte[] rotBytes = GetBytesVector3(ref rot);
        byte[] msg = CombineBytes(CombineBytes(cmdBytes, posBytes), rotBytes);
        StartSend(ref msg);
        return true;
    } // Send goal state message
    public bool RequestPlan(){
        byte[] cmdBytes = System.BitConverter.GetBytes(Convert.ToUInt16(UnityCommands.startPlanning));
        byte[] msg = cmdBytes;
        StartSend(ref msg);
        return true;
    } // Start planning

    public bool SendRequestedState(Vector3 pos, Vector3 rot, bool isFree){
        byte[] cmdBytes = System.BitConverter.GetBytes(Convert.ToUInt16(UnityCommands.sendRequestedState));
        byte[] posBytes = GetBytesVector3(ref pos);
        byte[] rotBytes = GetBytesVector3(ref rot);
        byte[] isFreeBytes = System.BitConverter.GetBytes(Convert.ToUInt16(isFree));
        byte[] msg = CombineBytes(CombineBytes(CombineBytes(cmdBytes, posBytes), rotBytes), isFreeBytes);
        StartSend(ref msg);
        return true;
    }
    private byte[] CombineBytes(byte[] first, byte[] second) {
        byte[] ret = new byte[first.Length + second.Length];
        Buffer.BlockCopy(first, 0, ret, 0, first.Length);
        Buffer.BlockCopy(second, 0, ret, first.Length, second.Length);
        return ret;
    } // combine two byte[] arrays
    private byte[] GetBytesVector3(ref Vector3 v){
        byte[] bytesX = System.BitConverter.GetBytes(v.x);
        byte[] bytesY = System.BitConverter.GetBytes(v.y);
        byte[] bytesZ = System.BitConverter.GetBytes(v.z);
        return CombineBytes(CombineBytes(bytesX, bytesY), bytesZ);
    } // transform Vector3 to bytes[]
}