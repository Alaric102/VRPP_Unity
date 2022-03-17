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
    [Header("External Modules")]
    public Navigation navigation = null;
    public Mapper mapper = null;
    private static Thread tcpServer;
    private TcpListener tcpListener;
    private static Socket clientSocket = null;
    private static ManualResetEvent clientConnected = new ManualResetEvent(false);
    public class StateObject{
        public Socket workSocket = null;
        public const int BufferSize = 1024;
        public byte[] buffer = new byte[BufferSize];
        public Navigation navigation = null;
    }
    enum UnityCommands {
        setStartPoint = 1,
        setGoalPoint,
        startPlanning
    }
    void Start()
    {
        tcpServer = new Thread( new ThreadStart(ListenForIncommingRequests) );
        tcpServer.IsBackground = true;
        tcpServer.Start();
        
    }

    // Update is called once per frame
    void Update()
    {
    }

    void OnDestroy(){
        tcpListener.Stop();
        clientSocket.Close();
    }

    private void ListenForIncommingRequests (){
			tcpListener = new TcpListener(IPAddress.Parse("127.0.0.1"), 12345);
			tcpListener.Start();

            while (true){
                clientConnected.Reset();
                if ((clientSocket == null) || (!clientSocket.Connected)){
                    Debug.Log("Server is listening");
                    tcpListener.BeginAcceptSocket(new AsyncCallback(AcceptTcpClientCallback), tcpListener);
                    clientConnected.WaitOne();
                }
                Debug.Log("Server is finished");
            }
    }
    private void AcceptTcpClientCallback(IAsyncResult ar)
    {
        clientConnected.Set();

        TcpListener listener = (TcpListener) ar.AsyncState;
        clientSocket = listener.EndAcceptSocket(ar);
        
        // Create the state object.  
        StateObject state = new StateObject();  
        state.workSocket = clientSocket;
        state.navigation = navigation;
        clientSocket.BeginReceive( state.buffer, 0, StateObject.BufferSize, 0,  
            new AsyncCallback(ReadCallback), state);  

        Debug.Log("Client connected.");
        tcpServer.Abort();
    } 
    public static void ReadCallback(IAsyncResult ar)
    {  
        // Retrieve the state object and the handler socket from the asynchronous state object.  
        StateObject state = (StateObject) ar.AsyncState;  
        Socket handler = state.workSocket;  
  
        // Read data from the client socket.
        int bytesRead = handler.EndReceive(ar);  
        // Debug.Log("bytesRead: " + bytesRead);
        
        if (bytesRead > 1) {
            string s = "";
            for (int i = 0; i < bytesRead; ++i){
                s += state.buffer[i].ToString() + " ";
            }
            Debug.Log(s);
            var bytes = state.buffer;
            
            int cmd = ParseCommand(ref bytes, bytesRead);
            switch (cmd){
            case (1):
                int offset = 3;
                List<Vector3> path = new List<Vector3>();
                while (offset < bytesRead){
                    float x = BitConverter.ToSingle(bytes, offset);
                    offset += 4;

                    float y = BitConverter.ToSingle(bytes, offset);
                    offset += 4;

                    float z = BitConverter.ToSingle(bytes, offset);
                    offset += 4;

                    path.Add(new Vector3(x, y, z));
                    Debug.Log(new Vector3(x, y, z));
                }
                Debug.Log(state.navigation == null);
                state.navigation.SetGlobalPlan(path);
                break;
            default:
                break;
            }
        }  
        Array.Clear(state.buffer, 0, state.buffer.Length);
        handler.BeginReceive(state.buffer, 0, StateObject.BufferSize, 0,  
            new AsyncCallback(ReadCallback), state);
    }

    public bool SendStartPoint(Vector3 pos, Vector3 rot){
        byte[] cmdBytes = System.BitConverter.GetBytes(Convert.ToUInt16(UnityCommands.setStartPoint));
        byte[] posBytes = GetBytesVector3(ref pos);
        byte[] rotBytes = GetBytesVector3(ref rot);
        byte[] msg = CombineBytes(CombineBytes(cmdBytes, posBytes), rotBytes);
        return sendBytes(ref msg);
    } // Send start state message
    public bool SendGoalPoint(Vector3 pos, Vector3 rot){
        byte[] cmdBytes = System.BitConverter.GetBytes(Convert.ToUInt16(UnityCommands.setGoalPoint));
        byte[] posBytes = GetBytesVector3(ref pos);
        byte[] rotBytes = GetBytesVector3(ref rot);
        byte[] msg = CombineBytes(CombineBytes(cmdBytes, posBytes), rotBytes);
        return sendBytes(ref msg);        
    } // Send goal state message
    public bool RequestPlan(){
        byte[] cmdBytes = System.BitConverter.GetBytes(Convert.ToUInt16(UnityCommands.startPlanning));
        byte[] msg = cmdBytes;
        return sendBytes(ref msg);         
    } // Start planning
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
    private bool sendBytes(ref byte[] data){
        if (clientSocket == null){
            return false;
        }
        if (!clientSocket.Connected){  
            return false;
        }
        byte[] prefix = {255, 255};
        byte[] dataLen = System.BitConverter.GetBytes(Convert.ToUInt16(data.Length));
        byte[] byteData = CombineBytes(CombineBytes(prefix, dataLen), data);
        return (byteData.Length == clientSocket.Send(byteData, 0, byteData.Length, SocketFlags.None));
    }

    private static int ParseCommand(ref byte[] bytes, int bytesRead){
        if (!(bytes[0] == 0xFF && bytes[1] == 0xFF)){
            Debug.Log("no msg Beggin");
            return -1;
        }

        int cmd = bytes[2];
        return cmd;
    }
}