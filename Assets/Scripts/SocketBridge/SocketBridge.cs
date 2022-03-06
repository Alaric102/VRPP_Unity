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
    private static Thread tcpServer;
    private TcpListener tcpListener;
    private static Socket clientSocket = null;
    private static ManualResetEvent clientConnected = new ManualResetEvent(false);
    public class StateObject{
        public Socket workSocket = null;
        public const int BufferSize = 1024;
        public byte[] buffer = new byte[BufferSize];
        public StringBuilder sb = new StringBuilder();
    }

    private enum UnityTypeEnum
    {
        Vector2Int_t = 1
    }
    void Awake()
    {

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
        // sendData("Hello world.");
        // sendVector2Int(Vector2Int.up * 2);
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

    // Process the client connection.
    private static void AcceptTcpClientCallback(IAsyncResult ar)
    {
        clientConnected.Set();

        TcpListener listener = (TcpListener) ar.AsyncState;
        clientSocket = listener.EndAcceptSocket(ar);
        
        // Create the state object.  
        StateObject state = new StateObject();  
        state.workSocket = clientSocket; 
        clientSocket.BeginReceive( state.buffer, 0, StateObject.BufferSize, 0,  
            new AsyncCallback(ReadCallback), state);  

        Debug.Log("Client connected.");
        tcpServer.Abort();
    } 
    public static void ReadCallback(IAsyncResult ar)
    {
        String content = String.Empty;  
  
        // Retrieve the state object and the handler socket from the asynchronous state object.  
        StateObject state = (StateObject) ar.AsyncState;  
        Socket handler = state.workSocket;  
  
        // Read data from the client socket.
        int bytesRead = handler.EndReceive(ar);  
  
        if (bytesRead > 0) {  
            // There  might be more data, so store the data received so far.  
            state.sb.Append(Encoding.ASCII.GetString(  
                state.buffer, 0, bytesRead));  
  
            // Check for end-of-file tag. If it is not there, read more data.  
            content = state.sb.ToString();  

            // Debug.Log("Received: " + content);

            state.sb.Clear();
            Array.Clear(state.buffer, 0, state.buffer.Length);
            handler.BeginReceive(state.buffer, 0, StateObject.BufferSize, 0,  
                new AsyncCallback(ReadCallback), state);
        }  
    }

    public void sendData(string s){
        if (clientSocket == null){
            return;
        }
        if (!clientSocket.Connected){  
            return;
        }
        byte[] byData = System.Text.Encoding.ASCII.GetBytes(s + '\n');
        clientSocket.Send(byData, 0, byData.Length, SocketFlags.None);
        Debug.Log("Sended: " + s);
    }
    public String getVector2IntMsg(Vector2Int v){
        String str_msg = v.x.ToString() + " " + v.y.ToString();
        return str_msg;
    }

}
