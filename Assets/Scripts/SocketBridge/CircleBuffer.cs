using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using System.Threading;
using System;
public class CircleBuffer : MonoBehaviour
{
    [Header("External Modules")]
    public Navigation navigation = null;
    public Mapper mapper = null;
    private const uint _size = 1024;
    private uint _begin = 0;
    private uint _end = 0;
    private byte[] _buffer = new byte[_size];
    private bool _isFull = false;
    private Thread proccessDataThread;
    enum ROSCommands {
        globalPath = 1,
        requestedPose
    }
    void Awake() {
        Reset();
        proccessDataThread = new Thread( new ThreadStart(ProccessData) );
        proccessDataThread.Start();
    }
    void OnApplicationQuit() {
        proccessDataThread.Abort();
    }
    public void Reset(){
        _begin = 0;
        _end = 0;
        _isFull = false;
    }
    public bool IsEmpty(){
        return (!_isFull && (_begin == _end));
    }
    public bool IsFull(){
        return _isFull;
    }
    public uint GetCapacity() {
        return _size;
    }
    public uint GetSize(){
        if(_isFull)
            return _size;
        if (_begin >= _end)
            return _begin -_end;
        else
            return _size + _begin -_end;
    }
    public void AddItem(byte data){
        _buffer[_begin] = data;
        if (_isFull){
            _end = (_end + 1) % _size;
        }
        _begin = (_begin + 1) % _size;
        _isFull = _begin == _end;
    }
    public void AddData(byte[] data, uint len){
        for (uint i = 0; i < len; ++i)
            AddItem(data[i]);
    }
    public byte GetItem(){
        byte res = _buffer[_end];
        _isFull = false;
        _end = (_end + 1) % _size;
        return res;
    }
    public byte[] GetData(uint len){
        byte[] res = new byte[len];
        for (uint i = 0; i < len; ++i)
            res[i] = GetItem();
        return res;
    }
    public byte PredictNext(){
        return _buffer[_end];
    }
    public void PrintBuffer(){
        string s = "Buffer: ";
        for (uint i = 0; i < _size; ++i){
            s += _buffer[i].ToString() + " ";
        }
        Debug.Log(s);
    }
    private int GetCommand(){
        if (IsEmpty()){
            return -1;
        }
        byte fhead = GetItem();
        byte shead = PredictNext();
        if (fhead != 0xFF || shead != 0xFF){
            return -1;
        }
        GetItem();
        return GetItem();
    }
    private float GetFloat(){
        byte[] fBytes = GetData(4);
        return BitConverter.ToSingle(fBytes, 0);
    }
    private void ProccessData(){
        while (true){
            int cmd = GetCommand();
            switch (cmd){
                case (((int)ROSCommands.globalPath)):{
                    int len = GetItem();
                    List<Vector3> path = new List<Vector3>();
                    for (int i = 0; i < len; ++i){
                        path.Add(new Vector3(GetFloat(), GetFloat(), GetFloat()));
                        // navigation.SetGlobalPlan(path);
                    }
                    break;
                } 
                case (((int)ROSCommands.requestedPose)):{
                    Vector3 v = new Vector3(GetFloat(), GetFloat(), GetFloat());
                    // navigation.CheckStateCollision(v);
                    break;
                } default: {
                    break;
                }
            }

        }
    }
}
