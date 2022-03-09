using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Navigation : MonoBehaviour
{
    public SocketBridge socketBridge = null;
    private Transform startState;
    private Transform goalState;

    void Awake()
    {
        startState = transform.GetChild(0);
        goalState = transform.GetChild(1);
    }
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    
    public void SetStartState(ref Vector3 v, ref Quaternion q){
        startState.position = v;
        startState.rotation = q;
    } // Set start State from Navigation Menu
    public void SetGoalState(ref Vector3 v, ref Quaternion q){ 
        goalState.position = v;
        goalState.rotation = q;
    } // Set goal State from Navigation Menu

    public void StartPlanning(){
        socketBridge.SendStartPoint(startState.position, startState.rotation);
        socketBridge.SendGoalPoint(goalState.position, goalState.rotation);
        socketBridge.RequestPlan();
    }
}
