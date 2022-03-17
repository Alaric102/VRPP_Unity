using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Navigation : MonoBehaviour
{
    public SocketBridge socketBridge = null;
    private Transform startState;
    private Transform goalState;
    private Vector3 startRotation, goalRotation; // Required to store angles in range [-180, 180)
    private LineRenderer globalPlanLine = null;
    private List<Vector3> globalPath = new List<Vector3>();
    void Awake()
    {
        startState = transform.GetChild(0);
        goalState = transform.GetChild(1);

        globalPlanLine = transform.GetComponent<LineRenderer>();
    }
    void Start()
    {
        globalPlanLine.positionCount = 0;
    }

    // Update is called once per frame
    void Update()
    {
        globalPlanLine.positionCount = globalPath.Count + 1;
        globalPlanLine.SetPosition(0, startState.position);
        for (int i = 0; i < globalPath.Count; ++i){
            globalPlanLine.SetPosition(i+1, globalPath[i]);
            // Debug.Log(path[i]);
        }
    }
    
    public void SetStartState(ref Vector3 v, ref Vector3 r){
        startState.position = v;
        startState.rotation = Quaternion.Euler(r);
        startRotation = r;
    } // Set start State from Navigation Menu
    public void SetGoalState(ref Vector3 v, ref Vector3 r){ 
        goalState.position = v;
        goalState.rotation = Quaternion.Euler(r);
        goalRotation = r;
    } // Set goal State from Navigation Menu

    public void StartPlanning(){
        Quaternion q = Quaternion.Euler(startRotation);
        socketBridge.SendStartPoint(startState.position, startRotation);
        // Debug.Log("Start state: pos: " + startState.position + 
        //     ", rot: " + startRotation + " (" + 
        //     q.x + ", " + 
        //     q.y + ", " + 
        //     q.z + ", " + 
        //     q.w + ", " + 
        //     ")");

        q = Quaternion.Euler(goalRotation);
        socketBridge.SendGoalPoint(goalState.position, goalRotation);
        // Debug.Log("Goal state: pos: " + goalState.position + 
        //     ", rot: " + goalRotation + " (" + 
        //     q.x + ", " + 
        //     q.y + ", " + 
        //     q.z + ", " + 
        //     q.w + ", " + 
        //     ")");

        socketBridge.RequestPlan();
    }

    public void SetGlobalPlan(List<Vector3> path){
        globalPath = path;
        // Debug.Log(path.Count);
        // // globalPlanLine.positionCount = path.Count + 1;
        // // Debug.Log(globalPlanLine.positionCount);
        // // 
        // // Debug.Log(globalPlanLine.positionCount);
        
    }
}
