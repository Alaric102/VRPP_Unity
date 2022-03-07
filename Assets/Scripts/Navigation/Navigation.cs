using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Navigation : MonoBehaviour
{
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

    public void SetStartState(Vector3 v, Quaternion q){
        startState.position = v;
        startState.rotation = q;
    }
    public void SetGoalState(Vector3 v, Quaternion q){ 
        goalState.position = v;
        goalState.rotation = q;    
    }

    public void StartPlanning(){

    }
}
