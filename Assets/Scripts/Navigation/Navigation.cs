using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Navigation : MonoBehaviour {
    private GlobalPlanner gPlanner;
    private LocalPlanner lPlanner;
    private List<Vector3> globalPlan = new List<Vector3>();
    private Queue<Tuple<Vector3, Quaternion>> goalStates = new Queue<Tuple<Vector3, Quaternion>>();
    private Transform startState, goalState;
    private Vector3 startRotation, goalRotation; // Required to store angles in range [-180, 180)
    void Awake() {
        gPlanner = transform.GetComponent<GlobalPlanner>();
        lPlanner = transform.GetComponent<LocalPlanner>();

        // find child objects of Navigation
        startState = transform.GetChild(0);
        goalState = transform.GetChild(1);
    }

    void Start(){
    }
    public Transform GetStartState(){ // Get activated star state transform
        startState.gameObject.SetActive(true);
        return startState;
    }
    public void SetStartState(Transform state){
        startState.position = state.position;
        startState.rotation = state.rotation;
    }
    public Transform GetGoalState(){ // Get activated star state transform
        goalState.gameObject.SetActive(true);
        return goalState;
    }
    public void AddGoalState(){
        goalStates.Enqueue(new Tuple<Vector3, Quaternion>(goalState.position, goalState.rotation));
    }
    public bool StartPlanning(){
        lPlanner.StartPlanning(startState, goalState);
        return true;
    }
    public void Replan(Vector3 newPose, Vector3 oldPose){
        lPlanner.Replan(newPose, oldPose);
    }
    public List<Vector3> GetGlobalPlan(){
        return lPlanner.GetGlobalPlan();
    }
    public void StopPlanning(){
        lPlanner.StopPlanning();
    }
    public void SetActiveStates(bool isActive){
        startState.gameObject.SetActive(isActive);
        goalState.gameObject.SetActive(isActive);
    }
    private Vector3 wrapAngle(Vector3 r){
        Vector3 res = new Vector3(-180.0f, -180.0f, -180.0f);
        res += r;
        res.x += (res.x > 0.0f) ? -180.0f : +180.0f;
        res.y += (res.y > 0.0f) ? -180.0f : +180.0f;
        res.z += (res.z > 0.0f) ? -180.0f : +180.0f;
        return res;
    }
}
