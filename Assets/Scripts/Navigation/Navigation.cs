using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Navigation : MonoBehaviour {
    private GlobalPlanner gPlanner;
    private LocalPlanner lPlanner;
    List<Vector3> globalPlan = new List<Vector3>();
    private Transform startState, goalState;
    private Vector3 startRotation, goalRotation; // Required to store angles in range [-180, 180)
    void Awake() {
        // find child objects of Navigation
        startState = transform.GetChild(0);
        goalState = transform.GetChild(1);
        
        gPlanner = transform.GetComponent<GlobalPlanner>();
        lPlanner = transform.GetComponent<LocalPlanner>();
    }
    void Start() {}
    void Update() {

    }
    public Transform GetStartState(){ // Get activated star state transform
        startState.gameObject.SetActive(true);
        return startState;
    }
    public Transform GetGoalState(){ // Get activated star state transform
        goalState.gameObject.SetActive(true);
        return goalState;
    }
    public void StartPlanning(){
        List<Vector3Int> plan = gPlanner.GetGlobalPlan(startState.position, goalState.position);
        globalPlan = gPlanner.ConvertPlanToCont(plan);
        gPlanner.ShowPlan(globalPlan);
        lPlanner.SetGlobalPath(globalPlan, startState, goalState);
    }
    public void Replan(Vector3 newGoal, int startID){
        gPlanner.SetWieght(gPlanner.GetDescrete(globalPlan[startID]), 1.0f);

        List<Vector3> newPath = new List<Vector3>();
        for (int id = 0; id < startID; ++id){
            newPath.Add(globalPlan[id]);
        }
        
        List<Vector3> planToNewGoal = gPlanner.ConvertPlanToCont(gPlanner.GetGlobalPlan(globalPlan[startID], newGoal));
        for (int id = 0; id < planToNewGoal.Count; ++id){
            newPath.Add(planToNewGoal[id]);
        }
        
        List<Vector3> planFromNewGoal = gPlanner.ConvertPlanToCont(
            gPlanner.GetGlobalPlan(planToNewGoal[planToNewGoal.Count - 1], goalState.position));
        for (int id = 0; id < planFromNewGoal.Count; ++id){
            newPath.Add(planFromNewGoal[id]);
        }

        globalPlan = newPath;
        gPlanner.ShowPlan(globalPlan);
        // lPlanner.SetGlobalPath(globalPlan);
    }
    public List<Vector3> GetGlobalPlan(){
        return globalPlan;
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
