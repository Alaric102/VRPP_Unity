using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LocalPlanner : MonoBehaviour {
    public VoxelMap voxelMap = null;
    public Transform runnerState = null;
    private List<Transform> localStates = new List<Transform>();
    private List<Vector3> globalPath = new List<Vector3>();
    private Vector3 globalPoseOrigin = Vector3.zero;
    public float dStep = 0.05f;
    private Transform currentState, goalState;
    private bool isPlanning = false;
    private int currentGoalID = 0;
    private GlobalPlanner gPlanner = null;
    private void ClearStates(){
        foreach (var state in localStates)
            Destroy(state.gameObject);
        localStates.Clear();
    }
    private Vector3 GetDelta(Vector3 pose){
        if (currentGoalID == globalPath.Count)
            return Vector3.zero;
        if ((pose - globalPath[currentGoalID]).magnitude < dStep){
            currentGoalID++;
            globalPoseOrigin = globalPath[currentGoalID];
            return GetDelta(pose);
        }
        return globalPath[currentGoalID] - pose;
    }
    private Quaternion GetDeltaAlignedOrigin(Vector3 delta){
        Vector3 deltaFront = delta.normalized;
        Vector3 deltaUp = Vector3.ProjectOnPlane(Vector3.up, delta).normalized;
        Vector3 deltaRight = Vector3.Cross(deltaUp, deltaFront).normalized;  
        
        Quaternion a = Quaternion.FromToRotation(transform.forward, deltaFront);
        Quaternion b = Quaternion.FromToRotation(a * transform.right, deltaRight);
        Quaternion c = Quaternion.FromToRotation(b * a * transform.up, deltaUp);
        return c * b * a;
    }
    private void DrawDirection(Vector3 pos, Quaternion rot){
        Debug.DrawRay(pos, rot * Vector3.forward * 0.15f, Color.blue);
        Debug.DrawRay(pos, rot * Vector3.right * 0.15f, Color.red);
        Debug.DrawRay(pos, rot * Vector3.up * 0.15f, Color.green);
    } 

    void Awake(){
        gPlanner = transform.GetComponent<GlobalPlanner>();
    }
    void Update() {
        if (isPlanning){
            // Calculate Global delta translation and rotation
            currentState = localStates[localStates.Count - 1];
            Robot currentRobot = currentState.GetComponent<Robot>();
            Vector3 deltaPose = GetDelta(currentState.position);
            Quaternion deltaDirection = GetDeltaAlignedOrigin(deltaPose);
            Quaternion deltaRotation = deltaDirection * Quaternion.Inverse(currentState.rotation);
            { // Draw deltaDirection origin
                DrawDirection(currentState.position, deltaDirection);
                Debug.DrawRay(currentState.position, deltaRotation.eulerAngles, Color.white);
                Debug.DrawLine(currentState.position, globalPath[currentGoalID], Color.yellow);
                Debug.Log("deltaPose: " + deltaPose.magnitude + "\n" + 
                    "deltaRotation.eulerAngles: " + deltaRotation.eulerAngles);
            }
            
            // Calculate mean clamped delta of tarnsition and rotation by each leg
            List<Tuple<Vector3, Quaternion>> legMovements = 
                currentRobot.GetLegMovements(deltaPose, deltaRotation);
            Vector3 meanTransition = Vector3.zero;
            Quaternion meanRotation = Quaternion.identity;
            foreach (var item in legMovements) {
                meanTransition = Vector3.Lerp(meanTransition, item.Item1, 0.5f);
                meanRotation = Quaternion.Lerp(meanRotation, item.Item2, 0.5f);
            }
            {
                Debug.DrawRay(currentState.position, meanTransition);
                Debug.DrawRay(currentState.position, meanRotation.eulerAngles);
            }

            // Propagate to next state
            Transform nextState = Instantiate(runnerState, currentState.position + meanTransition, 
                meanRotation * currentState.rotation,  transform);
            
            Robot nextRobot = nextState.GetComponent<Robot>();

            // Check propagated state
            if (!nextRobot.PropagateLegMovements()){
                Vector3 deviation = Vector3.ClampMagnitude(nextRobot.GetDeviation(), dStep);
                    
                globalPath[currentGoalID] += deviation;
                if (voxelMap.GetDiscreteState(globalPoseOrigin) != voxelMap.GetDiscreteState(globalPath[currentGoalID])){
                    Debug.Log("Inform gPlanner about new weight.");
                    
                    voxelMap.SetWeight(voxelMap.GetDiscreteState(globalPoseOrigin), 1.0f);
                    globalPath = RequestPathAfter(currentGoalID);
                    globalPoseOrigin = globalPath[currentGoalID];
                    gPlanner.ShowGlobalPlan(globalPath);
                }
                Destroy(nextState.gameObject);
            } else {
                nextState.gameObject.name = localStates.Count.ToString();

                localStates.Add(nextState);
                localStates[localStates.Count - 2].gameObject.SetActive(false);
            }
            
        }
    }
    private List<Vector3> RequestPathAfter(int pathID) {
        List<Vector3> newAfterPath = gPlanner.GetGlobalPlan(globalPath[pathID - 1], goalState.position);
        List<Vector3> newPath = new List<Vector3>();
        for (int id = 0; id < pathID; ++id)
            newPath.Add(globalPath[id]);
        for (int id = 0; id < newAfterPath.Count; ++id)
            newPath.Add(newAfterPath[id]);
        return newPath;
    }
    public void GetPath(Transform start, Transform goal){
        ClearStates();
        goalState = goal;
        globalPath = gPlanner.GetGlobalPlan(start.position, goal.position);
        
        gPlanner.ShowGlobalPlan(globalPath);
                
        currentGoalID = 0;
        currentState = Instantiate(runnerState, start.position, start.rotation, transform);
        currentState.gameObject.name = "local " + localStates.Count.ToString();
        localStates.Add(currentState);
        isPlanning = true;
    }
}
