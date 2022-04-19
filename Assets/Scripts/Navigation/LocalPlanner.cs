using System;
using System.Globalization;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LocalPlanner : MonoBehaviour {
    public VoxelMap voxelMap = null;
    public Transform robotPrefab = null;
    private GlobalPlanner gPlanner = null;
    public float dStep = 0.05f;
    private Dictionary<Vector3, List<Transform>> goalsToStates = new Dictionary<Vector3, List<Transform>>();
    private Queue<Transform> toAcheiveState = new Queue<Transform>();
    private List<Transform> reachedStates = new List<Transform>();
    private Transform currentState = null, goalState = null, globalStatePrefab;
    private bool isPlanning = false;
    private DateTime computingTime;
    private Vector3 deviation = Vector3.zero;
    private void ClearStates(){ // Clear all local planner states, achieved states and states to acheive
        ClearStatesToAchieve();
        ClearAcheivedStates();
        ClearLocalStates();
    }
    private void ClearLocalStates(){ // Clear all local planner states
        foreach (var goalToState in goalsToStates)
            foreach (var state in goalToState.Value)
                Destroy(state.gameObject);
        goalsToStates.Clear();
    }
    private void ClearAcheivedStates(){ // Clear all acheived states
        foreach (var item in reachedStates)
            Destroy(item.gameObject);
        reachedStates.Clear();
    }
    private void ClearStatesToAchieve(){ // Clear all state to acheive
        while (toAcheiveState.Count > 0)
            Destroy(toAcheiveState.Dequeue().gameObject);
    }
    private void AppendStateToAchieve(Vector3 pose){ // append pose to ToAcheive Queue
        Transform newState = Instantiate(globalStatePrefab, pose, Quaternion.identity, transform);
        toAcheiveState.Enqueue(newState);
        
        newState.gameObject.name = voxelMap.GetDiscreteState(pose).ToString() + 
            " toAcheive " + toAcheiveState.Count.ToString();
        newState.gameObject.SetActive(true);
    }
    private void AppendNextStateAchived(){ // Pop() first from ToAcheive and Add to Acheived
        var achieved = toAcheiveState.Dequeue();
        achieved.GetComponent<Renderer>().material.color = Color.green;
        reachedStates.Add(achieved);
        
        // Debug.Log("Acheived: " + achieved.gameObject.name);
        achieved.gameObject.name = voxelMap.GetDiscreteState(achieved.position).ToString() + 
            " achived " + reachedStates.Count.ToString();
    }
    private void GetNewGlobalPlan(Transform start, Transform goal){ // Clear previous states to achieve and append new
        ClearStatesToAchieve();
        var plan = gPlanner.GetGlobalPlan(start.position, goal.position);
        foreach (var pose in plan) {
            AppendStateToAchieve(pose);
        }
    }
    private Vector3 GetRelevantGoal(Vector3 pose){ // Return closest goal and update Acheived and ToAcheive containers
        if (toAcheiveState.Count == 0)
            return pose;
        // If we are close to goal move ToAcheive.Peek() to Acheived and repeat
        // Otherwise, return ToAcheive.Peek()
        if ((pose - toAcheiveState.Peek().position).magnitude < dStep){
            AppendNextStateAchived();
            return GetRelevantGoal(pose);
        } else {
            return toAcheiveState.Peek().position;
        }
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
    private Vector3 GetLastReachedGoal(int shift = 0){
        if (reachedStates.Count < (1 + shift)){
            Debug.Log("reachedStates.count" + reachedStates.Count + " < " + (1 + shift));
            // var lastAvailableStates = goalsToStates[toAcheiveState.Peek().position];
            return toAcheiveState.Peek().position;
        }
        return reachedStates[reachedStates.Count - (1 + shift)].position;
    }
    private void ForgetBranchToGoal(Vector3 pose){
        if (goalsToStates.ContainsKey(pose)){
            foreach (var item in goalsToStates[pose]) {
                Destroy(item.gameObject);
            }
            goalsToStates.Remove(pose);
        }
    }
    private void ForgetReachedAfter(Vector3 v){
        for (int i = reachedStates.Count - 1; i >= 0; --i) {
            if (reachedStates[i].position == v)
                return;
            else {
                Destroy(reachedStates[i].gameObject);
                ForgetBranchToGoal(reachedStates[i].position);
                reachedStates.RemoveAt(i);
            }
        }
    }
    void Awake(){
        gPlanner = transform.GetComponent<GlobalPlanner>();
        globalStatePrefab = transform.GetChild(2);
    }
    void Update() {
        if (isPlanning){
            // float timeStart = DateTime.Now.Ticks / 
            Vector3 actualGoal = GetRelevantGoal(currentState.position);
            if (actualGoal == currentState.position){
                isPlanning = false;
                Debug.Log("Local Planner is Finished");
                return;
            }

            Vector3 correctedGoal = actualGoal + deviation;
            Vector3 deltaPose = correctedGoal - currentState.position;
            Quaternion deltaDirection = GetDeltaAlignedOrigin(deltaPose);
            Quaternion deltaRotation = deltaDirection * Quaternion.Inverse(currentState.rotation);
            { // Draw deltaDirection origin
                DrawDirection(currentState.position, deltaDirection);
                Debug.DrawRay(currentState.position, deltaRotation.eulerAngles, Color.white);
                Debug.DrawLine(currentState.position, correctedGoal, Color.yellow);
                // Debug.Log("deltaPose: " + deltaPose.magnitude + "\n" + 
                //     "deltaRotation.eulerAngles: " + deltaRotation.eulerAngles);
            }
            
            // Calculate mean clamped delta of tarnsition and rotation by each leg
            Robot currentRobot = currentState.GetComponent<Robot>();
            List<Tuple<Vector3, Quaternion>> legMovements = 
                currentRobot.GetHeapMovements(deltaPose, deltaRotation, dStep);
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
            Transform nextState = Instantiate(robotPrefab, currentState.position + meanTransition, 
                meanRotation * currentState.rotation,  transform);
            Robot nextRobot = nextState.GetComponent<Robot>();

            // Check propagated state
            Vector3 newDeviation = Vector3.ClampMagnitude(nextRobot.IsPropagatableMovement(), dStep);
            if (newDeviation != Vector3.zero){
                deviation += newDeviation;
                
                { //Strategy for small deviation
                    // voxelMap.SetWeight(voxelMap.GetDiscreteState(nextState.position), 1.0f);
                    Vector3 lastReachedPose = GetLastReachedGoal(0);
                    Vector3Int action = voxelMap.GetDiscreteState(actualGoal) - voxelMap.GetDiscreteState(lastReachedPose);
                    gPlanner.SetActionCost(action, voxelMap.GetDiscreteState(lastReachedPose), 0.5f);
                }
                
                {
                    Debug.Log("Impossible to propagate." +
                        " NewDeviation: " + newDeviation.magnitude + ", total deviation: " + deviation.magnitude);
                    Debug.DrawRay(nextState.position, deviation, Color.black);
                }

                if (voxelMap.GetDiscreteState(actualGoal) != voxelMap.GetDiscreteState(correctedGoal)){   
                    // Set weight to bad actualGoal and descrete nextState
                    {   // Strategy for large deviations
                        voxelMap.SetWeight(voxelMap.GetDiscreteState(actualGoal), 1.0f);
                    }

                    Vector3 lastReachedPose = GetLastReachedGoal(1);
                    ForgetReachedAfter(lastReachedPose);
                    ForgetBranchToGoal(actualGoal);
                    currentState = goalsToStates[lastReachedPose][goalsToStates[lastReachedPose].Count - 1];
                    currentState.gameObject.SetActive(true);
                    GetNewGlobalPlan(currentState, goalState);

                    Debug.Log("Inform gPlanner about large displacement.\n" + 
                        voxelMap.GetDiscreteState(actualGoal) + " -> " + 
                        voxelMap.GetDiscreteState(correctedGoal) );

                    // Reset deviation
                    deviation = Vector3.zero;
                }

                // nextState is invalid for further propagation
                Destroy(nextState.gameObject);
            } else {
                // Append next state to actualGoal
                if (goalsToStates.ContainsKey(actualGoal))
                    goalsToStates[actualGoal].Add(nextState);
                else
                    goalsToStates.Add(actualGoal, new List<Transform>{nextState});
                
                nextState.gameObject.name = voxelMap.GetDiscreteState(actualGoal).ToString() + 
                    " local " + (goalsToStates[actualGoal].Count - 1).ToString();
                currentState.gameObject.SetActive(false);

                // reset deviation for next propagation
                deviation = Vector3.zero;
                currentState = nextState;
            }
        }
    }
    public void GetPath(Transform start, Transform goal){
        ClearStates();
        goalState = goal;
        GetNewGlobalPlan(start, goal);
        
        // Init currentState
        if (currentState != null)
            Destroy(currentState.gameObject);
        currentState = Instantiate(robotPrefab, start.position, start.rotation, transform);
        currentState.gameObject.SetActive(true);
        
        // Append initial currentState to first relevant goal
        Vector3 actualGoal = GetRelevantGoal(currentState.position);
        goalsToStates.Add(actualGoal, new List<Transform>{currentState});
        currentState.gameObject.name = voxelMap.GetDiscreteState(actualGoal).ToString() + 
                    " local " + (goalsToStates[actualGoal].Count - 1).ToString();
        
        isPlanning = true;
    }
}
