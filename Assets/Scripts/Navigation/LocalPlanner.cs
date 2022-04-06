using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LocalPlanner : MonoBehaviour
{
    public Transform runnerState = null;
    private List<Transform> globalStates = new List<Transform>();
    private List<Vector3> globalPath = new List<Vector3>();
    public float maxStepLength = 0.16f;
    private Vector3 currentState = Vector3.zero;
    private void ClearStates(){
        foreach (var state in globalStates)
            Destroy(state.gameObject);
        globalStates.Clear();
    }    
    private void DhowState(int id){
        foreach (var state in globalStates)
            state.gameObject.SetActive(false);
        if (globalStates.Count > id && globalStates.Count > 0 && id >= 0){
            globalStates[id].gameObject.SetActive(true);
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
    public void SetGlobalPath(List<Vector3> path){
        ClearStates();
        globalPath = path;
        
        for (int i = globalPath.Count - 2; i >= 0; i--) {
            Transform newState = Instantiate(runnerState, globalPath[i], GetDeltaAlignedOrigin(globalPath[i + 1] - globalPath[i]), transform);
            globalStates.Add(newState);

            Robot newRobot = newState.GetComponent<Robot>();
            Vector3 horizonShift = newRobot.GetDisplacementByLeg();
            Debug.DrawRay(globalPath[i], horizonShift, Color.gray);

            newState.position += horizonShift;
            globalPath[i] += horizonShift;
            newState.rotation = GetDeltaAlignedOrigin(globalPath[i + 1] - globalPath[i]);
            
            Vector3 vertBodyShift = newRobot.GetBodyHeightByLeg();
            newState.position += vertBodyShift;
            globalPath[i] += vertBodyShift;
            newState.rotation = GetDeltaAlignedOrigin(globalPath[i + 1] - globalPath[i]);

            newRobot.PlaceFoot();
        }

    }
    private Vector3 GetDelta(Vector3 pose){
        if (globalPath.Count == 0)
            return Vector3.zero;
        if ((globalPath[0] - pose).magnitude < maxStepLength){
            globalPath.RemoveAt(0);
            return GetDelta(pose);
        } else {
            return globalPath[0] - pose;
        }
    }
    void Awake() {
        
    }
    void Start() {
        
    }
    void Update() {
        // GetLocalPlan();
    }
    private void GetLocalPlan(){
        Vector3 currentGoalDelta = GetDelta(currentState);

        Debug.Log("curState: " + currentState + "\n" + 
            "curGoalDelta: " + currentGoalDelta);
        Debug.DrawRay(currentState, currentGoalDelta, Color.red);
    }
}
