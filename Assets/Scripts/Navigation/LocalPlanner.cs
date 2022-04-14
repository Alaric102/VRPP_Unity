using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LocalPlanner : MonoBehaviour
{
    public Transform runnerState = null;
    private List<Transform> localStates = new List<Transform>();
    private List<Vector3> globalPath = new List<Vector3>();
    public float dStep = 0.05f;
    private Transform currentState, goalState;
    private GlobalPlanner gPlanner = null;
    private bool isPlanning = false;
    private void ClearStates(){
        foreach (var state in localStates)
            Destroy(state.gameObject);
        localStates.Clear();
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
    private Vector3 GetDelta(Vector3 pose){
        if (globalPath.Count == 0)
            return Vector3.zero;
        return globalPath[0] - pose;
    }
    private void DrawDirection(Vector3 pos, Quaternion rot){
        Debug.DrawRay(pos, rot * Vector3.forward * 0.15f, Color.blue);
        Debug.DrawRay(pos, rot * Vector3.right * 0.15f, Color.red);
        Debug.DrawRay(pos, rot * Vector3.up * 0.15f, Color.green);
    }    void Update() {
        if (isPlanning){
            // Calculate Global delta translation and rotation
            if ((currentState.position - globalPath[0]).magnitude < dStep){
                globalPath.RemoveAt(0);
            }
            Vector3 deltaPose = GetDelta(currentState.position);
            Quaternion deltaDirection = GetDeltaAlignedOrigin(deltaPose);
            Quaternion deltaRotation = deltaDirection * Quaternion.Inverse(currentState.rotation);
            { // Draw deltaDirection origin
                DrawDirection(currentState.position, deltaDirection);
                Debug.DrawRay(currentState.position, deltaRotation.eulerAngles);
                Debug.Log("deltaPose: " + deltaPose + "\ndeltaRotation.eulerAngles: " + deltaRotation.eulerAngles);
            }

            Robot currentRobot = currentState.GetComponent<Robot>();
            
            // Calculate mean clamped delta of tarnsition and rotation by each leg
            List<Tuple<Vector3, Quaternion>> legMovements = currentRobot.GetLegMovements(deltaPose, deltaRotation);
            Vector3 meanTransition = Vector3.zero;
            Quaternion meanRotation = Quaternion.identity;
            foreach (var item in legMovements) {
                meanTransition = Vector3.Lerp(meanTransition, item.Item1, 0.5f);
                meanRotation = Quaternion.Lerp(meanRotation, item.Item2, 0.5f);
            }

            // Propagate to next state
            Transform nextState = Instantiate(runnerState, currentState.position + meanTransition, 
                meanRotation * currentState.rotation,  transform);
            localStates.Add(nextState);
            nextState.gameObject.name = "next state: " + localStates.Count.ToString();
            Robot nextRobot = nextState.GetComponent<Robot>();

            // Check propagated state
            if (!nextRobot.PropagateLegMovements()){
                Debug.Log("Invalid state.");
            } else {
                
            }

            currentState = nextState;
            if (localStates.Count > 2){
                localStates[0].gameObject.SetActive(false);
                localStates.RemoveAt(0);
            }   
        }
    }
    public void SetGlobalPath(List<Vector3> path, Transform start, Transform goal){
        // gPlanner = transform.GetComponent<GlobalPlanner>();
        ClearStates();
        isPlanning = true;
        globalPath = path;

        currentState = Instantiate(runnerState, start.position, start.rotation, transform);
        localStates.Add(currentState);

        Robot currentRobot = currentState.GetComponent<Robot>();
        // currentRobot.InitPose();

        // globalPath[globalPath.Count - 1] += Vector3.up * 0.22f;
        // for (int i = globalPath.Count - 2; i >= 0; i--) {
        //     Vector3 originPose = globalPath[i];
        //     Transform newState = Instantiate(runnerState, globalPath[i], GetDeltaAlignedOrigin(globalPath[i + 1] - globalPath[i]), transform);
        //     localStates.Add(newState);

        //     Robot newRobot = newState.GetComponent<Robot>();
        //     for (int n = 0; n < 5; ++n){
        //         Vector3 horizonShift = newRobot.GetDisplacementByLeg();
        //         Debug.DrawRay(globalPath[i], horizonShift, Color.green);
        //         newState.position += horizonShift;
        //         globalPath[i] += horizonShift;
        //         newState.rotation = GetDeltaAlignedOrigin(globalPath[i + 1] - globalPath[i]);
        //     }
            
        //     Vector3 vertBodyShift = newRobot.GetBodyHeightByLeg();
        //     Debug.DrawRay(globalPath[i], vertBodyShift, Color.green);
        //     newState.position += vertBodyShift;
        //     globalPath[i] += vertBodyShift;
        //     newState.rotation = GetDeltaAlignedOrigin(globalPath[i + 1] - globalPath[i]);

        //     bool isFine = newRobot.PlaceFoot();
        //     if (isFine)
        //         newState.gameObject.SetActive(true);
        //     else{
        //         newState.gameObject.SetActive(false);
        //         gPlanner.SetWieght(gPlanner.GetDescrete(originPose), 1.0f);
        //     }
        // }
    }
    // void Awake() {
    // }
    // void Start() {
        
    // }
    // private void GetLocalPlan(){
    //     Vector3 currentGoalDelta = GetDelta(currentState);

    //     Debug.Log("curState: " + currentState + "\n" + 
    //         "curGoalDelta: " + currentGoalDelta);
    //     Debug.DrawRay(currentState, currentGoalDelta, Color.red);
    // }
}
