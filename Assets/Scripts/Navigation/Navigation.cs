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
        lPlanner.SetGlobalPath(globalPlan);
        gPlanner.ShowPlan(globalPlan);
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
    // private Quaternion GetSafetyRotattion(Vector3 meanPose, Quaternion localOrigin){
    //     Vector3 fromFLHeap = meanPose + localOrigin * Vector3.forward * displaysmentFront - localOrigin * Vector3.right * displaysmentSlide;
    //     Vector3 fromFRHeap = meanPose + localOrigin * Vector3.forward * displaysmentFront + localOrigin * Vector3.right * displaysmentSlide;
    //     Vector3 fromRLHeap = meanPose -(localOrigin * Vector3.forward) * displaysmentFront - (localOrigin * Vector3.right) * displaysmentSlide;
    //     Vector3 fromRRHeap = meanPose -(localOrigin * Vector3.forward) * displaysmentFront + localOrigin * Vector3.right * displaysmentSlide;
    //     RaycastHit hitFLHeapSafe, hitFRHeapSafe, hitRLHeapSafe, hitRRHeapSafe;
    //     Quaternion displacement = Quaternion.identity;
    //     float FLMeanDistance = displaysmentUp;
    //     float FRMeanDistance = displaysmentUp;
    //     float RLMeanDistance = displaysmentUp;
    //     float RRMeanDistance = displaysmentUp;
    //     if (Physics.Raycast(fromFLHeap, -(localOrigin * Vector3.up), out hitFLHeapSafe, fromFLHeap.magnitude, ~0)){
    //         Debug.DrawLine(fromFLHeap, hitFLHeapSafe.point, Color.black);
    //         FLMeanDistance = (hitFLHeapSafe.point - fromFLHeap).magnitude;
    //     }
    //     if (Physics.Raycast(fromFRHeap,  -(localOrigin * Vector3.up), out hitFRHeapSafe, fromFRHeap.magnitude, ~0)){
    //         Debug.DrawLine(fromFRHeap, hitFRHeapSafe.point, Color.black);
    //         FRMeanDistance = (hitFRHeapSafe.point - fromFRHeap).magnitude;
    //     }
    //     if (Physics.Raycast(fromRLHeap,  -(localOrigin * Vector3.up), out hitRLHeapSafe, fromRLHeap.magnitude, ~0)){
    //         Debug.DrawLine(fromRLHeap, hitRLHeapSafe.point, Color.black);
    //         RLMeanDistance = (hitRLHeapSafe.point - fromRLHeap).magnitude;
    //     }
    //     if (Physics.Raycast(fromRRHeap,  -(localOrigin * Vector3.up), out hitRRHeapSafe, fromRRHeap.magnitude, ~0)){
    //         Debug.DrawLine(fromRRHeap, hitRRHeapSafe.point, Color.black);
    //         RRMeanDistance = (hitRRHeapSafe.point - fromRRHeap).magnitude;
    //     }

    //     Vector3 newRight_FrontSide = Vector3.right * displaysmentSlide + Vector3.up * (FLMeanDistance - FRMeanDistance);
    //     Vector3 newRight_RearSide = Vector3.right * displaysmentSlide + Vector3.up * (RLMeanDistance - RRMeanDistance);
    //     Quaternion rotFront1 = Quaternion.FromToRotation(Vector3.right * displaysmentSlide, newRight_FrontSide);
    //     Quaternion rotFront2 = Quaternion.FromToRotation(Vector3.right * displaysmentSlide, newRight_RearSide);
    //     Quaternion a = (rotFront1 * rotFront2).normalized;

    //     Vector3 newForward_RightSide = Vector3.forward * displaysmentFront + Vector3.up * (RRMeanDistance - FRMeanDistance);
    //     Vector3 newForward_LeftSide = Vector3.forward * displaysmentFront + Vector3.up * (RLMeanDistance - FLMeanDistance);
    //     Quaternion rotRight1 = Quaternion.FromToRotation(Vector3.forward * displaysmentFront, newForward_RightSide);
    //     Quaternion rotRight2 = Quaternion.FromToRotation(Vector3.forward * displaysmentFront, newForward_LeftSide);
    //     Quaternion b = Quaternion.Slerp(rotRight1, rotRight2, 0.5f);
    //     return a*b;
    // }
    // private Vector3 GetHorizonDisplacement(Vector3 pos, Quaternion rot){

    // }
}
