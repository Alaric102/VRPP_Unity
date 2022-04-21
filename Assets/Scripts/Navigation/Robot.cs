using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Robot : MonoBehaviour {
    private bool isShowPotentialsSphere = false;
    private bool isShowFeasiblePotentials = true;
    private bool isShowObstacleFreePotentials = false;
    private bool isShowInCollision = false;
    private class Leg {
        public Leg(Transform robot, Transform zeroJoint){
            joint0 = zeroJoint;
            joint1 = joint0.GetChild(0);
            joint2 = joint1.GetChild(0);
            endEffector = joint2.GetChild(0);
            
            relativeHeapPosition = joint1.position - robot.position;
            
            // { // Define joints length
            // for (int id = 0; id < jointDistances.Capacity; ++id)
            //     jointDistances.Add(Vector3.Distance(FLjoints[id].position, FLjoints[id + 1].position));
            // }
        }
        public Transform joint0 = null, joint1 = null, joint2 = null, endEffector = null;
        public Vector3 relativeHeapPosition;

    }
    private List<Leg> legs = new List<Leg>();

    [Header("Common parameters")]
    private Transform body;
    private List<float> jointDistances = new List<float>(3);
    private List<Vector3> heapRelativePose = new List<Vector3>(4);
    public float discretization = 0.5f;
    private List<float> maxAngles = new List<float>{60.0f, 180.0f, 90.0f};
    private List<float> minAngles = new List<float>{-60.0f, -180.0f, -0.0f};
    public float minDistance = 0.1053526f, maxDistance = 0.3111828f;
    
    [Header("FL leg variables")]
    private List<Transform> FLjoints = new List<Transform>(4);
    
    [Header("FR leg variables")]
    private List<Transform> FRjoints = new List<Transform>(4);
    
    [Header("RL leg variables")]
    private List<Transform> RLjoints = new List<Transform>(4);
    
    [Header("RR leg variables")]
    private List<Transform> RRjoints = new List<Transform>(4);
    private List<Tuple<Vector3, Vector3>> collisionData = new List<Tuple<Vector3, Vector3>>();
    public class GaitControll {
        public GaitControll(){
            status = new List<bool>{true, false, false, false};
        }
        public List<bool> status = new List<bool>();
    }

    GaitControll gaitController = new GaitControll();
    void Awake() {
        Initialize();
    }
    private void Initialize(){
        body = transform.GetChild(0);
        { // Define FL joints
            FLjoints.Add(body.GetChild(0));
            for (int id = 1; id < FLjoints.Capacity; ++id)
                FLjoints.Add(FLjoints[id - 1].GetChild(0));
        }
        { // Define FR joints
            FRjoints.Add(body.GetChild(1));
            for (int id = 1; id < FRjoints.Capacity; ++id)
                FRjoints.Add(FRjoints[id - 1].GetChild(0));
        }
        { // Define RL joints
            RLjoints.Add(body.GetChild(2));
            for (int id = 1; id < RLjoints.Capacity; ++id)
                RLjoints.Add(RLjoints[id - 1].GetChild(0));
        }
        { // Define RR joints
            RRjoints.Add(body.GetChild(3));
            for (int id = 1; id < RRjoints.Capacity; ++id)
                RRjoints.Add(RRjoints[id - 1].GetChild(0));
        }
        { // Define Relative Joint2 positions
            heapRelativePose = new List<Vector3>{ 
                FLjoints[1].position - transform.position, FRjoints[1].position - transform.position, 
                RLjoints[1].position - transform.position, RRjoints[1].position - transform.position};
        }
        { // Define joints length
            for (int id = 0; id < jointDistances.Capacity; ++id)
                jointDistances.Add(Vector3.Distance(FLjoints[id].position, FLjoints[id + 1].position));
        }
        maxDistance = Mathf.Sqrt( Mathf.Pow(jointDistances[0], 2.0f) + Mathf.Pow((jointDistances[1] + jointDistances[2]), 2.0f)); 
    }
    private float Joint1AngleLeftSide(Vector3 pos, float alpha, float betta){
        float angle = 0.0f;
        if (pos.x > 0.0f)
            angle = Mathf.PI - betta - alpha;
        else if (pos.x > -jointDistances[0])
            angle = betta - alpha;
        else
            angle = -(alpha - betta);
        angle *= Mathf.Rad2Deg;
        return angle;
    }
    private float Joint1AngleRightSide(Vector3 pos, float alpha, float betta){
        float angle = 0.0f;
        if (pos.x < 0.0f)
            angle = Mathf.PI - betta - alpha;
        else if (pos.x < jointDistances[0])
            angle = betta - alpha;
        else
            angle = -(alpha - betta);
        angle *= -Mathf.Rad2Deg;
        return angle;
    }
    private float GetJoint1Angle(Vector3 pos, Func<Vector3, float, float, float> SideFunction){
        float pos_xy = Mathf.Sqrt(Mathf.Pow(pos.y, 2.0f) + Mathf.Pow(pos.x, 2.0f));
        float conjugateJoint1Alpha = Mathf.Acos( Mathf.Abs(jointDistances[0])/pos_xy );
        float conjugateJoint1Betta = Mathf.Acos( Mathf.Abs(pos.x)/pos_xy );
        
        float angle = SideFunction(pos, conjugateJoint1Alpha, conjugateJoint1Betta);

        if (angle < minAngles[0] || angle > maxAngles[0])
            return float.NaN;
        return angle;
    }
    private float GetJoint2Angle(Vector3 pos){
        float angle = 0.0f;
        float pos_xy = Mathf.Sqrt( Mathf.Pow(pos.y, 2.0f) + Mathf.Pow(pos.x, 2.0f) );
        float h = Mathf.Sqrt(Mathf.Pow(pos_xy, 2.0f) - Mathf.Pow(jointDistances[0], 2.0f));
        float pos_joints23 = Mathf.Sqrt( Mathf.Pow(pos.z, 2.0f) + Mathf.Pow(h, 2.0f) );

        float conjugateJoint2 = Mathf.Acos( 
            (Mathf.Pow(jointDistances[2], 2.0f) - Mathf.Pow(jointDistances[1], 2.0f) - Mathf.Pow(pos_joints23, 2.0f))/
            (-2.0f * jointDistances[1] * pos_joints23));
        if (pos.z >= 0)
            conjugateJoint2 += Mathf.Acos( Mathf.Abs(pos.z)/pos_joints23 );
        else
            conjugateJoint2 += Mathf.PI - Mathf.Acos( Mathf.Abs(pos.z)/pos_joints23 );
        angle = -(Mathf.PI - conjugateJoint2) * Mathf.Rad2Deg;

        if (angle < minAngles[1] || angle > maxAngles[1])
            return float.NaN;
        return angle;
    }
    private float GetJoint3Angle(Vector3 pos){
        float pos_xy = Mathf.Sqrt( Mathf.Pow(pos.y, 2.0f) + Mathf.Pow(pos.x, 2.0f) );
        float h = Mathf.Sqrt( Mathf.Pow(pos_xy, 2.0f) - Mathf.Pow(jointDistances[0], 2.0f) );
        float pos_joints23 = Mathf.Sqrt( Mathf.Pow(pos.z, 2.0f) + Mathf.Pow(h, 2.0f) );

        float angle = Mathf.Acos( 
            (Mathf.Pow(pos_joints23, 2.0f) - Mathf.Pow(jointDistances[1], 2.0f) - Mathf.Pow(jointDistances[2], 2.0f))/
            (-2.0f * jointDistances[1] * jointDistances[2])) * Mathf.Rad2Deg;

        if (angle < minAngles[2] || angle > maxAngles[2])
            return float.NaN;
        return angle;
    }
    private List<float> ResolveInverseKinematicsFL(Vector3 pos){
        return new List<float>{ GetJoint1Angle(pos, Joint1AngleLeftSide), GetJoint2Angle(pos), GetJoint3Angle(pos) };
    }
    private List<float> ResolveInverseKinematicsFR(Vector3 pos){
        return new List<float>{ GetJoint1Angle(pos, Joint1AngleRightSide), GetJoint2Angle(pos), GetJoint3Angle(pos) };
    }
    private List<float> ResolveInverseKinematicsRL(Vector3 pos){
        return new List<float>{ GetJoint1Angle(pos, Joint1AngleLeftSide), GetJoint2Angle(pos), GetJoint3Angle(pos) };
    }
    private List<float> ResolveInverseKinematicsRR(Vector3 pos){
        return new List<float>{ GetJoint1Angle(pos, Joint1AngleRightSide), GetJoint2Angle(pos), GetJoint3Angle(pos) };
    }
    private bool IsAnglesInCollision(List<Transform> joints, List<float> angles, bool isShow = false){
        /*
        Return true if any collision in joints, otherwise false
        */
        bool res = false;
        // Check joint0
        joints[0].localRotation = Quaternion.Euler(angles[0], 0, 0);
        var jointCollision = InCollision(joints[0]);
        if ( jointCollision ){
            res = res || jointCollision;
        }
        // Check joint1
        joints[1].localRotation = Quaternion.Euler(0, angles[1], 0);
        jointCollision = InCollision(joints[1]);
        if ( jointCollision ){
            res = res || jointCollision;
        }
        joints[2].localRotation = Quaternion.Euler(0, angles[2], 0);
        return res;
    }
    private bool InCollision(Transform obj){
        /* 
        Return true if in Collision, otherwise return false
        */
        // Find raycast verteces
        List<Vector3> verts = new List<Vector3>();
        BoxCollider b = obj.GetComponent<BoxCollider>();
        for (int x = -1; x < 2; x += 2){
            for (int y = -1; y < 2; y += 2){
                for (int z = -1; z < 2; z += 2){
                    Vector3 v = (b.center + new Vector3(x * b.size.x, y * b.size.y, z * b.size.z) * 0.5f);
                    verts.Add( obj.TransformPoint(v) );
                }
            }
        }
        // Find Collisions
        bool res = false;
        for (int i = 0; i < verts.Count; i++) {
            for (int j = 0; j < verts.Count; j++) {
                if (i == j) continue;
                RaycastHit hit;
                Vector3 direction = verts[j] - verts[i];
                if (Physics.Raycast(verts[i], direction, out hit, direction.magnitude, ~(1 << 6))){
                    Vector3 toBodyDirection = hit.point - transform.position;
                    collisionData.Add(new Tuple<Vector3, Vector3>(toBodyDirection, hit.normal));
                    res = true;
                    if (isShowInCollision) Debug.DrawLine(verts[i], hit.point, Color.red);
                }
            }
        }
        return res;
    }
    public List<Tuple<Vector3, Quaternion>> GetHeapMovements(Vector3 deltaTransition, Quaternion deltaRotation,
            float dStep, bool isShow = false){
        // Calculate heap translation and rotation movements
        List<Tuple<Vector3, Quaternion>> deltas = new List<Tuple<Vector3, Quaternion>>(4);
        foreach (var heapPose in heapRelativePose) {
            Vector3 deltaRot = Vector3.ClampMagnitude(deltaRotation * heapPose - heapPose, dStep);
            Vector3 deltaTrans = Vector3.ClampMagnitude(deltaTransition, dStep);
            deltas.Add(new Tuple<Vector3, Quaternion>(deltaTrans, Quaternion.FromToRotation(heapPose, heapPose + deltaRot)));

            if(isShow) Debug.DrawRay(transform.position + heapPose, deltaTrans + deltaRot, Color.red);
        }
        return deltas;
    }
    public class Potential{
        private Vector3 _direction, _propagation, _normal;
        private List<float> _angles = new List<float>();
        public Vector3 crossDirectionPropagation, crossNormalPropagation;
        public Potential(){
            _direction = Vector3.zero; _propagation = Vector3.zero; _normal = Vector3.zero;
        }
        public Potential(Vector3 direction, Vector3 propagation, Vector3 normal) {
            _direction = direction; _propagation = propagation; _normal = normal;
            crossDirectionPropagation = Vector3.Cross(_direction, _propagation);
            crossNormalPropagation = Vector3.Cross(_normal, _propagation);
        }
        public Vector3 GetPropagation() { return _propagation; }
        public Vector3 GetNormal() { return _normal; }
        public void SetAngles(List<float> angles) { _angles = angles; }
        public List<float> GetAngles() { return _angles; }
    }
    private int ComparePotentials(Potential lft, Potential rht){
        /* 
        Function to comapre to potentials
        */
        float lftWeight = lft.crossDirectionPropagation.magnitude * lft.crossNormalPropagation.magnitude;
        float rhtWeight = rht.crossDirectionPropagation.magnitude * rht.crossNormalPropagation.magnitude;
        return lftWeight.CompareTo(rhtWeight);
    }
    private static int ComparePotentialsOut(Potential lft, Potential rht){
        /* 
        Function to comapre to potentials
        */
        float lftWeight = lft.crossDirectionPropagation.magnitude;
        float rhtWeight = rht.crossDirectionPropagation.magnitude;
        return lftWeight.CompareTo(rhtWeight);
    }
    public List<Potential> resolvedPotentials = new List<Potential>();
    public int activeLeg; // FL = 0, FR = 1, RL = 2; RR = 3
    public List<Vector3> accDeltaStep = new List<Vector3>();
    public Vector3 accActiveDelta = Vector3.zero;
    public List<Vector3> GetLegDirectionsByMotion(
            List<Tuple<Vector3, Quaternion>> deltas,
            List<Potential> lastPropagations,
            bool isShow = false){
        // Find next active leg by biggest accumulated delta
        float maxAccumulated = float.MinValue;
        for(int i = 0; i < accDeltaStep.Count; ++i)
            if (accDeltaStep[i].magnitude > maxAccumulated){
                activeLeg = i;
                maxAccumulated = accDeltaStep[i].magnitude;
            }
        // Debug.Log("active leg: " + activeLeg);

        List<Vector3> res = new List<Vector3>();
        for (int i = 0; i < heapRelativePose.Count; i++){
            // Calculate delta vector
            Vector3 transition = deltas[i].Item1;
            Quaternion rotation = deltas[i].Item2;
            Vector3 delta = -(transition + rotation * heapRelativePose[i] - heapRelativePose[i]);
            // Define direction to propagate
            Vector3 direction = Vector3.zero;
            if (i == activeLeg){ // Active leg direction should compencate accumulatted delta vector
                Vector3 compencation = accDeltaStep[i];
                accDeltaStep[i] -= compencation;
                direction = lastPropagations[i].GetPropagation() - compencation;
                accActiveDelta += delta + compencation;
            } else { // Should stay on same place, but accumulate delta
                direction = lastPropagations[i].GetPropagation() + delta;
                accDeltaStep[i] += delta;
            }
            
            res.Add(direction);

            if (isShow) Debug.DrawRay(transform.position + heapRelativePose[i], accDeltaStep[i], Color.yellow);
            if (isShow) Debug.DrawRay(transform.position + heapRelativePose[i], direction, Color.white);
            if (isShow) Debug.DrawRay(transform.position + heapRelativePose[i] + delta, lastPropagations[i].GetPropagation(), Color.gray);
        }
        
        return res;
    }
    public int FindActiveLeg(List<Vector3> directions){
        int nextActive = activeLeg;
        float maxAngle = 0.0f;
        for (int i = 0; i < heapRelativePose.Count; i++){
            if (i == activeLeg){
                Debug.Log(i + " active: " + accActiveDelta.magnitude);
                continue;
            }
            float angle = Vector3.Angle(directions[i], directions[activeLeg]);
            Debug.Log(i + " passive: " + accDeltaStep[i] + "\n" + angle);
                
            if (angle > 30.0f && angle > maxAngle){
                nextActive = i;
                maxAngle = angle;
            }
        }
        return nextActive;
    }
    private List<Potential> GetPotentials(
        Vector3 pos, 
        Vector3 primeDirection,
        Func<Potential, Potential, int> sortFunc)
        {
        // 1. Creates a sphere raycast from position {pose} directed along prime direction {primeDirection}.
        List<Vector3> rayDirections = new List<Vector3>{primeDirection};
        for (float latitude = -0.5f * Mathf.PI; latitude < 0.5f * Mathf.PI; latitude += discretization){
            for (float longitude = -0.25f * Mathf.PI; longitude < 0.25f * Mathf.PI; longitude += discretization){
                Quaternion rotation = Quaternion.Euler(Mathf.Rad2Deg * longitude, Mathf.Rad2Deg * latitude, 0.0f);
                rayDirections.Add(rotation * primeDirection.normalized);
            }
        }

        // 2. Store Potential data (primeDirection, propagation, normal) in potentails List
        List<Potential> potentials = new List<Potential>();
        foreach (Vector3 dir in rayDirections) {
            RaycastHit hit;
            if (Physics.Raycast(pos, dir, out hit, Mathf.Infinity, ~(1 << 6))){
                if (isShowPotentialsSphere) Debug.DrawLine(pos, hit.point, Color.gray);
                potentials.Add(new Potential(primeDirection, hit.point - pos, hit.normal));
            }
        }

        // 3. Sorts potentials by {sortFunc} 
        potentials.Sort(new Comparison<Potential>(sortFunc));
        return potentials;
    }
    private List<Potential> GetFilteredPotentials(
            List<Potential> potentials, 
            List<Transform> jointsToApply,
            Func<Vector3, List<float>> inverseKinematicsFunc)
        {
        // 1. Filter potentials by several conditions
        List<Potential> feasiblePotentials = new List<Potential>();
        float filterAngle = 45.0f;
        foreach (var potential in potentials) {
            // Don't use vertical surfaces
            float angle = Mathf.Abs( Vector3.Angle(Vector3.up, potential.GetNormal()));
            if (angle > filterAngle)
                continue;
            // define propagation vector from zero joint
            Vector3 propagationFromJoint0 = transform.InverseTransformVector(potential.GetPropagation() + 
                (jointsToApply[1].position - jointsToApply[0].position));
            // Don't use far distances
            if (propagationFromJoint0.magnitude >= maxDistance)
                continue;
            // Don't use unresolvable propagations
            List<float> angles = inverseKinematicsFunc(propagationFromJoint0);
            if (float.IsNaN(angles[0]) || float.IsNaN(angles[1]) || float.IsNaN(angles[2]))
                continue;
            // if feasible save angles and append to list
            potential.SetAngles(angles);
            feasiblePotentials.Add(potential);

            if (isShowFeasiblePotentials) 
                Debug.DrawRay(jointsToApply[0].position, transform.TransformVector(propagationFromJoint0), Color.green);
        }
        return feasiblePotentials;
    }
    private List<Potential> GetFilteredPotentialsOut(
            List<Potential> potentials, 
            List<Transform> jointsToApply,
            Func<Vector3, List<float>> inverseKinematicsFunc)
        {
        // 1. Filter potentials by several conditions
        List<Potential> feasiblePotentials = new List<Potential>();
        float filterAngle = 45.0f;
        foreach (var potential in potentials) {
            // Don't use vertical surfaces
            float angle = Mathf.Abs( Vector3.Angle(Vector3.up, potential.GetNormal()));
            if (angle > filterAngle)
                continue;
            // define propagation vector from zero joint
            Vector3 propagationFromJoint0 = transform.InverseTransformVector(potential.GetPropagation() * 0.8f + 
                (jointsToApply[1].position - jointsToApply[0].position));
            // Don't use far distances
            if (propagationFromJoint0.magnitude <= minDistance)
                continue;
            // Don't use unresolvable propagations
            List<float> angles = inverseKinematicsFunc(propagationFromJoint0);
            // if (angles[0] == float.NaN || angles[1] == float.NaN || angles[2] == float.NaN)
            
            if (float.IsNaN(angles[0]) || float.IsNaN(angles[1]) || float.IsNaN(angles[2]))
                continue;
            // if feasible save angles and append to list
            potential.SetAngles(angles);
            feasiblePotentials.Add(potential);

            if (isShowFeasiblePotentials) 
                Debug.DrawRay(jointsToApply[0].position, transform.TransformVector(propagationFromJoint0), Color.green);
        }
        return feasiblePotentials;
    }
    private bool IsObstacleFreeExists(
            List<Transform> jointsToApply,
            Vector3 primeDirection,
            Func<Potential, Potential, int> sortPotentialsFunc,
            Func<List<Potential>, List<Transform>, Func<Vector3, List<float>>, List<Potential>> filterPotentialsFunc,
            Func<Vector3, List<float>> inverseKinematicsFunc)
    {
        // Get potentials around pose {jointsToApply[1].position} with direction {primeDirection} sorted by {sortPotentialsFunc}
        List<Potential> potentials = GetPotentials(jointsToApply[1].position, primeDirection, sortPotentialsFunc);
        if (potentials.Count == 0){
            Debug.Log("No potentials.");
            return false;
        }
        
        // Filter potentials {potentials} for joints {jointsToApply} and IK function {inverseKinematicsFunc}
        potentials = filterPotentialsFunc(potentials, jointsToApply, inverseKinematicsFunc);
        if (potentials.Count == 0){
            collisionData.Add(new Tuple<Vector3, Vector3>(-Vector3.up * 0.01f, Vector3.zero));
            Debug.Log("No feasuble potentials.");
            return false;
        }
        
        // // Vector3 meanNormal = Vector3.zero;
        // // int collisionCounter = 0;
        foreach (var potential in potentials) {
            bool isInCollision = IsAnglesInCollision(jointsToApply, potential.GetAngles(), false);
            if (isInCollision){
                continue;
            }
            resolvedPotentials.Add(potential);
            if (isShowObstacleFreePotentials) Debug.DrawRay(jointsToApply[1].position, potential.GetPropagation(), Color.green);
            return true;
        }
        Debug.Log("No obstacle free potentials");
        return false;
    }
    public bool IsPropagatable(List<Vector3> directions){
        bool res = true;

        for (int i = 0; i < heapRelativePose.Count; i++) {
            Vector3 pose = transform.position + heapRelativePose[i];
            Vector3 direction = directions[i];
            List<Transform> joints = FLjoints;
            Func<Vector3, List<float>> func = ResolveInverseKinematicsFL;
            switch (i) {
                case (0):
                    joints = FLjoints;
                    func = ResolveInverseKinematicsFL;
                    break;
                case (1):
                    joints = FRjoints;
                    func = ResolveInverseKinematicsFR;
                    break;
                case (2):
                    joints = RLjoints;
                    func = ResolveInverseKinematicsRL;
                    break;
                case (3):
                    joints = RRjoints;
                    func = ResolveInverseKinematicsRR;
                    break;
                default:
                    break;
            }
            if (i == activeLeg){
                res = res && IsObstacleFreeExists(joints, direction, ComparePotentialsOut, GetFilteredPotentialsOut, func);
            } else {
                res = res && IsObstacleFreeExists(joints, direction, ComparePotentials, GetFilteredPotentials, func);
            }
        }
        return res;
    }

    public Vector3 GetDeviationVector(){
        Vector3 res = Vector3.zero;
        foreach (var item in collisionData) {
            res += item.Item1;
        }
        if (collisionData.Count == 0)
            Debug.Log("Empty collisions");
        else {
            res /= collisionData.Count;
        }
        return res;
    }
}
