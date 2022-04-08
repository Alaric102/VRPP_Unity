using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Robot : MonoBehaviour {
    [Header("Common parameters")]
    private Transform body;
    private float lengthJoint1, lengthJoint2, lengthJoint3;
    public float discretization = 0.5f;
    public List<float> maxAngles = new List<float>{90.0f, 180.0f, 180.0f};
    public List<float> minAngles = new List<float>{-90.0f, -180.0f, -180.0f};
    private float minDistance = 0.1053526f, maxDistance = 0.3111828f;
    
    [Header("FL leg variables")]
    private Transform FLjoint1, FLjoint2, FLjoint3, FLEndEffector;
    public List<float> FLAngles = new List<float>{0.0f, 0.0f, 0.0f};
    
    [Header("FR leg variables")]
    private Transform FRjoint1, FRjoint2, FRjoint3, FREndEffector;
    public List<float> FRAngles = new List<float>{0.0f, 0.0f, 0.0f};
    
    [Header("RL leg variables")]
    private Transform RLjoint1, RLjoint2, RLjoint3, RLEndEffector;
    public List<float> RLAngles = new List<float>{0.0f, 0.0f, 0.0f};
    
    [Header("RR leg variables")]
    private Transform RRjoint1, RRjoint2, RRjoint3, RREndEffector;
    public List<float> RRAngles = new List<float>{0.0f, 0.0f, 0.0f};
    void Awake() {
        body = transform.GetChild(0);
        // Define FL joints
        FLjoint1 = body.GetChild(0);
        FLjoint2 = FLjoint1.GetChild(0);
        FLjoint3 = FLjoint2.GetChild(0);
        FLEndEffector = FLjoint3.GetChild(0);
        // Define FR joints
        FRjoint1 = body.GetChild(1);
        FRjoint2 = FRjoint1.GetChild(0);
        FRjoint3 = FRjoint2.GetChild(0);
        FREndEffector = FRjoint3.GetChild(0);
        // Define RL joints
        RLjoint1 = body.GetChild(2);
        RLjoint2 = RLjoint1.GetChild(0);
        RLjoint3 = RLjoint2.GetChild(0);
        RLEndEffector = RLjoint3.GetChild(0);
        // Define RR joints
        RRjoint1 = body.GetChild(3);
        RRjoint2 = RRjoint1.GetChild(0);
        RRjoint3 = RRjoint2.GetChild(0);
        RREndEffector = RRjoint3.GetChild(0);

        lengthJoint1 = Vector3.Distance(FLjoint1.position, FLjoint2.position);
        lengthJoint2 = Vector3.Distance(FLjoint2.position, FLjoint3.position);
        lengthJoint3 = Vector3.Distance(FLjoint3.position, FLEndEffector.position);
    }
    private float GetJoint1Angle_LeftLegs(Vector3 pos){
        float pos_xy = Mathf.Sqrt(Mathf.Pow(pos.y,2.0f) + Mathf.Pow(pos.x,2.0f));
        float conjugateJoint1Alpha = Mathf.Acos( Mathf.Abs(lengthJoint1)/pos_xy);
        float conjugateJoint1Betta = Mathf.Acos( Mathf.Abs(pos.x)/pos_xy);
        float angle = 0.0f;

        if (pos.x > 0.0f)
            angle = Mathf.PI - conjugateJoint1Betta - conjugateJoint1Alpha;
        else if (pos.x > -lengthJoint1)
            angle = conjugateJoint1Betta - conjugateJoint1Alpha;
        else
            angle = -(conjugateJoint1Alpha - conjugateJoint1Betta);
        
        angle *= Mathf.Rad2Deg;
        if (angle < minAngles[0])
            angle = minAngles[0];
        else if (angle > maxAngles[0])
            angle = maxAngles[0];
        return angle;
    }
    private float GetJoint1Angle_RightLegs(Vector3 pos){
        float pos_xy = Mathf.Sqrt(Mathf.Pow(pos.y,2.0f) + Mathf.Pow(pos.x,2.0f));
        float conjugateJoint1Alpha = Mathf.Acos( Mathf.Abs(lengthJoint1)/pos_xy);
        float conjugateJoint1Betta = Mathf.Acos( Mathf.Abs(pos.x)/pos_xy);
        float angle = 0.0f;

        if (pos.x < 0.0f)
            angle = Mathf.PI - conjugateJoint1Betta - conjugateJoint1Alpha;
        else if (pos.x < lengthJoint1)
            angle = conjugateJoint1Betta - conjugateJoint1Alpha;
        else
            angle = -(conjugateJoint1Alpha - conjugateJoint1Betta);
        angle *= -Mathf.Rad2Deg;

        if (angle < minAngles[0])
            angle = minAngles[0];
        else if (angle > maxAngles[0])
            angle = maxAngles[0];
        return angle;
    }
    private float GetJoint3Angle(Vector3 pos){
        float pos_xy = Mathf.Sqrt(Mathf.Pow(pos.y,2.0f) + Mathf.Pow(pos.x,2.0f));
        float h = Mathf.Sqrt(Mathf.Pow(pos_xy,2.0f) - Mathf.Pow(lengthJoint1,2.0f));
        float pos_joints23 = Mathf.Sqrt( Mathf.Pow(pos.z, 2.0f) + Mathf.Pow(h, 2.0f) );

        float angle = Mathf.Acos( 
            (Mathf.Pow(pos_joints23, 2) - Mathf.Pow(lengthJoint2,2) - Mathf.Pow(lengthJoint3,2))/
            (-2.0f * lengthJoint2 * lengthJoint3)) * Mathf.Rad2Deg;

        if (angle < minAngles[2])
            angle = minAngles[2];
        else if (angle > maxAngles[2])
            angle = maxAngles[2];
        return angle;
    }
    private float GetJoint2Angle(Vector3 pos){
        float angle = 0.0f;
        float pos_xy = Mathf.Sqrt(Mathf.Pow(pos.y,2.0f) + Mathf.Pow(pos.x,2.0f));
        float h = Mathf.Sqrt(Mathf.Pow(pos_xy,2.0f) - Mathf.Pow(lengthJoint1,2.0f));
        float pos_joints23 = Mathf.Sqrt( Mathf.Pow(pos.z, 2.0f) + Mathf.Pow(h, 2.0f) );

        float conjugateJoint2 = Mathf.Acos( 
            (Mathf.Pow(lengthJoint3, 2) - Mathf.Pow(lengthJoint2,2) - Mathf.Pow(pos_joints23,2))/
            (-2.0f * lengthJoint2 * pos_joints23));
        if (pos.z >= 0)
            conjugateJoint2 += Mathf.Acos( Mathf.Abs(pos.z)/pos_joints23);
        else
            conjugateJoint2 += Mathf.PI - Mathf.Acos( Mathf.Abs(pos.z)/pos_joints23);
        angle = -(Mathf.PI - conjugateJoint2) * Mathf.Rad2Deg;

        if (angle < minAngles[1])
            angle = minAngles[1];
        else if (angle > maxAngles[1])
            angle = maxAngles[1];
        return angle;
    }
    private List<float> ResolveInverseKinematicsFL(Vector3 pos){
        // Debug.Log(pos);
        List<float> res = new List<float>{ GetJoint1Angle_LeftLegs(pos), GetJoint2Angle(pos), GetJoint3Angle(pos) };
        // foreach (var item in res){
        //     Debug.Log(item);
        // }
        return res;
    }
    private List<float> ResolveInverseKinematicsFR(Vector3 pos){
        // Debug.Log(pos);
        List<float> res = new List<float>{ GetJoint1Angle_RightLegs(pos), GetJoint2Angle(pos), GetJoint3Angle(pos) };
        // foreach (var item in res){
        //     Debug.Log(item);
        // }
        return res;
    }
    private List<float> ResolveInverseKinematicsRL(Vector3 pos){
        // Debug.Log(pos);
        List<float> res = new List<float>{ GetJoint1Angle_LeftLegs(pos), GetJoint2Angle(pos), GetJoint3Angle(pos) };
        // foreach (var item in res){
        //     Debug.Log(item);
        // }
        return res;
    }
    private List<float> ResolveInverseKinematicsRR(Vector3 pos){
        // Debug.Log(pos);
        List<float> res = new List<float>{ GetJoint1Angle_RightLegs(pos), GetJoint2Angle(pos), GetJoint3Angle(pos) };
        // foreach (var item in res){
        //     Debug.Log(item);
        // }
        return res;
    }
    private bool ApplyFLAngles(List<float> angles){
        if (angles.Count != 3)
            return false;
        foreach (var angle in angles) {
            if (angle == float.NaN)
                return false;
        }
        FLjoint1.localRotation = Quaternion.Euler(angles[0], 0, 0);
        FLjoint2.localRotation = Quaternion.Euler(0, angles[1], 0);
        FLjoint3.localRotation = Quaternion.Euler(0, angles[2], 0);
        return true;
    }
    private bool ApplyFRAngles(List<float> angles){
        if (angles.Count != 3)
            return false;
        foreach (var angle in angles) {
            if (angle == float.NaN)
                return false;
        }
        FRjoint1.localRotation = Quaternion.Euler(angles[0], 0, 0);
        FRjoint2.localRotation = Quaternion.Euler(0, angles[1], 0);
        FRjoint3.localRotation = Quaternion.Euler(0, angles[2], 0);
        return true;
    }
    private bool ApplyRLAngles(List<float> angles){
        if (angles.Count != 3)
            return false;
        foreach (var angle in angles) {
            if (angle == float.NaN)
                return false;
        }
        RLjoint1.localRotation = Quaternion.Euler(angles[0], 0, 0);
        RLjoint2.localRotation = Quaternion.Euler(0, angles[1], 0);
        RLjoint3.localRotation = Quaternion.Euler(0, angles[2], 0);
        return true;
    }
    private bool ApplyRRAngles(List<float> angles){
        if (angles.Count != 3)
            return false;
        foreach (var angle in angles) {
            if (angle == float.NaN)
                return false;
        }
        RRjoint1.localRotation = Quaternion.Euler(angles[0], 0, 0);
        RRjoint2.localRotation = Quaternion.Euler(0, angles[1], 0);
        RRjoint3.localRotation = Quaternion.Euler(0, angles[2], 0);
        return true;
    }
    private List<Tuple<Vector3, Vector3>> GetSphereData(Vector3 pos, Quaternion rot){
        List<Vector3> rayDirections = new List<Vector3>();
        for (float latitude = 0.0f; latitude < 2.0f * Mathf.PI; latitude += discretization)
            for (float longitude = 0.0f; longitude < Mathf.PI; longitude += discretization){
                Quaternion yRotation = Quaternion.Euler(Mathf.Rad2Deg * longitude, Mathf.Rad2Deg * latitude, 0.0f);
                rayDirections.Add(yRotation * Vector3.forward * (lengthJoint2 + lengthJoint3) );
            }
        
        List<Tuple<Vector3, Vector3>> rayHitData = new List<Tuple<Vector3, Vector3>>(rayDirections.Count);
        foreach (Vector3 dir in rayDirections) {
            RaycastHit hit;
            if (Physics.Raycast(pos, dir, out hit, dir.magnitude, ~0)){
                rayHitData.Add( new Tuple<Vector3, Vector3>(hit.point - pos, hit.normal) );
            }
        }
        return rayHitData;
    }
    private List<Tuple<Vector3, Vector3>> FilterDataByAngle(List<Tuple<Vector3, Vector3>> rayHitData, float angle = 45.0f){
        List<Tuple<Vector3, Vector3>> res = new List<Tuple<Vector3, Vector3>>();
        foreach (var rayItem in rayHitData){
            if (Mathf.Abs(Vector3.Angle(Vector3.up, rayItem.Item2)) < angle)
                res.Add(rayItem);
        }
        return res;
    }
    private Vector3 GetDeltaSum(List<Tuple<Vector3, Vector3>> rayHitData){
        Vector3 sumDelta = Vector3.zero;
        foreach (var rayItem in rayHitData)
            sumDelta += rayItem.Item1;
        if (rayHitData.Count > 0)
            sumDelta /= rayHitData.Count;
        return sumDelta;
    }
    private List<Tuple<Vector3, Vector3>> FilterDataByLength(List<Tuple<Vector3, Vector3>> rayHitData){
        List<Tuple<Vector3, Vector3>> res = new List<Tuple<Vector3, Vector3>>();
        foreach (var rayItem in rayHitData){
            if (rayItem.Item1.magnitude < maxDistance)
                res.Add(rayItem);
        }
        return res;
    }
    private Vector3 GetMinima(List<Tuple<Vector3, Vector3>> rayHitData){
        Vector3 res = Vector3.zero;
        float minMagnitude = float.MaxValue;
        foreach (var rayItem in rayHitData) {
            if (rayItem.Item1.magnitude < minMagnitude){
                res = rayItem.Item1;
                minMagnitude = rayItem.Item1.magnitude;
            }                
        }
        return res;
    }
    public Vector3 GetDisplacementByLeg(){
        List<Tuple<Vector3, Vector3>> FLrayData = GetSphereData(FLjoint2.position, FLjoint2.rotation);
        List<Tuple<Vector3, Vector3>> FRrayData = GetSphereData(FRjoint2.position, FRjoint2.rotation);
        List<Tuple<Vector3, Vector3>> RLrayData = GetSphereData(RLjoint2.position, RLjoint2.rotation);
        List<Tuple<Vector3, Vector3>> RRrayData = GetSphereData(RRjoint2.position, RRjoint2.rotation);

        Vector3 FLdelta = Vector3.ProjectOnPlane(GetDeltaSum(FilterDataByAngle(FLrayData)), Vector3.up);
        Vector3 FRdelta = Vector3.ProjectOnPlane(GetDeltaSum(FilterDataByAngle(FRrayData)), Vector3.up);
        Vector3 RLdelta = Vector3.ProjectOnPlane(GetDeltaSum(FilterDataByAngle(RLrayData)), Vector3.up);
        Vector3 RRdelta = Vector3.ProjectOnPlane(GetDeltaSum(FilterDataByAngle(RRrayData)), Vector3.up);
        
        // Debug.DrawRay(FLjoint2.position, FLdelta, Color.gray); Debug.DrawRay(FRjoint2.position, FRdelta, Color.gray);
        // Debug.DrawRay(RLjoint2.position, RLdelta, Color.gray); Debug.DrawRay(RRjoint2.position, RRdelta, Color.gray);

        float totalWeight = FLdelta.magnitude + FRdelta.magnitude + RLdelta.magnitude + RRdelta.magnitude;
        Vector3 weightedDelta = 
            FLdelta.magnitude/totalWeight*FLdelta + 
            FRdelta.magnitude/totalWeight*FRdelta + 
            RLdelta.magnitude/totalWeight*RLdelta + 
            RRdelta.magnitude/totalWeight*RRdelta;

        return weightedDelta;
    }
    public Vector3 GetBodyHeightByLeg(float meanHeight = 0.22f){
        List<Tuple<Vector3, Vector3>> FLrayData = GetSphereData(FLjoint2.position, FLjoint2.rotation);
        List<Tuple<Vector3, Vector3>> FRrayData = GetSphereData(FRjoint2.position, FRjoint2.rotation);
        List<Tuple<Vector3, Vector3>> RLrayData = GetSphereData(RLjoint2.position, RLjoint2.rotation);
        List<Tuple<Vector3, Vector3>> RRrayData = GetSphereData(RRjoint2.position, RRjoint2.rotation);
        List<Tuple<Vector3, Vector3>> bodyRayData = GetSphereData(transform.position, transform.rotation);

        Vector3 FLminima = GetMinima(FilterDataByAngle(FLrayData));
        Vector3 FRminima = GetMinima(FilterDataByAngle(FRrayData));
        Vector3 RLminima = GetMinima(FilterDataByAngle(RLrayData));
        Vector3 RRminima = GetMinima(FilterDataByAngle(RRrayData));
        Vector3 bodyMinima = GetMinima(FilterDataByAngle(bodyRayData));

        Vector3 idealFLPose = FLjoint2.position + FLminima + Vector3.up * meanHeight;
        Vector3 FLdelta = Vector3.Project(idealFLPose - FLjoint2.position, Vector3.up);
        Vector3 idealFRPose = FRjoint2.position + FRminima + Vector3.up * meanHeight;
        Vector3 FRdelta = Vector3.Project(idealFRPose - FRjoint2.position, Vector3.up);
        Vector3 idealRLPose = RLjoint2.position + RLminima + Vector3.up * meanHeight;
        Vector3 RLdelta = Vector3.Project(idealRLPose - RLjoint2.position, Vector3.up);
        Vector3 idealRRPose = RRjoint2.position + RRminima + Vector3.up * meanHeight;
        Vector3 RRdelta = Vector3.Project(idealRRPose - RRjoint2.position, Vector3.up);
        Vector3 idealBodyPose = transform.position + bodyMinima + Vector3.up * meanHeight;
        Vector3 bodyDelta = Vector3.Project(idealBodyPose - transform.position, Vector3.up);
        // Debug.DrawLine(FLjoint2.position + FLminima, idealFLPose, Color.black);
        // Debug.DrawLine(FRjoint2.position + FRminima, idealFRPose, Color.black);
        // Debug.DrawLine(RLjoint2.position + RLminima, idealRLPose, Color.black);
        // Debug.DrawLine(RRjoint2.position + RRminima, idealRRPose, Color.black);
        // Debug.DrawLine(transform.position + bodyMinima, idealBodyPose, Color.black);

        List<Vector3> deltas = new List<Vector3>{
            Vector3.Project(idealBodyPose - idealFLPose, Vector3.up),
            Vector3.Project(idealBodyPose - idealFRPose, Vector3.up),
            Vector3.Project(idealBodyPose - idealRLPose, Vector3.up),
            Vector3.Project(idealBodyPose - idealRRPose, Vector3.up)
        };
        float totalWeight = 0.0f;
        Vector3 sumDelta = Vector3.zero;
        foreach (var item in deltas)
            totalWeight += item.magnitude;
        if (totalWeight > 0.0f)
            foreach (var item in deltas)
                sumDelta += item.magnitude/totalWeight*item;
        // Debug.DrawRay(transform.position, sumDelta, Color.red);

        Vector3 shift = Vector3.Project(idealBodyPose - transform.position, Vector3.up) - sumDelta;
        return shift;
    }
    public bool PlaceFoot(){
        List<Tuple<Vector3, Vector3>> FLrayData = FilterDataByAngle(GetSphereData(FLjoint2.position, FLjoint2.rotation));
        List<Tuple<Vector3, Vector3>> FRrayData = FilterDataByAngle(GetSphereData(FRjoint2.position, FRjoint2.rotation));
        List<Tuple<Vector3, Vector3>> RLrayData = FilterDataByAngle(GetSphereData(RLjoint2.position, RLjoint2.rotation));
        List<Tuple<Vector3, Vector3>> RRrayData = FilterDataByAngle(GetSphereData(RRjoint2.position, RRjoint2.rotation));

        Vector3 minProjection = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
        int minFLid = -1;
        for (int id = 0; id < FLrayData.Count; ++id){
            if (Vector3.ProjectOnPlane(FLrayData[id].Item1, Vector3.up).magnitude < minProjection.magnitude){
                minFLid = id;
                minProjection = Vector3.ProjectOnPlane(FLrayData[id].Item1, Vector3.up);
            }
        }
        if (minFLid == -1)
            return false;
        minProjection = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
        int minFRid = -1;
        for (int id = 0; id < FRrayData.Count; ++id){
            if (Vector3.ProjectOnPlane(FRrayData[id].Item1, Vector3.up).magnitude < minProjection.magnitude){
                minFRid = id;
                minProjection = Vector3.ProjectOnPlane(FRrayData[id].Item1, Vector3.up);
            }
        }
        if (minFRid == -1)
            return false;
        minProjection = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
        int minRLid = -1;
        for (int id = 0; id < RLrayData.Count; ++id){
            if (Vector3.ProjectOnPlane(RLrayData[id].Item1, Vector3.up).magnitude < minProjection.magnitude){
                minRLid = id;
                minProjection = Vector3.ProjectOnPlane(RLrayData[id].Item1, Vector3.up);
            }
        }
        if (minRLid == -1)
            return false;
        minProjection = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
        int minRRid = -1;
        for (int id = 0; id < RRrayData.Count; ++id){
            if (Vector3.ProjectOnPlane(RRrayData[id].Item1, Vector3.up).magnitude < minProjection.magnitude){
                minRRid = id;
                minProjection = Vector3.ProjectOnPlane(RRrayData[id].Item1, Vector3.up);
            }
        }
        if (minRRid == -1)
            return false;
        Vector3 FLminima = FLrayData[minFLid].Item1;
        Vector3 FRminima = FRrayData[minFRid].Item1;
        Vector3 RLminima = RLrayData[minRLid].Item1;
        Vector3 RRminima = RRrayData[minRRid].Item1;
        Debug.DrawRay(FLjoint2.position, FLminima, Color.black);
        Debug.DrawRay(FRjoint2.position, FRminima, Color.black);
        Debug.DrawRay(RLjoint2.position, RLminima, Color.black);
        Debug.DrawRay(RRjoint2.position, RRminima, Color.black);

        Vector3 FLDirection = FLminima + FLjoint2.position - FLjoint1.position;
        Vector3 FRDirection = FRminima + FRjoint2.position - FRjoint1.position;
        Vector3 RLDirection = RLminima + RLjoint2.position - RLjoint1.position;
        Vector3 RRDirection = RRminima + RRjoint2.position - RRjoint1.position;
        
        Vector3 pos = transform.InverseTransformVector(FLDirection);
        List<float> anglesFL =  ResolveInverseKinematicsFL(pos);
        pos = transform.InverseTransformVector(FRDirection);
        List<float> anglesFR = ResolveInverseKinematicsFR(pos);
        pos = transform.InverseTransformVector(RLDirection);
        List<float> anglesRL = ResolveInverseKinematicsRL(pos);
        pos = transform.InverseTransformVector(RRDirection);
        List<float> anglesRR = ResolveInverseKinematicsRR(pos);
        // return true;
        return ApplyFLAngles(anglesFL) && ApplyFLAngles(anglesFR) && ApplyFLAngles(anglesRL) && ApplyFLAngles(anglesRR);
    }
    
    // private Vector3 GetBodyDisplacements(Quaternion rot){
        // Vector3 res = Vector3.zero;

        // List<Vector3> rayDirections = new List<Vector3>();
        // for (float angle = 0.0f; angle < 2.0f * Mathf.PI; angle += discretization){
        //     Quaternion yRotation = Quaternion.Euler(0.0f, Mathf.Rad2Deg * angle, 0.0f);
        //     rayDirections.Add(rot * yRotation * Vector3.forward * 0.3f);
        // }

        // List<Tuple<Vector3, Vector3>> rayHitData = new List<Tuple<Vector3, Vector3>>(rayDirections.Count);
        // foreach (Vector3 dir in rayDirections) {
        //     // Debug.DrawRay(pos, dir, Color.gray);
        //     RaycastHit hit;
        //     if (Physics.Raycast(pos, dir, out hit, dir.magnitude, ~0)){
        //         rayHitData.Add( new Tuple<Vector3, Vector3>(hit.point, hit.normal) );
        //     } else {
        //         rayHitData.Add( new Tuple<Vector3, Vector3>(Vector3.zero, Vector3.zero) );
        //     }
        // }

    //     List<Vector3> nonZeroDisplacements = new List<Vector3>();
    //     for (int id = 0; id < rayDirections.Count; ++id){
    //         Vector3 hitPoint = rayHitData[id].Item1;
    //         Vector3 hitNormal = rayHitData[id].Item2;
    //         Vector3 displacement = hitNormal * Mathf.Abs((pos - hitPoint).magnitude - rayDirections[id].magnitude);
    //         Debug.DrawRay(hitPoint, displacement, Color.white);
    //         if (displacement != Vector3.zero){
    //             nonZeroDisplacements.Add(displacement);
    //         }
    //     }

    //     if (nonZeroDisplacements.Count > 0){
    //         foreach (Vector3 v in nonZeroDisplacements)
    //             res += v;
    //         res /= nonZeroDisplacements.Count;
    //     }
        // return res;
    // }
}
