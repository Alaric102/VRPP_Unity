using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Robot : MonoBehaviour
{
    // Common parameters
    private Transform body;
    private float lengthJoint1, lengthJoint2, lengthJoint3;
    public List<float> maxAngles = new List<float>{90.0f, 180.0f, 180.0f};
    public List<float> minAngles = new List<float>{-90.0f, -180.0f, -180.0f};
    private float minDistance = 0.1053526f, maxDistance = 0.3111828f;
    // FL leg variables
    private Transform FLjoint1, FLjoint2, FLjoint3, FLEndEffector;
    public List<float> FLAngles = new List<float>{0.0f, 0.0f, 0.0f};
    public Vector3 relativeFLPosition = Vector3.zero;
    // FR leg variables
    private Transform FRjoint1, FRjoint2, FRjoint3, FREndEffector;
    public List<float> FRAngles = new List<float>{0.0f, 0.0f, 0.0f};
    public Vector3 relativeFRPosition = Vector3.zero;
    // RL leg variables
    private Transform RLjoint1, RLjoint2, RLjoint3, RLEndEffector;
    public List<float> RLAngles = new List<float>{0.0f, 0.0f, 0.0f};
    public Vector3 relativeRLPosition = Vector3.zero;
    // RR leg variables
    private Transform RRjoint1, RRjoint2, RRjoint3, RREndEffector;
    public List<float> RRAngles = new List<float>{0.0f, 0.0f, 0.0f};
    public Vector3 relativeRRPosition = Vector3.zero;
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
    private void ForwardKinematics(){
        FLjoint1.localRotation = Quaternion.Euler(FLAngles[0], 0, 0);
        FLjoint2.localRotation = Quaternion.Euler(0, FLAngles[1], 0);
        FLjoint3.localRotation = Quaternion.Euler(0, FLAngles[2], 0);
        
        FRjoint1.localRotation = Quaternion.Euler(FRAngles[0], 0, 0);
        FRjoint2.localRotation = Quaternion.Euler(0, FRAngles[1], 0);
        FRjoint3.localRotation = Quaternion.Euler(0, FRAngles[2], 0);

        RLjoint1.localRotation = Quaternion.Euler(RLAngles[0], 0, 0);
        RLjoint2.localRotation = Quaternion.Euler(0, RLAngles[1], 0);
        RLjoint3.localRotation = Quaternion.Euler(0, RLAngles[2], 0);
        
        RRjoint1.localRotation = Quaternion.Euler(RRAngles[0], 0, 0);
        RRjoint2.localRotation = Quaternion.Euler(0, RRAngles[1], 0);
        RRjoint3.localRotation = Quaternion.Euler(0, RRAngles[2], 0);
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
    private void InverseKinematics(){
        FLAngles[0] = GetJoint1Angle_LeftLegs(relativeFLPosition);
        RLAngles[0] = GetJoint1Angle_LeftLegs(relativeRLPosition);
        FRAngles[0] = GetJoint1Angle_RightLegs(relativeFRPosition);
        RRAngles[0] = GetJoint1Angle_RightLegs(relativeRRPosition);
        
        FLAngles[2] = GetJoint3Angle(relativeFLPosition);
        RLAngles[2] = GetJoint3Angle(relativeRLPosition);
        FRAngles[2] = GetJoint3Angle(relativeFRPosition);
        RRAngles[2] = GetJoint3Angle(relativeRRPosition);

        FLAngles[1] = GetJoint2Angle(relativeFLPosition);
        RLAngles[1] = GetJoint2Angle(relativeRLPosition);
        FRAngles[1] = GetJoint2Angle(relativeFRPosition);
        RRAngles[1] = GetJoint2Angle(relativeRRPosition);
    }
}
