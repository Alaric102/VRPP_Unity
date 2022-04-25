using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Valve.VR;


public class NavigationMenu : MonoBehaviour {
    private bool menuIsActive = true;
    private Vector2 trackpadVector = Vector2.zero;
    private int menuStatus = ((int)MenuStatus.NOTHING);
    private bool isSettingStartState = false, isSettingGoalState = false, isSettingRotation = false;
    private bool isEditingPath = false, isMovingPathState = false;
    private GameObject setStartStatePanel = null, setGoalStatePanel = null, startPlanningPanel = null, editPathPanel = null, resetPanel = null;
    private VRUI vrui = null;
    private HelpBar helpBar = null;
    private LineRenderer lineRenderer = null;
    private enum MenuStatus : int {
        NOTHING,
        SET_START, 
        SET_GOAL,
        START_PLAN,
        EDIT_PATH,
        BACK
    } 
    [Header("External modules")]
    private Transform ActiveController;
    public Navigation navigation = null;
    public Transform editRegionPrefab;
    private Transform editRegion;
    private Vector3 movingPose = Vector3.zero;
    private Vector3 newPoseState = Vector3.zero;
    void Awake() {
        // find child objects of VR GUI
        setStartStatePanel = transform.GetChild(0).gameObject;
        setGoalStatePanel = transform.GetChild(1).gameObject;
        startPlanningPanel = transform.GetChild(2).gameObject;
        editPathPanel = transform.GetChild(3).gameObject;
        resetPanel = transform.GetChild(transform.childCount-1).gameObject;
        // find external objects of VR GUI
        vrui = transform.parent.GetComponent<VRUI>();
        helpBar = transform.parent.GetChild(transform.parent.childCount - 1).GetComponent<HelpBar>();

        ActiveController = transform.parent.parent.GetChild(1);
        lineRenderer = gameObject.GetComponent<LineRenderer>();
        editRegion = Instantiate(editRegionPrefab, Vector3.zero, Quaternion.identity);
        editRegion.gameObject.SetActive(false);
    }
    void Start(){
        // Set default values
        navigation.SetActiveStates(false);
        lineRenderer.positionCount = 0;
    }
    void Update() {
        lineRenderer.positionCount = 0;
        // show menu if menu is active and not setting start/goal states or editting path/obstacles
        if (menuIsActive && !(isSettingStartState || isSettingGoalState || isEditingPath)){
            ShowMenu(true);
            GetMenuStatus(trackpadVector);
            HighlightSelected();
            helpBar.SetHelpText("Press trackpad to select option.");
        } else {
            // else hide menu panels
            ShowMenu(false);
            helpBar.SetHelpText("Use trackpad to see Navigation Menu");
        }
        // Process setting start states
        if(isSettingStartState){
            Transform startState = navigation.GetStartState();
            string prefixStr;
            if (isSettingRotation){
                startState.rotation = Quaternion.Euler(GetRotationEuler());
                prefixStr = "Press trigger to finish.";
            } else {
                startState.position = GetPosition();
                prefixStr = "Press trigger to set rotation.";
            }
            string data = GetHelpText(startState.position, startState.rotation.eulerAngles);
            helpBar.SetHelpText(prefixStr + "\n" + data, 20);
        }
        // Process setting goal states
        if(isSettingGoalState){
            Transform goalState = navigation.GetGoalState();
            string prefixStr;
            if (isSettingRotation){
                goalState.rotation = Quaternion.Euler(GetRotationEuler());
                prefixStr = "Press trigger to finish.";
            } else {
                goalState.position = GetPosition() + Vector3.up * 0.2f;
                prefixStr = "Press trigger to set rotation.";
            }
            string data = GetHelpText(goalState.position, goalState.rotation.eulerAngles);
            helpBar.SetHelpText(prefixStr + "\n" + data, 20);
        }
        // Process editting path states
        if (isEditingPath){
            navigation.StopPlanning();
            Vector3 directionRay = GetPosition();
            // get list of all gPlan states (acheieved and toacheive)
            List<Vector3> globalPlan = navigation.GetGlobalPlan();
            if (globalPlan.Count == 0){
                isEditingPath = false;
                return;
            }
            // enable sphere for edit
            editRegion.gameObject.SetActive(true);
            if (isMovingPathState){
                newPoseState = directionRay;
                editRegion.position = newPoseState;
            } else { // selection of globaPose
                float minDistance = float.MaxValue;
                foreach (var pose in globalPlan) {
                    if ((pose - directionRay).magnitude < minDistance && (pose - directionRay).magnitude < 0.3f){
                        minDistance = (pose - directionRay).magnitude;
                        movingPose = pose;
                    }
                }
                // if (movingPose >= 0)
                editRegion.position = movingPose;
            }
        } else {
            editRegion.gameObject.SetActive(false);
        }
    }
    private void ShowMenu(bool isShow){ // Show menu's objects if isShow, otherwise hide
        setStartStatePanel.SetActive(isShow);
        setGoalStatePanel.SetActive(isShow);
        startPlanningPanel.SetActive(isShow);
        editPathPanel.SetActive(isShow);
        resetPanel.SetActive(isShow);

        // editRegion.gameObject.SetActive(isShow);
    }
    private int GetMenuStatus(Vector2 axis){
        float angle = Vector2.Angle(Vector2.right, axis) * Mathf.Sign(axis.y);
        float length = Vector2.SqrMagnitude(axis);
        menuStatus = ((int)MenuStatus.NOTHING);

        if (length > 0.5f){
            if (angle > 120.0f){
                menuStatus = ((int)MenuStatus.SET_START);
            } else if (angle > 60.0f){
                menuStatus = ((int)MenuStatus.NOTHING);
            } else if (angle > 0.0f){
                menuStatus = ((int)MenuStatus.NOTHING);
            } else if (angle > -60.0f) {
                menuStatus = ((int)MenuStatus.EDIT_PATH);
            } else if (angle > -120.0f){
                menuStatus = ((int)MenuStatus.START_PLAN);
            } else {
                menuStatus = ((int)MenuStatus.SET_GOAL);
            }
        } else if (length < 0.2f){
            menuStatus = ((int)MenuStatus.BACK);
        }
        return menuStatus;
    }
    private void HighlightSelected(){
        setStartStatePanel.GetComponent<Image>().color = Color.white;
        setGoalStatePanel.GetComponent<Image>().color = Color.white;    
        startPlanningPanel.GetComponent<Image>().color = Color.white;  
        editPathPanel.GetComponent<Image>().color = Color.white;  
        resetPanel.GetComponent<Image>().color = Color.white;
        // Switch color for selected menu option
        switch (menuStatus) {
            case ((int)MenuStatus.SET_START):
                setStartStatePanel.GetComponent<Image>().color = Color.green;
                break;
            case ((int)MenuStatus.SET_GOAL):
                setGoalStatePanel.GetComponent<Image>().color = Color.green;  
                break;
            case ((int)MenuStatus.START_PLAN):
                startPlanningPanel.GetComponent<Image>().color = Color.green;
                break;
            case ((int)MenuStatus.EDIT_PATH):
                editPathPanel.GetComponent<Image>().color = Color.green;
                break;
            case ((int)MenuStatus.BACK):
                resetPanel.GetComponent<Image>().color = Color.green;
                break;
            default:
                break;
        }
    }
    private Vector3 GetPosition(){        
        Debug.DrawRay(ActiveController.transform.position, 
            ActiveController.transform.TransformDirection(Vector3.forward) * 100,
            Color.yellow);
            
        lineRenderer = gameObject.GetComponent<LineRenderer>();
        RaycastHit hit;
        if (Physics.Raycast(ActiveController.transform.position, 
            ActiveController.transform.TransformDirection(Vector3.forward), 
            out hit, Mathf.Infinity, 7)){
                lineRenderer.positionCount = 2;
                lineRenderer.SetPosition(0, ActiveController.transform.position);
                lineRenderer.SetPosition(1, hit.point);
                return hit.point;
        } else {
            lineRenderer.positionCount = 0;
        }
        return Vector3.zero;
    }
    private Vector3 GetRotationEuler(){
        Vector3 rot =  ActiveController.transform.rotation.eulerAngles;
        return wrapAngle(rot);
    }
    private string GetHelpText(Vector3 pos, Vector3 rot){
        string res = 
            "Position x: " + Mathf.Round(pos.x).ToString() +
            ", y: " + Mathf.Round(pos.y).ToString() +
            ", z: " + Mathf.Round(pos.z).ToString() +
            "Angle: x: " + Mathf.Round(rot.x).ToString() +
            ", y: " + Mathf.Round(rot.y).ToString() +
            ", z: " + Mathf.Round(rot.z).ToString();
        return res;
    }
    public void processTouchTrackpad(bool newState){
        menuIsActive = newState;
    }
    public void processTrackpadPosition(Vector2 axis){
        trackpadVector = axis;
    }
    public void processTrackpadPressRelease(){
        if (!menuIsActive)
            return;
        switch (menuStatus) {
            case ((int)MenuStatus.SET_START):
                // isSettingStartState = true;
                break;
            case ((int)MenuStatus.SET_GOAL):
                isSettingGoalState = true;
                break;
            case ((int)MenuStatus.START_PLAN):
                navigation.StartPlanning();
                break;
            case ((int)MenuStatus.EDIT_PATH):
                isEditingPath = true;
                break;
            case ((int)MenuStatus.BACK):
                // Hide Robots
                navigation.SetActiveStates(false);
                vrui.SetActiveMainMenu();
                break;
            default:
                break;
        }
    }
    public void processTriggerPress(){
        if (isSettingStartState){
            if (isSettingRotation){
                isSettingStartState = false;
                isSettingRotation = false;
            } else {
                isSettingRotation = true;
            }
        }
        if (isSettingGoalState){
            if (isSettingRotation){
                isSettingGoalState = false;
                isSettingRotation = false;
                navigation.AddGoalState();
            } else {
                isSettingRotation = true;
            }
        }
        if (isEditingPath){
            if (isMovingPathState){
                isMovingPathState = false;
                isEditingPath = false;
                editRegion.gameObject.SetActive(false);
                navigation.Replan(editRegion.position, movingPose);
            } else {
                isMovingPathState = true;
            }
        }
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
