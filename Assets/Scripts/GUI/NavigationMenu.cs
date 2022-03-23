using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Valve.VR;


public class NavigationMenu : MonoBehaviour
{
    private bool menuIsActive = true;
    private Vector2 trackpadVector = Vector2.zero;
    private int menuStatus = ((int)MenuStatus.NOTHING);
    private bool isSettingStartPosition = false, isSettingGoalPosition = false, isSettingRotation = false;
    private GameObject setStartStatePanel = null, setGoalStatePanel = null, startPlanningPanel = null, resetPanel = null;
    private VRUI vrui = null;
    private HelpBar helpBar = null;
    private LineRenderer lineRenderer = null;
    private Vector3 setStatePosition, setStateRotation;
    private enum MenuStatus : int {
        NOTHING,
        SET_START, 
        SET_GOAL,
        START_PLAN,
        BACK
    } 

    [Header("External modules")]
    public GameObject ActiveController;
    public Navigation navigation = null;
    void Awake() {
        // find objects of VR GUI
        setStartStatePanel = transform.GetChild(0).gameObject;
        setGoalStatePanel = transform.GetChild(1).gameObject;
        startPlanningPanel = transform.GetChild(2).gameObject;
        resetPanel = transform.GetChild(3).gameObject;
        vrui = transform.parent.GetComponent<VRUI>();
        helpBar = transform.parent.GetChild(transform.parent.childCount - 1).GetComponent<HelpBar>();
        lineRenderer = gameObject.GetComponent<LineRenderer>();
    }

    void Start(){
        // Hide Robots
        navigation.SetActiveStates(false);
        // clean lineRenderer
        lineRenderer.positionCount = 0;
        // set default values
        setStatePosition = Vector3.zero;
        setStateRotation = Vector3.zero;
    }
    void Update() {
        lineRenderer.positionCount = 0;
        // Show robots if menu is active
        navigation.SetActiveStates(menuIsActive);

        if (menuIsActive && !(isSettingStartPosition || isSettingGoalPosition)){
            // show menu if menu is active and not setting start/goal states
            setStartStatePanel.SetActive(true);
            setGoalStatePanel.SetActive(true);
            startPlanningPanel.SetActive(true);
            resetPanel.SetActive(true);

            getMenuStatus(trackpadVector);
            HighlightSelected();
            helpBar.SetHelpText("Press trackpad to select option.");
        } else {
            // else hide menu panels
            setStartStatePanel.SetActive(false);
            setGoalStatePanel.SetActive(false);
            startPlanningPanel.SetActive(false);
            resetPanel.SetActive(false);
            
            helpBar.SetHelpText("Use trackpad to see Navigation Menu");
        }

        if(isSettingStartPosition || isSettingGoalPosition){
            // if setting start/goal states
            string prefixStr;
            if (isSettingRotation){
                // if setting rotation
                setStateRotation = GetRotationEuler();
                prefixStr = "Press trigger to finish.";
            } else {
                // if setting position
                setStatePosition = GetPosition();
                prefixStr = "Press trigger to set rotation.";
            }
            string data = GetHelpText(setStatePosition, setStateRotation);
            helpBar.SetHelpText(prefixStr + "\n" + data, 20);

            // update states in navigation module
            if (isSettingStartPosition){
                navigation.SetStartState(ref setStatePosition, ref setStateRotation);
            } else if (isSettingGoalPosition){
                navigation.SetGoalState(ref setStatePosition, ref setStateRotation);
            }
        }
    }
    private int getMenuStatus(Vector2 axis){
        float angle = Vector2.Angle(Vector2.right, axis) * Mathf.Sign(axis.y);
        float length = Vector2.SqrMagnitude(axis);
        menuStatus = ((int)MenuStatus.NOTHING);

        if (length > 0.5f){
            if (angle > 120.0f){
                menuStatus = ((int)MenuStatus.SET_START);
            } else if (angle > -60.0f){
                menuStatus = ((int)MenuStatus.NOTHING);
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
                isSettingStartPosition = true;
                break;
            case ((int)MenuStatus.SET_GOAL):
                isSettingGoalPosition = true;
                break;
            case ((int)MenuStatus.START_PLAN):
                navigation.StartPlanning();
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
        if (isSettingStartPosition || isSettingGoalPosition){
            if (isSettingRotation){
                isSettingStartPosition = false;
                isSettingGoalPosition = false;
                isSettingRotation = false;
            } else {
                isSettingRotation = true;
            }
        }
    }
    private Vector3 wrapAngle(Vector3 r){
        Vector3 res = new Vector3(-180.0f, -180.0f, -180.0f);
        res += r;
        if (res.x > 0.0f){
            res.x -= 180.0f;
        } else {
            res.x += 180.0f;
        }

        if (res.y > 0.0f){
            res.y -= 180.0f;
        } else {
            res.y += 180.0f;
        }

        if (res.z > 0.0f){
            res.z -= 180.0f;
        } else {
            res.z += 180.0f;
        }
        return res;
    }
}
