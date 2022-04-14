using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class MappingMenu : MonoBehaviour
{
    private bool menuIsActive = true;
    private Vector2 trackpadVector = Vector2.zero;
    private int menuStatus = ((int)MenuStatus.NOTHING);
    private GameObject makeMapPanel = null, saveMapPanel = null, resetPanel = null, currentMapPanel = null, addObstaclePanel = null, showMapPanel = null;
    private bool isSettingObstacle = false, isSettingRotation = false;
    private VRUI vrui = null;
    private HelpBar helpBar = null;
    private enum MenuStatus : int {
        NOTHING,
        MAKE_MAP, 
        SAVE_MAP,
        SHOW_MAP,
        ADD_OBSTACLE,
        BACK
    }
    private LineRenderer lineRenderer = null;
    private Transform ActiveController;

    [Header("External modules")]
    public Mapper mapper = null;
    void Awake()
    {
        makeMapPanel = transform.GetChild(0).gameObject;
        saveMapPanel = transform.GetChild(1).gameObject;
        addObstaclePanel = transform.GetChild(2).gameObject;
        currentMapPanel = transform.GetChild(3).gameObject;
        showMapPanel = transform.GetChild(4).gameObject;
        resetPanel = transform.GetChild(transform.childCount - 1).gameObject;

        ActiveController = transform.parent.parent.GetChild(1);
        vrui = transform.parent.GetComponent<VRUI>();
        helpBar = transform.parent.GetChild(transform.parent.childCount - 1).GetComponent<HelpBar>();
        lineRenderer = transform.GetComponent<LineRenderer>();
    }
    void Update() {
        lineRenderer.positionCount = 0;
        if (menuIsActive && !(isSettingObstacle)){
            ShowMenu(true);
            
            getMenuStatus(trackpadVector);
            HighlightSelected();
            helpBar.SetHelpText("Press trackpad to select option.");
        } else {
            ShowMenu(false);

            helpBar.SetHelpText("Use trackpad to see Mapping Menu");
        }
        // Process setting obstacle states
        if (isSettingObstacle){
            Transform obstacleState = mapper.GetObstacleState();
            string prefixStr;
            if (isSettingRotation){
                obstacleState.rotation = Quaternion.Euler(GetRotationEuler());
                prefixStr = "Press trigger to finish.";
            } else {
                obstacleState.position = GetPosition();
                prefixStr = "Press trigger to set rotation.";
            }
            string data = GetHelpText(obstacleState.position, obstacleState.rotation.eulerAngles);
            helpBar.SetHelpText(prefixStr + "\n" + data, 20);
        }
    }    
    private void ShowMenu(bool isShow){
        makeMapPanel.SetActive(isShow);
        saveMapPanel.SetActive(isShow);
        currentMapPanel.SetActive(isShow);
        addObstaclePanel.SetActive(isShow);
        showMapPanel.SetActive(isShow);
        currentMapPanel.SetActive(isShow);
        resetPanel.SetActive(isShow);
    }
    private int getMenuStatus(Vector2 axis){
        float angle = Vector2.Angle(Vector2.right, axis) * Mathf.Sign(axis.y);
        float length = Vector2.SqrMagnitude(axis);
        menuStatus = ((int)MenuStatus.NOTHING);

        if (length > 0.5f){
            if ( Mathf.Abs(angle) > 135.0f ){
                menuStatus = ((int)MenuStatus.MAKE_MAP);
            } else if (angle > 45){
                menuStatus = ((int)MenuStatus.NOTHING);
            } else if (angle > 0.0f){
                menuStatus = ((int)MenuStatus.SAVE_MAP);
            } else if (angle > -45.0f){
                menuStatus = ((int)MenuStatus.SHOW_MAP);
            } else if (angle > -135.0f) {
                menuStatus = ((int)MenuStatus.ADD_OBSTACLE);
            } else {
                menuStatus = ((int)MenuStatus.NOTHING);
            }
        } else if (length < 0.2f){
            menuStatus = ((int)MenuStatus.BACK);
        }
        return menuStatus;
    }
    private void HighlightSelected(){
        makeMapPanel.GetComponent<Image>().color = Color.white;
        saveMapPanel.GetComponent<Image>().color = Color.white;    
        addObstaclePanel.GetComponent<Image>().color = Color.white;
        showMapPanel.GetComponent<Image>().color = Color.white;
        resetPanel.GetComponent<Image>().color = Color.white;
        // Switch color for selected menu option
        switch (menuStatus) {
            case ((int)MenuStatus.MAKE_MAP):
                makeMapPanel.GetComponent<Image>().color = Color.green;
                break;
            case ((int)MenuStatus.SAVE_MAP):
                saveMapPanel.GetComponent<Image>().color = Color.green;  
                break;
            case ((int)MenuStatus.ADD_OBSTACLE):
                addObstaclePanel.GetComponent<Image>().color = Color.green;
                break;
            case ((int)MenuStatus.BACK):
                resetPanel.GetComponent<Image>().color = Color.green;
                break;
            case ((int)MenuStatus.SHOW_MAP):
                showMapPanel.GetComponent<Image>().color = Color.green;
                break;
            default:
                break;
        }
    }
    private Vector3 GetPosition(){        
        Debug.DrawRay(ActiveController.position, 
            ActiveController.TransformDirection(Vector3.forward) * 100,
            Color.yellow);
            
        lineRenderer = gameObject.GetComponent<LineRenderer>();
        RaycastHit hit;
        if (Physics.Raycast(ActiveController.position, 
            ActiveController.TransformDirection(Vector3.forward), 
            out hit, Mathf.Infinity, 7)){
                lineRenderer.positionCount = 2;
                lineRenderer.SetPosition(0, ActiveController.position);
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
        if (!menuIsActive){
            return;
        }
        switch (menuStatus) {
            case ((int)MenuStatus.MAKE_MAP):
                mapper.MakeMap();
                break;
            case ((int)MenuStatus.SAVE_MAP):
                mapper.SaveMap();
                break;
            case ((int)MenuStatus.ADD_OBSTACLE):
                isSettingObstacle = true;
                break;
            case ((int)MenuStatus.BACK):
                vrui.SetActiveMainMenu();
                break;
            case ((int)MenuStatus.SHOW_MAP):
                mapper.ShowMap();
                break;
            default:
                break;
        }
    }
    public void processTriggerPress(){
        if (isSettingObstacle){
            if (isSettingRotation){
                isSettingObstacle = false;
                isSettingRotation = false;
                mapper.GenerateObstacle();
            } else {
                isSettingRotation = true;
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
