using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Valve.VR;
public class MainMenu : MonoBehaviour
{
    private bool menuIsActive = false;
    private Vector2 trackpadVector = Vector2.zero;
    private int menuStatus = ((int)MenuStatus.NOTHING);
    private GameObject mapPanel = null, navPanel = null;
    private VRUI vrui = null;
    private HelpBar helpBar = null;
    private Transform ActiveController;
    private LineRenderer lineRenderer = null;
    private enum MenuStatus : int
    {
        NOTHING,
        MAPPING, 
        NAVIGATION
    }
    void Awake() {
        // find objects of VRUI
        mapPanel = transform.GetChild(0).gameObject;
        navPanel = transform.GetChild(1).gameObject;
        vrui = transform.parent.GetComponent<VRUI>();
        helpBar = transform.parent.GetChild(transform.parent.childCount - 1).GetComponent<HelpBar>();
        
        lineRenderer = gameObject.GetComponent<LineRenderer>();
        ActiveController = transform.parent.parent.GetChild(1);
    }
    void Update() {
        lineRenderer.positionCount = 0;
        if (menuIsActive){
            mapPanel.SetActive(true);
            navPanel.SetActive(true);

            getMenuStatus(trackpadVector);
            HighlightSelected();
            
            helpBar.SetHelpText("Press trackpad to select option.\n Press trigger to teleportate.");
        } else {
            mapPanel.SetActive(false);
            navPanel.SetActive(false);
            Vector3 pose = GetPosition();
            helpBar.SetHelpText("Use trackpad to see Main Menu.\n Press trigger to teleportate.");
        }
    }
    private int getMenuStatus(Vector2 axis){
        float angle = Vector2.Angle(Vector2.right, axis) * Mathf.Sign(axis.y);
        float length = Vector2.SqrMagnitude(axis);
        menuStatus = ((int)MenuStatus.NOTHING);

        if (length > 0.5f){
            if (angle > 0.0f){
                menuStatus = ((int)MenuStatus.MAPPING);
            } else {
                menuStatus = ((int)MenuStatus.NAVIGATION);
            }
        } else if (length < 0.2f){
            menuStatus = ((int)MenuStatus.NOTHING);
        }
        return menuStatus;
    }
    private void HighlightSelected(){
        mapPanel.GetComponent<Image>().color = Color.white;
        navPanel.GetComponent<Image>().color = Color.white;    

        switch (menuStatus) {
            case ((int)MenuStatus.MAPPING):
                mapPanel.GetComponent<Image>().color = Color.green;
                break;
            case ((int)MenuStatus.NAVIGATION):
                navPanel.GetComponent<Image>().color = Color.green;
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
            case ((int)MenuStatus.MAPPING):
                vrui.SetActiveMappingMenu();
                break;
            case ((int)MenuStatus.NAVIGATION):
                vrui.SetActiveNavigationMenu();
                break;
            default:
                break;
        }
    }
    public void processTriggerPress(){
        Vector3 pose = GetPosition();
        transform.parent.parent.position = pose;
    }
}
