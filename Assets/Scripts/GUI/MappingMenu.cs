using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class MappingMenu : MonoBehaviour
{
    private bool menuIsActive = true;
    private Vector2 trackpadVector = Vector2.zero;
    private int menuStatus = ((int)MenuStatus.NOTHING);
    private GameObject makeMapPanel = null, saveMapPanel = null, resetPanel = null, currentMapPanel = null;
    private VRUI vrui = null;
    private HelpBar helpBar = null;
    
    [Header("External modules")]
    public Mapper mapper = null;
    private enum MenuStatus : int
    {
        NOTHING,
        MAKE_MAP, 
        SAVE_MAP,
        BACK
    }
    void Awake()
    {
        makeMapPanel = transform.GetChild(0).gameObject;
        saveMapPanel = transform.GetChild(1).gameObject;
        resetPanel = transform.GetChild(2).gameObject;
        currentMapPanel = transform.GetChild(3).gameObject;

        vrui = transform.parent.GetComponent<VRUI>();
        helpBar = transform.parent.GetChild(transform.parent.childCount - 1).GetComponent<HelpBar>();
    }

    // Update is called once per frame
    void Update()
    {
        if (menuIsActive){
            makeMapPanel.SetActive(true);
            saveMapPanel.SetActive(true);
            resetPanel.SetActive(true);
            currentMapPanel.SetActive(true);

            getMenuStatus(trackpadVector);
            HighlightSelected();
            helpBar.SetHelpText("Press trackpad to select option.");
        } else {
            makeMapPanel.SetActive(false);
            saveMapPanel.SetActive(false);
            resetPanel.SetActive(false);
            currentMapPanel.SetActive(false);

            helpBar.SetHelpText("Use trackpad to see Mapping Menu");
        }
    }
    private int getMenuStatus(Vector2 axis){
        float angle = Vector2.Angle(Vector2.right, axis) * Mathf.Sign(axis.y);
        float length = Vector2.SqrMagnitude(axis);
        menuStatus = ((int)MenuStatus.NOTHING);

        if (length > 0.5f){
            if ( Mathf.Abs(angle) > 135.0f ){
                menuStatus = ((int)MenuStatus.MAKE_MAP);
            } else if ( Mathf.Abs(angle) < 45.0f ){
                menuStatus = ((int)MenuStatus.SAVE_MAP);
            }
        } else if (length < 0.2f){
            menuStatus = ((int)MenuStatus.BACK);
        }
        return menuStatus;
    }
    private void HighlightSelected(){
        makeMapPanel.GetComponent<Image>().color = Color.white;
        saveMapPanel.GetComponent<Image>().color = Color.white;    
        resetPanel.GetComponent<Image>().color = Color.white;
        // Switch color for selected menu option
        switch (menuStatus) {
            case ((int)MenuStatus.MAKE_MAP):
                makeMapPanel.GetComponent<Image>().color = Color.green;
                break;
            case ((int)MenuStatus.SAVE_MAP):
                saveMapPanel.GetComponent<Image>().color = Color.green;  
                break;
            case ((int)MenuStatus.BACK):
                resetPanel.GetComponent<Image>().color = Color.green;
                break;
            default:
                break;
        }
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
            case ((int)MenuStatus.BACK):
                vrui.SetActiveMainMenu();
                break;
            default:
                break;
        }
    }
}
