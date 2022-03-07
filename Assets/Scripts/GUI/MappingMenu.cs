using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class MappingMenu : MonoBehaviour
{
    private GameObject makeMapPanel = null, saveMapPanel = null, resetPanel = null;
    public GameObject mapper;
    private enum MainMenuStatus : int
    {
        NOTHING,
        MAKE_MAP, 
        SAVE_MAP,
        BACK
    }
    private int menuMod = ((int)MainMenuStatus.NOTHING);
    void Awake()
    {
        makeMapPanel = transform.GetChild(0).gameObject;
        saveMapPanel = transform.GetChild(1).gameObject;
        resetPanel = transform.GetChild(2).gameObject;
    }

    // Update is called once per frame
    void Update()
    {
        menuMod = ((int)MainMenuStatus.NOTHING);
        resetPanel.GetComponent<Image>().color = Color.white;
        makeMapPanel.GetComponent<Image>().color = Color.white;
        saveMapPanel.GetComponent<Image>().color = Color.white;
    }
    public void Proccess(Vector2 axis){
        float angle = Vector2.Angle(Vector2.right, axis);
        float length = Vector2.SqrMagnitude(axis);
        
        // Set default parameters
        menuMod = ((int)MainMenuStatus.NOTHING);
        resetPanel.GetComponent<Image>().color = Color.white;
        makeMapPanel.GetComponent<Image>().color = Color.white;
        saveMapPanel.GetComponent<Image>().color = Color.white;
        
        // Highlight menu selection
        if (length > 0.25f){
            if (angle > 90.0f){
                makeMapPanel.GetComponent<Image>().color = Color.green;
                menuMod = ((int)MainMenuStatus.MAKE_MAP);
            } else {
                saveMapPanel.GetComponent<Image>().color = Color.green;
                menuMod = ((int)MainMenuStatus.SAVE_MAP);
            }
        } else if (length < 0.15f){
            resetPanel.GetComponent<Image>().color = Color.green;
            menuMod = ((int)MainMenuStatus.BACK);
        }
    }
    public void ProccessTrigger(){
        switch (menuMod) {
            case ((int)MainMenuStatus.NOTHING):
                Debug.Log("Nothing");
                break;
            case ((int)MainMenuStatus.MAKE_MAP):
                Debug.Log("Make Map");
                mapper.GetComponent<Mapper>().MakeMap();
                break;
            case ((int)MainMenuStatus.SAVE_MAP):
                Debug.Log("Save map");
                mapper.GetComponent<Mapper>().SaveMap();
                break;
            case ((int)MainMenuStatus.BACK):
                Debug.Log("Go to main (1)");
                // transform.parent.GetComponent<VRUI>().SetMenuStatus(1);
                break;
            default:
                Debug.Log("Main menu can't recognize trigger.");
                break;
        }
    }
}
