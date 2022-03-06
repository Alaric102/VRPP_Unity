using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
public class MainMenu : MonoBehaviour
{
    private GameObject mapPanel = null, navPanel = null;
    private enum MainMenuStatus : int
    {
        NOTHING,
        MAPPING, 
        NAVIGATION
    }
    private int menuMod = ((int)MainMenuStatus.NOTHING);
    void Awake()
    {
        mapPanel = transform.GetChild(0).gameObject;
        navPanel = transform.GetChild(1).gameObject;
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    public void Proccess(Vector2 axis){
        float angle = Vector2.Angle(Vector2.up, axis);
        float length = Vector2.SqrMagnitude(axis);
        
        // Set default parameters
        menuMod = ((int)MainMenuStatus.NOTHING);
        mapPanel.GetComponent<Image>().color = Color.white;
        navPanel.GetComponent<Image>().color = Color.white;
        
        // Highlight menu selection
        if (length > 0.25f){
            if (angle < 90.0f){
                mapPanel.GetComponent<Image>().color = Color.green;
                menuMod = ((int)MainMenuStatus.MAPPING);
            } else {
                navPanel.GetComponent<Image>().color = Color.green;
                menuMod = ((int)MainMenuStatus.NAVIGATION);
            }
        }
    }

    public void ProccessTrigger(){
        switch (menuMod) {
            case ((int)MainMenuStatus.NOTHING):
                Debug.Log("Nothing");
                break;
            case ((int)MainMenuStatus.MAPPING):
                Debug.Log("Go to mapping (2)");
                transform.parent.GetComponent<VRUI>().SetMenuStatus(2);
                break;
            case ((int)MainMenuStatus.NAVIGATION):
                Debug.Log("Go to navigation (3)");
                transform.parent.GetComponent<VRUI>().SetMenuStatus(3);
                break;
            default:
                Debug.Log("Main menu can't recognize trigger.");
                break;
        }
    }

}
