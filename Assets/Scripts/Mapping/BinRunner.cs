using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BinRunner : MonoBehaviour {
    public int objectLimit = 1000;
    private Transform gridRunner;
    private Vector3 minCorner_ = Vector3.zero, maxCorner_ = Vector3.zero;
    private int currentLevel, maxLevel;
    private int toGenerate = -1;
    private bool isObstacle = false;
    private VoxelMap voxelMap = null;
    private float spawnDelay = 0.005f;
    private float spawnTimer = 1.0f;
    private float lifeTimer = 0.05f;
    void Awake(){
        gridRunner = transform.parent.GetChild(1);
        voxelMap =  transform.parent.GetComponent<VoxelMap>();
    }
    void Start() {
        gameObject.GetComponent<Collider>().enabled = true;
        spawnDelay = 0.01f*currentLevel;
        spawnTimer = spawnDelay;
        objectLimit = ((int)(500*(currentLevel + 1) / (maxLevel + 1)));
    }

    void Update() {
        if (toGenerate > 0){
            if (transform.parent.childCount > objectLimit){
                objectLimit += toGenerate;
                return;
            } else if (spawnTimer  < 0.0f) {
                int x = (((toGenerate - 1) >> 0) & 1)*2-1;
                int y = (((toGenerate - 1) >> 1) & 1)*2-1;
                int z = (((toGenerate - 1) >> 2) & 1)*2-1;
                
                Vector3 scale = (maxCorner_ - minCorner_)/2.0f;
                Vector3 pose = transform.position - Vector3.Scale(scale / 2.0f, new Vector3(x, y, z));
                Vector3 min = minCorner_ + Vector3.Scale(scale, new Vector3(x+1, y+1, z+1));
                Vector3 max = min + scale;
                Transform newObj = Instantiate(gridRunner, pose, Quaternion.identity, transform.parent);
                newObj.GetComponent<BinRunner>().Init(min, max, currentLevel + 1, maxLevel);
                newObj.GetComponent<Collider>().enabled = false;
                newObj.gameObject.SetActive(true);

                toGenerate--;
                spawnTimer  = spawnDelay;
            }
            spawnTimer  -= Time.deltaTime;
        } else if (toGenerate == 0){ // Case after generating 8 chunks
            Destroy(gameObject);
        } else { // No collision case
            lifeTimer -= Time.deltaTime;
            if (lifeTimer < 0.0f) 
                Destroy(gameObject);
        }
    }
    void OnTriggerEnter(Collider other) {
        if (other.gameObject.tag != "CellBinary"){
            if (currentLevel < maxLevel){   // proceed chunk generation
                toGenerate = 8;
            } else {                        // define is obstacle
                isObstacle = true;
                Destroy(gameObject);
            }
            gameObject.GetComponent<Collider>().enabled = false;
        } else {
            return;
        }
    }
    public bool Init(Vector3 min, Vector3 max, int lvl, int maxLvl){
        // Define scale by min/max range
        minCorner_ = min; maxCorner_ = max;
        transform.localScale = transform.parent.InverseTransformVector(max - min);
        
        if (lvl > maxLvl){
            Debug.Log("Invalid lvl: " + lvl + " > " + maxLevel);
            return false;
        }
        currentLevel = lvl;
        maxLevel = maxLvl;
        
        return true;
    }

    void OnDestroy(){ // On destroy we want to notify voxel map about collision if we reached max level
        if (currentLevel == maxLevel){
            Vector3 globalMinCorner = transform.parent.GetComponent<Mapper>().GetMinCorner();
            Vector3 globalPose = transform.position;
            Vector3 unsignedGlobalPose = globalPose + transform.lossyScale / 2.0f - globalMinCorner;
            // Debug.DrawLine(globalMinCorner, globalPose);
            // Debug.DrawRay(globalPose, transform.lossyScale / 2.0f, Color.black);

            Vector3Int voxelCellPose = new Vector3Int(
                ((int)Mathf.Round(unsignedGlobalPose.x/transform.lossyScale.x)) - 1,
                ((int)Mathf.Round(unsignedGlobalPose.y/transform.lossyScale.y)) - 1,
                ((int)Mathf.Round(unsignedGlobalPose.z/transform.lossyScale.z)) - 1
            );
            if (isObstacle){
                // Debug.Log("globalPose: " + GetVectorString(globalPose) + "\n" +
                //     "unsignedGlobalPose: " + GetVectorString(unsignedGlobalPose) + "\n" +
                //     "voxelCellPose: " + voxelCellPose);
                voxelMap.SetObstacleCell(voxelCellPose, globalPose);
            }
        }
    }
    private string GetVectorString(Vector3 v){ return v.x + ", " + v.y + ", " + v.z; }
}
