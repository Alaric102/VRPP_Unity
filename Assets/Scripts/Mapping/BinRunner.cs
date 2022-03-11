using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class BinRunner : MonoBehaviour
{
    public Transform GridRunner;
    private Vector3 minCorner = new Vector3(-5.0f, -5.0f, -5.0f);
    private Vector3 maxCorner = new Vector3(5.0f, 5.0f, 5.0f);
    private int maxLevel;
    private int currentLevel;
    private float spawnDelay = 0.005f;
    private float spawnTimer = 1.0f;
    private float lifeTimer = 0.05f;
    private float timeDestroyLeft = 0.0f;
    private int toGenerate =-1;
    public int objectLimit = 1000;
    private VoxelMap voxelMap = null;
    private Mapper mapper = null;
    private bool isObstacle = false;
    private Vector3Int imageIdx = Vector3Int.zero;
    void Awake(){
        voxelMap =  transform.parent.GetComponent<VoxelMap>();
        mapper =  transform.parent.GetComponent<Mapper>();
    }
    void Start() {
        gameObject.GetComponent<Collider>().enabled = true;
        spawnDelay = 0.01f*currentLevel;
        spawnTimer = spawnDelay;
        objectLimit = ((int)(500*(currentLevel + 1) / (maxLevel + 1)));
    }

    void Update()
    {
        if (toGenerate > 0){
            if (transform.parent.childCount > objectLimit){
                return;
            } else if (spawnTimer < 0.0f){
                int x = (((toGenerate - 1) >> 0) & 1)*2-1;
                int y = (((toGenerate - 1) >> 1) & 1)*2-1;
                int z = (((toGenerate - 1) >> 2) & 1)*2-1;
                
                Vector3 scale = (maxCorner - minCorner)/2.0f;
                Vector3 pose = transform.position - Vector3.Scale(scale / 2.0f, new Vector3(x, y, z));
                Vector3 min = minCorner + Vector3.Scale(scale, new Vector3(x+1, y+1, z+1));
                Vector3 max = min + scale;
                Transform new_object = Instantiate(GridRunner, pose, Quaternion.identity, transform.parent);
                new_object.localScale = scale;
                new_object.GetComponent<BinRunner>().setMinCorner(min);
                new_object.GetComponent<BinRunner>().setMaxCorner(max);
                new_object.GetComponent<BinRunner>().setMaxLevel(maxLevel);
                new_object.GetComponent<BinRunner>().setCurrentLevel(currentLevel + 1);
                new_object.GetComponent<Collider>().enabled = false;
                toGenerate--;
                spawnTimer = spawnDelay;
            }
            spawnTimer -= Time.deltaTime;
        } else if (toGenerate == 0){
            Destroy(gameObject);
        } else {
            lifeTimer -= Time.deltaTime;
            if (lifeTimer < 0.0f){
                Destroy(gameObject);
            }
        }
    }
    void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.tag != "CellBinary"){
            if (currentLevel < maxLevel){
                toGenerate = 8;
            } else {
                isObstacle = true;
                Destroy(gameObject);
            }
            gameObject.GetComponent<Collider>().enabled = false;
        } else {
            return;
        }
    }
    public void setMinCorner(Vector3 v){
        minCorner = v;
    }
    public void setMaxCorner(Vector3 v){
        maxCorner = v;
    }
    public void setMaxLevel(int v){
        maxLevel = v;
    }
    public void setCurrentLevel(int v){
        currentLevel = v;
    }

    void OnDestroy(){
        if (currentLevel == maxLevel){
            Vector3 minCorner = transform.parent.GetComponent<Mapper>().getMinCorner();
            Vector3 globalPos = transform.parent.transform.TransformPoint(transform.position);
            Vector3 shiftPos = globalPos + transform.localScale / 2.0f;
            Vector3 imagePos = shiftPos - minCorner;
            
            imageIdx = new Vector3Int(
                ((int)Mathf.Round(imagePos.x/transform.localScale.x)) - 1,
                ((int)Mathf.Round(imagePos.y/transform.localScale.y)) - 1,
                ((int)Mathf.Round(imagePos.z/transform.localScale.z)) - 1
            );
            
            voxelMap.setMapVoxel(ref imageIdx, isObstacle);
            if (isObstacle)
                mapper.setMapPixel(imageIdx.x, imageIdx.z);
        }
    }
}
