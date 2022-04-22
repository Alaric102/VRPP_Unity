using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Sprites;

public class Mapper : MonoBehaviour
{
    [Header("Mapper settings")]
    public Vector3Int chunksNumber = new Vector3Int(4, 1, 4);
    public int maxLevel = 4;

    // [Header("Output settings")]
    // public GameObject panel = null;

    // Private declarations
    private Transform boundary = null, gridRunner = null, obstacleState = null;
    private Vector3 minCorner_ = Vector3.zero, maxCorner_ = Vector3.zero;
    private VoxelMap voxelMap;
    private float mappingDuration = 0.0f;
    private bool isMapping = false;
    void Awake() {
        boundary = transform.GetChild(0);
        gridRunner = transform.GetChild(1);
        obstacleState = transform.GetChild(2);

        voxelMap = transform.GetComponent<VoxelMap>();
        
        boundary.gameObject.SetActive(false);
        gridRunner.gameObject.SetActive(false);
        obstacleState.gameObject.SetActive(false);
    }
    public Navigation navigation;
    void Start(){
        if (!voxelMap.LoadVoxelMap("D:/catkin_ws/src/VRPP_ROS/launch" + "/map.txt")){
            isMapping = MakeMap();
        } else {
            minCorner_ = voxelMap.GetMinCorner();
        }
        navigation.StartPlanning();
    }
    void Update() {
        if (isMapping){
            if (transform.childCount > 3){
                mappingDuration += Time.deltaTime;
            } else {
                Debug.Log("MappingDuration: " + mappingDuration.ToString());
                isMapping = false;
                // navigation.StartPlanning();
            }
        }
    }
    private void CleanMap(){ // Destroy all child pbjects except boundary
        for (int i = 3; i < transform.childCount; ++i)
            Destroy(transform.GetChild(i).gameObject);
    }
    private Tuple<Vector3, Vector3> DefineMinMaxCorners(Transform bb){ // define min and max corner vectors from boundary
        if (bb == null){
            Debug.Log("Empty bb Item!");
            return new Tuple<Vector3, Vector3>(Vector3.zero, Vector3.zero);
        }
        bb.gameObject.SetActive(true);
        Vector3 minCorner = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
        Vector3 maxCorner = new Vector3(float.MinValue, float.MinValue, float.MinValue);
        Mesh mesh = bb.GetComponent<MeshFilter>().sharedMesh;
        Vector3[] vertices = mesh.vertices;
        for (var i = 0; i < vertices.Length; i++) {
            Vector3 direction = bb.TransformPoint(vertices[i]);
            if(direction.x < minCorner.x)
                minCorner.x = direction.x;
            else if (direction.x > maxCorner.x)
                maxCorner.x = direction.x;

            if(direction.y < minCorner.y)
                minCorner.y = direction.y;
            else if (direction.y > maxCorner.y)
                maxCorner.y = direction.y;

            if(direction.z < minCorner.z)
                minCorner.z = direction.z;
            else if (direction.z > maxCorner.z)
                maxCorner.z = direction.z;
        }
        bb.gameObject.SetActive(false);
        Debug.Log("minCorner: " + GetVectorString(minCorner) + "\nmax corner: " + GetVectorString(maxCorner));
        return new Tuple<Vector3, Vector3>(minCorner, maxCorner);
    }
    private string GetVectorString(Vector3 v){ return v.x + ", " + v.y + ", " + v.z; }
    private Vector3 GetChunkSize(Vector3 minCorner, Vector3 maxCorner){ // define size of each chunk
        if (boundary == null){
            Debug.Log("Empty boundary Item!");
            return Vector3.zero;
        }
        Vector3 maxSize = maxCorner - minCorner;
        Vector3 chunkSize = new Vector3( maxSize.x / chunksNumber.x, maxSize.y / chunksNumber.y, maxSize.z / chunksNumber.z);
        Debug.Log("Chunk size: " + GetVectorString(chunkSize));
        return chunkSize;
    }
    private bool GenerateChunks(Vector3 minCorner, Vector3 maxCorner, Vector3 chunkSize){ // Generates first chunks of gridRunner objects
        if (gridRunner == null){
            Debug.Log("Empty Chunk Item!");
            return false;
        }
        for (int x = 0; x < chunksNumber.x; ++x)
            for (int y = 0; y < chunksNumber.y; ++y)
                for (int z = 0; z < chunksNumber.z; ++z){
                    Vector3 min = minCorner + Vector3.Scale(chunkSize, new Vector3(x, y, z));
                    Vector3 max = min + chunkSize;
                    
                    Transform newObj = Instantiate(gridRunner, (max + min) / 2.0f, Quaternion.identity, transform);
                    newObj.GetComponent<BinRunner>().Init(min, max, 0, maxLevel);
                    // newObj.GetComponent<BinRunner>().SetRecursionLevels();
                    newObj.GetComponent<Collider>().enabled = false;
                    newObj.gameObject.SetActive(true);
                }
        return true;
    }
    public bool MakeMap(){ // Create full map in boundary box
        CleanMap();
        Tuple<Vector3, Vector3> corners = DefineMinMaxCorners(boundary.transform);
        minCorner_ = corners.Item1;
        maxCorner_ = corners.Item2;
        voxelMap.SetMinCorner(minCorner_);

        Vector3 chunkSize = GetChunkSize(corners.Item1, corners.Item2);
        if (chunkSize == Vector3.zero){
            Debug.Log("Invalid chunkSize.");
            return false;
        }

        // Define voxel map size
        Vector3Int voxelMapSize = chunksNumber * ((int)Mathf.Pow(2, maxLevel));
        voxelMap.SetMapSize(voxelMapSize);
        // Define final grid size
        Vector3 range = maxCorner_ - minCorner_;
        Vector3 gridSize = new Vector3(range.x / voxelMapSize.x, range.y / voxelMapSize.y, range.z / voxelMapSize.z);
        voxelMap.SetGridSize(gridSize);

        // isMapping = true;
        // mappingDuration = 0.0f;
        return GenerateChunks(minCorner_, maxCorner_, chunkSize);
    }
    public Vector3 GetMinCorner(){ return minCorner_; }
    public Transform GetObstacleState(){ // Get activated obstacle transform with disabled collider
        obstacleState.gameObject.SetActive(true);
        obstacleState.GetChild(0).GetComponent<Collider>().enabled = false;
        return obstacleState;
    }
    public void GenerateObstacle(){ // Generate new obstacle from prefab with activated collider
        Transform newObst = Instantiate(obstacleState, obstacleState.position, obstacleState.rotation);
        newObst.gameObject.SetActive(true);
        newObst.name = "userObst";
        newObst.GetChild(0).GetComponent<Collider>().enabled = true;
        UpdateMap(obstacleState);
    }
    private void UpdateMap(Transform obstacle){
        // Implement updating mapp in some region
        Vector3Int pos = voxelMap.GetDiscreteState(obstacle.position);
        Vector3 scale = obstacle.lossyScale;
        Vector3Int range = new Vector3Int(
            ((int)(scale.x/voxelMap.GetGridSize().x)),
            ((int)(scale.y/voxelMap.GetGridSize().y)),
            ((int)(scale.z/voxelMap.GetGridSize().z))
        );
        isMapping = true;
        for (int x = pos.x - range.x; x < pos.x + range.x; ++x)
            for (int y = pos.y - range.y; y < pos.y + range.y; ++y)
                for (int z = pos.z - range.z; z < pos.z + range.z; ++z){
                    if (!voxelMap.IsObstacle(new Vector3Int(x, y, z))){
                        Vector3 newPos = voxelMap.GetContinuousState(new Vector3Int(x, y, z));
                        Vector3 min = newPos - voxelMap.GetGridSize() / 2.0f;
                        Vector3 max = newPos + voxelMap.GetGridSize() / 2.0f;
                        
                        Transform newObj = Instantiate(gridRunner, newPos, Quaternion.identity, transform);
                        newObj.GetComponent<BinRunner>().Init(min, max, maxLevel, maxLevel);
                        // newObj.GetComponent<BinRunner>().SetRecursionLevels();
                        newObj.GetComponent<Collider>().enabled = false;
                        newObj.gameObject.SetActive(true);
                    }
                }
    }
    // public void setMapPixel(int x, int y){
    //     Color pixColor = mapImage.GetPixel(x, y);
    //     pixColor.a = 1.0f;
    //     pixColor.r-=0.1f;
    //     pixColor.g-=0.1f;
    //     pixColor.b-=0.1f;
    //     mapImage.SetPixel(x, y, pixColor);
    //     mapImage.Apply();

    //     if (panel == null){
    //         Debug.Log("No panel attached.");
    //     } else if (mapImage != null){
    //         Sprite new_sprite = Sprite.Create(mapImage, 
    //             new Rect(0.0f, 0.0f, mapImage.width, mapImage.height),
    //             new Vector2(0.5f, 0.5f));
    //         panel.GetComponent<Image>().overrideSprite = new_sprite;
    //     }
    // }

    public void SaveMap(){
        // SaveTextureAsPNG(mapImage, "D:/catkin_ws/src/VRPP_ROS/launch" + "/map.png");
        voxelMap.SaveVoxelMap("D:/catkin_ws/src/VRPP_ROS/launch" + "/map.txt");
    }
    public void ShowMap(){
        // SaveTextureAsPNG(mapImage, "D:/catkin_ws/src/VRPP_ROS/launch" + "/map.png");
        voxelMap.ShowMap();
    }
    // private static void SaveTextureAsPNG(Texture2D _texture, string _fullPath)
    // {
    //     byte[] _bytes =_texture.EncodeToPNG();
    //     System.IO.File.WriteAllBytes(_fullPath, _bytes);
    //     Debug.Log(_bytes.Length/1024  + "Kb was saved as: " + _fullPath);
    // }

    // public void setStartPoint(Vector3 newPoint){
    //     float x = Mathf.Round(newPoint.x / gridSize.x)*gridSize.x;
    //     float z = Mathf.Round(newPoint.z / gridSize.z)*gridSize.z;
    //     Vector3 gridPoint = new Vector3(
    //         x, newPoint.y, z
    //     );

    //     startPointObj.transform.position = gridPoint;
    //     startPointObj.localScale = gridSize;
        
    //     startPoint = new Vector2Int(
    //         ((int)Mathf.Round(newPoint.x / gridSize.x)),
    //         ((int)Mathf.Round(newPoint.z / gridSize.z))
    //     );
    // }

    // public void setGoalPoint(Vector3 newPoint){
    //     float x = Mathf.Round(newPoint.x / gridSize.x)*gridSize.x;
    //     float z = Mathf.Round(newPoint.z / gridSize.z)*gridSize.z;
    //     Vector3 gridPoint = new Vector3(
    //         x, newPoint.y, z
    //     );

    //     goalPointObj.transform.position = gridPoint;
    //     goalPointObj.localScale = gridSize;

    //     goalPoint = new Vector2Int(
    //         ((int)Mathf.Round(newPoint.x / gridSize.x)),
    //         ((int)Mathf.Round(newPoint.z / gridSize.z))
    //     );
    // }
}
