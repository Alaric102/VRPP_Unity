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
    
    // [Header("Socket interface")]
    // public SocketBridge socketBridge;

    // Private declarations
    private Transform boundary = null, gridRunner = null;
    private Tuple<Vector3, Vector3> corners = new Tuple<Vector3, Vector3>(Vector3.zero, Vector3.zero);
    private VoxelMap voxelMap;
    private float mappingDuration = 0.0f;
    void Awake() {
        boundary = transform.GetChild(0);
        gridRunner = transform.GetChild(1);

        voxelMap = transform.GetComponent<VoxelMap>();
        
        boundary.gameObject.SetActive(false);
        gridRunner.gameObject.SetActive(false);
    }
    void Start(){
        MakeMap();
    }
    void Update() {
        if (transform.childCount > 3){
            mappingDuration += Time.deltaTime;
        } else if (mappingDuration > 0.0f) {
            Debug.Log("MappingDuration: " + mappingDuration.ToString());
            mappingDuration = 0.0f;
        }
    }
    private void CleanMap(){ // Destroy all child pbjects except boundary
        for (int i = 3; i < transform.childCount; ++i)
            Destroy(transform.GetChild(i).gameObject);
    }
    private Tuple<Vector3, Vector3> DefineMinMaxCorners(){ // define min and max corner vectors from boundary
        if (boundary == null){
            Debug.Log("Empty boundary Item!");
            return new Tuple<Vector3, Vector3>(Vector3.zero, Vector3.zero);
        }
        boundary.gameObject.SetActive(true);
        Vector3 minCorner = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
        Vector3 maxCorner = new Vector3(float.MinValue, float.MinValue, float.MinValue);
        Mesh mesh = boundary.GetComponent<MeshFilter>().sharedMesh;
        Vector3[] vertices = mesh.vertices;
        for (var i = 0; i < vertices.Length; i++) {
            Vector3 direction = boundary.TransformPoint(vertices[i]);
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
        boundary.gameObject.SetActive(false);
        return new Tuple<Vector3, Vector3>(minCorner, maxCorner);
    }
    private Vector3 GetChunkSize(Vector3 minCorner, Vector3 maxCorner){ // define size of each chunk
        if (boundary == null){
            Debug.Log("Empty boundary Item!");
            return Vector3.zero;
        }
        Vector3 range = maxCorner - minCorner;
        Vector3 chunkSize = new Vector3(
            range.x / chunksNumber.x, 
            range.y / chunksNumber.y,
            range.z / chunksNumber.z);
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
                    newObj.GetComponent<BinRunner>().SetCorners(min, max);
                    newObj.GetComponent<BinRunner>().SetRecursionLevels(0, maxLevel);
                    newObj.GetComponent<Collider>().enabled = false;
                    newObj.gameObject.SetActive(true);
                }
        return true;
    }
    public bool MakeMap(){
        CleanMap();
        corners = DefineMinMaxCorners();
        Vector3 chunkSize = GetChunkSize(corners.Item1, corners.Item2);

        if (chunkSize == Vector3.zero){
            Debug.Log("Invalid chunkSize.");
            return false;
        }

        // Vector2Int chunksNumber2D = new Vector2Int(chunksNumber.x, chunksNumber.z);
        // imageSize = chunksNumber2D * ((int)Mathf.Pow(2, maxLevel));
        // Debug.Log("Image size: " + imageSize);
        
        Vector3Int voxelMapSize = chunksNumber * ((int)Mathf.Pow(2, maxLevel));
        voxelMap.SetMapSize(voxelMapSize);
        Debug.Log(corners.Item1.x + ", " + corners.Item1.y + ", " + corners.Item1.z);
        voxelMap.SetMinCorner(corners.Item1);
        Vector3 range = corners.Item2 - corners.Item1;
        Vector3 gridSize = new Vector3(
            range.x / voxelMapSize.x, 
            range.y / voxelMapSize.y,
            range.z / voxelMapSize.z);
        Debug.Log(gridSize);
        voxelMap.SetGridSize(gridSize);

        // mapImage = new Texture2D(imageSize.x, imageSize.y, TextureFormat.RGB24, false);
        // mappingDuration = 0.0f;
        return GenerateChunks(corners.Item1, corners.Item2, chunkSize);
    }
    public Vector3 GetMinCorner(){
        return corners.Item1;
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

    // public void SaveMap(){
    //     SaveTextureAsPNG(mapImage, "D:/catkin_ws/src/VRPP_ROS/launch" + "/map.png");
    //     voxelMap.SaveVoxelMap("D:/catkin_ws/src/VRPP_ROS/launch" + "/map.txt");
    // }
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
