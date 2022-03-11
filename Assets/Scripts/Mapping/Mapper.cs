using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.Sprites;

public class Mapper : MonoBehaviour
{
    [Header("Mapper settings")]
    public Vector3Int chunksNumber = new Vector3Int(4, 1, 4);
    public Transform GridRunner = null;
    public int maxLevel = 4;

    [Header("Output settings")]
    public GameObject panel = null;
    
    [Header("Socket interface")]
    public SocketBridge socketBridge;

    // Private declarations
    private Vector3 gridSize = Vector3.zero;
    private Texture2D mapImage = null;
    private Transform Boundary = null;
    private Vector3 minCorner = Vector3.zero;
    private Vector3 maxCorner = Vector3.zero;
    private Vector3 chunkSize = Vector3.zero;
    private Vector2Int startPoint = Vector2Int.zero;
    private Vector2Int goalPoint = Vector2Int.zero;
    private Transform startPointObj;
    private Transform goalPointObj;
    private Vector2Int imageSize;
    private VoxelMap voxelMap;
    void Awake()
    {
        Boundary = transform.GetChild(0);
        voxelMap = transform.GetComponent<VoxelMap>();
    }

    void Start(){
        Boundary.gameObject.SetActive(false);
    }

    void Update()
    {
        
    }
    private void DefineCorners(){
        Boundary.gameObject.SetActive(true);
        minCorner = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
        maxCorner = new Vector3(float.MinValue, float.MinValue, float.MinValue);
        Mesh mesh = Boundary.GetComponent<MeshFilter>().sharedMesh;
        Vector3[] vertices = mesh.vertices;
        for (var i = 0; i < vertices.Length; i++)
        {
            Vector3 direction = Boundary.TransformPoint(vertices[i]);
            if(direction.x < minCorner.x){
                minCorner.x = direction.x;
            } else if (direction.x > maxCorner.x){
                maxCorner.x = direction.x;
            }
            if(direction.y < minCorner.y){
                minCorner.y = direction.y;
            } else if (direction.y > maxCorner.y){
                maxCorner.y = direction.y;
            }
            if(direction.z < minCorner.z){
                minCorner.z = direction.z;
            } else if (direction.z > maxCorner.z){
                maxCorner.z = direction.z;
            }
        }
        Debug.Log("Min Corner: " + minCorner);
        Debug.Log("Max Corner: " + maxCorner);
    }
    private Vector3 getChunkSize(){
        if (Boundary == null){
            Debug.Log("Empty Boundary Item!");
            return Vector3.zero;
        }
        Vector3 range = maxCorner - minCorner;
        Vector3 chunkSize = new Vector3(
            range.x / chunksNumber.x, 
            range.y / chunksNumber.y,
            range.z / chunksNumber.z);

        Debug.Log("Chunk Size: " + chunkSize);  
        return chunkSize;
    }
    private void GenerateChunks(){
        if (GridRunner == null){
            Debug.Log("Empty Chunk Item!");
            return;
        }
        for (int x = 0; x < chunksNumber.x; ++x){
            for (int y = 0; y < chunksNumber.y; ++y){
                for (int z = 0; z < chunksNumber.z; ++z){
                    Vector3 min = minCorner + Vector3.Scale(chunkSize, new Vector3(x, y, z));
                    Vector3 max = min + chunkSize;
                    Vector3 pos = (max + min) / 2.0f;
                    
                    Transform new_object = Instantiate(GridRunner, pos, Quaternion.identity, transform);
                    new_object.localScale = max - min;
                    new_object.GetComponent<BinRunner>().setMinCorner(min);
                    new_object.GetComponent<BinRunner>().setMaxCorner(max);
                    new_object.GetComponent<BinRunner>().setMaxLevel(maxLevel);
                    new_object.GetComponent<BinRunner>().setCurrentLevel(0);
                    new_object.GetComponent<Collider>().enabled = false;
                }
            }
        }
    }

    private void cleanMap(){
        for (int i = 1; i < transform.childCount; ++i){
            Destroy(transform.GetChild(i).gameObject);
        }
    }
    
    public void MakeMap(){
        cleanMap();
        DefineCorners();
        
        chunkSize = getChunkSize();
        if (chunkSize == Vector3.zero){
            Debug.Log("Invalid chunkSize." + chunkSize);
            return;
        }


        Vector2Int chunksNumber2D = new Vector2Int(chunksNumber.x, chunksNumber.z);
        imageSize = chunksNumber2D * ((int)Mathf.Pow(2, maxLevel));
        Debug.Log("Image size: " + imageSize);
        
        Vector3Int voxelMapSize = chunksNumber * ((int)Mathf.Pow(2, maxLevel));
        voxelMap.SetMapSize(ref voxelMapSize);
        Vector3 range = maxCorner - minCorner;
        Vector3 gridSize = new Vector3(
            range.x / voxelMapSize.x, 
            range.y / voxelMapSize.y,
            range.z / voxelMapSize.z);
        voxelMap.SetGridSize(ref gridSize);

        mapImage = new Texture2D(imageSize.x, imageSize.y, TextureFormat.RGB24, false);
        GenerateChunks();
    }

    public Vector3 getMinCorner(){
        return minCorner;
    }

    public void setVoxel(){

    }
    public void setMapPixel(int x, int y){
        Color pixColor = mapImage.GetPixel(x, y);
        pixColor.a = 1.0f;
        pixColor.r-=0.1f;
        pixColor.g-=0.1f;
        pixColor.b-=0.1f;
        mapImage.SetPixel(x, y, pixColor);
        mapImage.Apply();

        if (panel == null){
            Debug.Log("No panel attached.");
        } else if (mapImage != null){
            Sprite new_sprite = Sprite.Create(mapImage, 
                new Rect(0.0f, 0.0f, mapImage.width, mapImage.height),
                new Vector2(0.5f, 0.5f));
            panel.GetComponent<Image>().overrideSprite = new_sprite;
        }
    }

    public void SaveMap(){
        SaveTextureAsPNG(mapImage, "D:/catkin_ws/src/VRPP_ROS/launch" + "/map.png");
        voxelMap.SaveVoxelMap("D:/catkin_ws/src/VRPP_ROS/launch" + "/map.txt");
    }
    private static void SaveTextureAsPNG(Texture2D _texture, string _fullPath)
    {
        byte[] _bytes =_texture.EncodeToPNG();
        System.IO.File.WriteAllBytes(_fullPath, _bytes);
        Debug.Log(_bytes.Length/1024  + "Kb was saved as: " + _fullPath);
    }

    public void setStartPoint(Vector3 newPoint){
        float x = Mathf.Round(newPoint.x / gridSize.x)*gridSize.x;
        float z = Mathf.Round(newPoint.z / gridSize.z)*gridSize.z;
        Vector3 gridPoint = new Vector3(
            x, newPoint.y, z
        );

        startPointObj.transform.position = gridPoint;
        startPointObj.localScale = gridSize;
        
        startPoint = new Vector2Int(
            ((int)Mathf.Round(newPoint.x / gridSize.x)),
            ((int)Mathf.Round(newPoint.z / gridSize.z))
        );
    }

    public void setGoalPoint(Vector3 newPoint){
        float x = Mathf.Round(newPoint.x / gridSize.x)*gridSize.x;
        float z = Mathf.Round(newPoint.z / gridSize.z)*gridSize.z;
        Vector3 gridPoint = new Vector3(
            x, newPoint.y, z
        );

        goalPointObj.transform.position = gridPoint;
        goalPointObj.localScale = gridSize;

        goalPoint = new Vector2Int(
            ((int)Mathf.Round(newPoint.x / gridSize.x)),
            ((int)Mathf.Round(newPoint.z / gridSize.z))
        );
    }
}
