using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;
using System.Linq;

public class VoxelMap : MonoBehaviour
{
    private Vector3Int mapSize_;
    public Transform obstacleCell = null;
    private Vector3 gridSize_, minCorner_;
    private Dictionary<Vector3Int, Vector3> voxelMap = new Dictionary<Vector3Int, Vector3>(0);
    private Dictionary<Vector3Int, float> weightMap = new Dictionary<Vector3Int, float>(0);
    void Awake() {
    }
    public void SetMapSize(Vector3Int v){
        mapSize_ = v;
        voxelMap = new Dictionary<Vector3Int, Vector3>(mapSize_.x * mapSize_.y * mapSize_.z);
        weightMap = new Dictionary<Vector3Int, float>(mapSize_.x * mapSize_.y * mapSize_.z);
        Debug.Log("New voxel size: " + mapSize_);
    }
    public void SetGridSize(Vector3 v){
        gridSize_ = v;
        Debug.Log("New voxel grid size: " + gridSize_.x + ", " + gridSize_.y + ", " + gridSize_.z);
    }
    public Vector3 GetGridSize(){
        return gridSize_;
    }
    public void SetMinCorner(Vector3 v){
        minCorner_ = v;
        Debug.Log("New voxel min corner: " + minCorner_.x + ", " + minCorner_.y + ", " + minCorner_.z);
    }
    public Vector3 GetMinCorner(){
        return minCorner_;
    }
    public void SetObstacleCell(Vector3Int vDiscrete, Vector3 vCont){
        if (!voxelMap.ContainsKey(vDiscrete))
            voxelMap.Add(vDiscrete, vCont);
        if (!weightMap.ContainsKey(vDiscrete))
            weightMap.Add(vDiscrete, Mathf.Infinity);
    }
    public void SetWeight(Vector3Int vD, float cost){
        if (weightMap.ContainsKey(vD))
            weightMap[vD] += cost;
        else 
            weightMap.Add(vD, cost);
    }
    public float GetWeigth(Vector3Int vD){
        if (weightMap.ContainsKey(vD))
            return weightMap[vD];
        else 
            return 0.0f;
    }
    public void ShowMap(){
        if (obstacleCell == null){
            Debug.Log("Empty boundary Item!");
            return;
        }
        foreach (var item in voxelMap) {
            Transform newObj = Instantiate(obstacleCell, item.Value, Quaternion.identity, transform);
            newObj.localScale = gridSize_;
            newObj.gameObject.name = item.Key.ToString();
        }
    }
    public void ShowCell(Vector3Int v){
        if (!voxelMap.ContainsKey(v)){
            Debug.Log("No voxel key: " + v);
            return;
        }
        Transform newObj = Instantiate(obstacleCell, voxelMap[v], Quaternion.identity, transform);
        newObj.localScale = gridSize_;
        newObj.gameObject.name = voxelMap[v].ToString();
    }
    public Vector3 GetContinuousState(Vector3Int v){
        return new Vector3(
            (v.x)*gridSize_.x + gridSize_.x/2.0f + minCorner_.x ,
            (v.y)*gridSize_.y + gridSize_.y/2.0f + minCorner_.y ,
            (v.z)*gridSize_.z + gridSize_.z/2.0f + minCorner_.z );
    }
    public Vector3Int GetDiscreteState(Vector3 v){
        Vector3 unsignedGlobalPose = v - minCorner_;
        return new Vector3Int(
                ((int)Mathf.Floor(unsignedGlobalPose.x/gridSize_.x)),
                ((int)Mathf.Floor(unsignedGlobalPose.y/gridSize_.y)),
                ((int)Mathf.Floor(unsignedGlobalPose.z/gridSize_.z)));
    }
    private bool IsOnMap(Vector3Int v){
        return (v.x >= 0 && v.x < mapSize_.x) && 
            (v.y >= 0 && v.y < mapSize_.y) && 
            (v.z >= 0 && v.z < mapSize_.z);
    }
    public bool IsObstacle(Vector3Int v){
        if (!IsOnMap(v))
            return true;
        return voxelMap.ContainsKey(v);
    }
    public bool LoadVoxelMap(string _fullPath){
        try {
            StreamReader reader = new StreamReader(_fullPath);
            SetMapSize(FormatStringToVector3Int(reader.ReadLine()));
            SetGridSize(FormatStringToVector3(reader.ReadLine()));
            SetMinCorner(FormatStringToVector3(reader.ReadLine()));
            
            string voxelMapCell;
            while ((voxelMapCell = reader.ReadLine()) != null) {
                string[] data = voxelMapCell.Split(';');
                Vector3Int vD = FormatStringToVector3Int(data[0]);
                Vector3 vC = FormatStringToVector3(data[1]);
                SetObstacleCell(vD, vC);
            }
            reader.Close();
        } catch (FileNotFoundException) {
            Debug.Log("File " + _fullPath + " not found.");
            return false;
        }
        return true;
    }
    public void SaveVoxelMap(string _fullPath){
        StreamWriter file = new StreamWriter(_fullPath, append: false);
        file.Write(FormatVector3Int(mapSize_) + "\n");
        file.Write(FormatVector3(gridSize_) + "\n");
        file.Write(FormatVector3(minCorner_) + "\n");
        foreach( KeyValuePair<Vector3Int, Vector3> item in voxelMap){
            file.Write(FormatVector3Int(item.Key) + ";" + FormatVector3(item.Value) + "\n");
        }
        file.Close();
        Debug.Log("Voxel map saved: " + _fullPath);
    }
    private string FormatVector3Int(Vector3Int v){
        return v.x.ToString() + " " + v.y.ToString() + " " + v.z.ToString();
    }
    private string FormatVector3(Vector3 v){
        return v.x.ToString() + " " + v.y.ToString() + " " + v.z.ToString();
    }
    private Vector3Int FormatStringToVector3Int(string str){
        int[] numbers = str.Split(' ').Select(n => System.Convert.ToInt32(n)).ToArray();
        return new Vector3Int(numbers[0], numbers[1], numbers[2]);
    }
    private Vector3 FormatStringToVector3(string str){
        double[] numbers = str.Split(' ').Select(n => System.Convert.ToDouble(n)).ToArray();
        return new Vector3(((float)numbers[0]), ((float)numbers[1]), ((float)numbers[2]));
    }
    public void UpdateRegion(Transform obstacle){
        Debug.Log(obstacle.position);
        Debug.Log(GetDiscreteState(obstacle.position));
    }
}
