using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using System.Text;
public class VoxelMap : MonoBehaviour
{
    private Vector3Int mapSize_;
    private int voxelCapacity_;
    private Dictionary<Vector3Int, bool> voxelMap = new Dictionary<Vector3Int, bool>(0);
    void Awake() {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    public void setMapSize(ref Vector3Int v){
        mapSize_ = v;
        voxelCapacity_ = mapSize_.x * mapSize_.y * mapSize_.z;
        voxelMap = new Dictionary<Vector3Int, bool>(voxelCapacity_);
        Debug.Log("New voxel size: " + mapSize_ + ", capacity: " + voxelCapacity_);
    }

    public void setMapVoxel(ref Vector3Int v, bool isObstacle){
        voxelMap[v] = isObstacle;
    }

    public void SaveVoxelMap(string _fullPath){
        StreamWriter file = new StreamWriter(_fullPath, append: false);
        file.Write(formatVector3Int(mapSize_) + "\n");
        foreach( KeyValuePair<Vector3Int, bool> item in voxelMap){
            file.Write(formatVector3Int(item.Key) + " " + item.Value.ToString() + "\n");
        }
        file.Close();
        Debug.Log("Voxel map saved to " + _fullPath);
    }
    private string formatVector3Int(Vector3Int v){
        return v.x.ToString() + " " + v.y.ToString() + " " + v.z.ToString();
    }
}
