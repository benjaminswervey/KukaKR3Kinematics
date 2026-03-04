using UnityEngine;

public class JointGrapher : MonoBehaviour
{
    public LineRenderer[] jointLines;
    
    [Header("Graph Settings")]
    public float graphWidth = 5f;
    public float graphHeight = 2f;
    public float angleRange = 400f; // Shows from -180 to 180

    public void UpdateGraph(float[,] path)
    {
        int joints = path.GetLength(0);
        int steps = path.GetLength(1);

        for (int j = 0; j < joints; j++)
        {
            jointLines[j].positionCount = steps;
            
            for (int s = 0; s < steps; s++)
            {
                // Normalize X from 0 to graphWidth
                float x = (float)s / (steps - 1) * graphWidth;
                
                // Normalize Y: center it at 0, scale by angleRange
                float y = (path[j, s] / angleRange) * (graphHeight / 2f);
                
                // Set the local position
                jointLines[j].SetPosition(s, new Vector3(x, y, 0));
            }
        }
    }
}