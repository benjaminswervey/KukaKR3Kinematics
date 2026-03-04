using UnityEngine;

public class JointVisualizer : MonoBehaviour
{
    public float lineLength = 0.5f;

    // This runs automatically in the Scene View
    void OnDrawGizmos()
    {
        // Draw the local axes of the joint
        Gizmos.color = Color.red; // X Axis
        Gizmos.DrawRay(transform.position, transform.right * lineLength);

        Gizmos.color = Color.green; // Y Axis
        Gizmos.DrawRay(transform.position, transform.up * lineLength);

        Gizmos.color = Color.blue; // Z Axis
        Gizmos.DrawRay(transform.position, transform.forward * lineLength);
    }
}