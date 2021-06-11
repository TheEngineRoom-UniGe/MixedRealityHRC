using UnityEngine;

public class DrawTrajectory : MonoBehaviour
{
    public GameObject pick;
    public GameObject target;

    private LineRenderer lineRenderer;
    private readonly Vector3 pickPoseOffset = Vector3.up * 0.15f;
    private float sphereRadius = 0.03f;

    void Start()
    {
        lineRenderer = GetComponent<LineRenderer>();
        Vector3[] positions = new Vector3[4] { 
            pick.transform.position,
            pick.transform.position + pickPoseOffset,
            target.transform.position + pickPoseOffset,
            target.transform.position};
        DrawTraj(positions);
    }

    void DrawTraj(Vector3[] vertexPositions)
    {

        lineRenderer.positionCount = vertexPositions.Length;
        lineRenderer.SetPositions(vertexPositions);
        foreach(Vector3 point in vertexPositions)
        {
            GameObject sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            sphere.transform.position = point;
            sphere.transform.localScale = new Vector3(sphereRadius, sphereRadius, sphereRadius);
            sphere.GetComponent<Collider>().enabled = false;
        }
 
    }

}