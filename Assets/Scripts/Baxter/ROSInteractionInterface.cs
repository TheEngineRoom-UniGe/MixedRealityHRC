using UnityEngine;
using RosMessageTypes.BaxterMoveit;

public class ROSInteractionInterface : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;

    // Scene objects
    public GameObject robot;
    public GameObject ground;

    // Variables required for ROS communication
    public string plannerServiceName = "baxter_moveit_trajectory";
    public string jointStateServiceName = "baxter_joint_states";

    private MRTKInteractionController controller;

    // Interactable gameobjects
    private GameObject[] tools;
    private int selectedTool = 0;

    void Start()
    {
        // Get ROS connection static instance
        ros = ROSConnection.instance;
        
        // Instantiate Baxter Controller
        controller = gameObject.AddComponent<MRTKInteractionController>();
        controller.Init(robot, ground.transform);

        var request = new JointConfigServiceRequest();
        ros.SendServiceMessage<JointConfigServiceResponse>(jointStateServiceName, request, controller.JointStateServiceResponse);

        // Get reference to interactable gameobjects
        int nTools = ground.transform.childCount;
        int firstToolIdx = 3;
        tools = new GameObject[nTools - firstToolIdx];
        for(int i = 0; i < tools.Length; i++)
        {
            tools[i] = ground.transform.GetChild(i + firstToolIdx).gameObject;
        }
    }

    public void Spawn()
    {
        controller.SpawnRobotAndInterface();
    }

    public void CallPlannerService(string op)
    {
        TrajectoryServiceRequest request = null;
        if(op == "pickandplace")
        {
            request = controller.PickAndPlaceService();
        }
        else if(op == "handover")
        {
            // TODO: reference which tool to be picked up
            request = controller.HandoverService(selectedTool);
        }
        ros.SendServiceMessage<TrajectoryServiceResponse>(plannerServiceName, request, controller.ROSServiceResponse);
    }

    public void AddWaypoint()
    {
        controller.AddWaypoint();
    }

    public void ResetScene()
    {
        controller.ResetScene();
        var request = new JointConfigServiceRequest();
        ros.SendServiceMessage<JointConfigServiceResponse>(jointStateServiceName, request, controller.JointStateServiceResponse);
    }

    public void SelectTool(int id)
    {
        selectedTool = id;
        for(int i=0; i<tools.Length; i++)
        {
            var renderer = tools[i].transform.GetChild(0).gameObject.GetComponent<Renderer>();

            var metallic = Resources.Load("Screw_Metallic", typeof(Material)) as Material;
            var highlight = Resources.Load("Highlight", typeof(Material)) as Material;

            if (i == id)
            {
                renderer.material = highlight;
            }
            else
            {
                renderer.material = metallic;
            }
        }
    }
}