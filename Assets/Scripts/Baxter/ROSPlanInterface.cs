using UnityEngine;
using RosMessageTypes.BaxterMoveit;
using System.Collections;

using Bool = RosMessageTypes.Std.Bool;

public class ROSPlanInterface : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;

    // Scene objects
    public GameObject robot;
    public GameObject ground;

    // Variables required for ROS communication
    public string plannerServiceName = "baxter_moveit_trajectory";
    public string jointStateServiceName = "baxter_joint_states";
    public string baxterActionTopicName = "baxter_action";

    private MRTKPlanController controller;

    void Start()
    {
        // Get ROS connection static instance
        ros = ROSConnection.instance;

        // Subscribe to baxter action topic
        ros.Subscribe<BaxterAction>(baxterActionTopicName, PlanAction);
        
        // Instantiate Baxter Controller
        controller = gameObject.AddComponent<MRTKPlanController>();
        controller.Init(robot, ground.transform);

        var request = new JointConfigServiceRequest();
        ros.SendServiceMessage<JointConfigServiceResponse>(jointStateServiceName, request, controller.JointStateServiceResponse);
    }

    public void Spawn()
    {
        controller.SpawnRobotAndInterface();
    }

    public void ResetScene()
    {
        controller.ResetScene();
        var request = new JointConfigServiceRequest();
        ros.SendServiceMessage<JointConfigServiceResponse>(jointStateServiceName, request, controller.JointStateServiceResponse);
    }

    public void PlanAction(BaxterAction msg)
    {
        TrajectoryServiceRequest request = null;

        if (msg.operation == "pickandplace")
        {
            request = controller.PickAndPlaceService(msg.id);
        }
        else if (msg.operation == "componenthandover")
        {
            request = controller.ComponentHandoverService(msg.id);
        }
        else if (msg.operation == "toolhandover")
        {
            request = controller.ToolHandoverService(msg.id);
        }
        else if (msg.operation == "putback")
        {
            request = controller.PutBackService(msg.id);
        }
        ros.SendServiceMessage<TrajectoryServiceResponse>(request.arm + "_group/" + plannerServiceName, request, controller.ROSServiceResponse);
    }
}