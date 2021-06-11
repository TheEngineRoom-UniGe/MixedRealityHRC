using UnityEngine;
using RosMessageTypes.BaxterMoveit;
using System.Collections;

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

    private MRTKPlanController controller;

    void Start()
    {
        // Get ROS connection static instance
        ros = ROSConnection.instance;
        
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

    public void Plan()
    {
        StartCoroutine(ExecutePlan());
    }

    private IEnumerator ExecutePlan()
    {
        TrajectoryServiceRequest request = null;

        request = controller.HandoverService(1);
        ros.SendServiceMessage<TrajectoryServiceResponse>(plannerServiceName, request, controller.ROSServiceResponse);
        yield return new WaitForSeconds(5);
        /*request = controller.HandoverService(0);
        ros.SendServiceMessage<TrajectoryServiceResponse>(plannerServiceName, request, controller.ROSServiceResponse);
        yield return new WaitForSeconds(20);*/
        request = controller.PickAndPlaceService(1);
        ros.SendServiceMessage<TrajectoryServiceResponse>(plannerServiceName, request, controller.ROSServiceResponse);
        /*yield return new WaitForSeconds(5);
        request = controller.HandoverService(1);
        ros.SendServiceMessage<TrajectoryServiceResponse>(plannerServiceName, request, controller.ROSServiceResponse);*/
    }
}