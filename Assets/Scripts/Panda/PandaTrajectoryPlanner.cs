using System.Collections;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.PandaMoveit;
using UnityEngine;

using ROSGeometry;
using Quaternion = UnityEngine.Quaternion;
using RosImage = RosMessageTypes.Sensor.Image;
using Transform = UnityEngine.Transform;
using Vector3 = UnityEngine.Vector3;


public class PandaTrajectoryPlanner : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;

    // Hardcoded variables 
    private int numRobotJoints = 7;
    private readonly float jointAssignmentWaitRest = 0.01f;
    private readonly float jointAssignmentWait = 0.1f;
    private readonly float poseAssignmentWait = 1.0f;
    private readonly Vector3 pickPoseOffset = Vector3.up * 0.1f;
    
    // Variables required for ROS communication
    public string rosServiceName = "panda_moveit_trajectory";

    public GameObject robot;
    public GameObject pickup;
    public GameObject place;

    // Articulation Bodies
    private ArticulationBody[] jointArticulationBodies;
    private ArticulationBody leftGripper;
    private ArticulationBody rightGripper;

    private Transform gripperBase;
    private Transform leftGripperGameObject;
    private Transform rightGripperGameObject;

    private enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Place
    };
    
    /// <summary>
    ///     Close the gripper
    /// </summary>
    private void CloseGripper()
    {
        var leftDrive = leftGripper.xDrive;
        var rightDrive = rightGripper.xDrive;

        leftDrive.target = 0.0f;
        rightDrive.target = 0.0f;

        leftGripper.xDrive = leftDrive;
        rightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Open the gripper
    /// </summary>
    public void OpenGripper()
    {
        var leftDrive = leftGripper.xDrive;
        var rightDrive = rightGripper.xDrive;

        leftDrive.target = 0.04f;
        rightDrive.target = 0.04f;

        leftGripper.xDrive = leftDrive;
        rightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>robotMoveitJoints</returns>
    PandaMoveitJoints CurrentJointConfig()
    {
        PandaMoveitJoints joints = new PandaMoveitJoints();
        
        joints.joint_00 = jointArticulationBodies[0].xDrive.target;
        joints.joint_01 = jointArticulationBodies[1].xDrive.target;
        joints.joint_02 = jointArticulationBodies[2].xDrive.target;
        joints.joint_03 = jointArticulationBodies[3].xDrive.target;
        joints.joint_04 = jointArticulationBodies[4].xDrive.target;
        joints.joint_05 = jointArticulationBodies[5].xDrive.target;
        joints.joint_06 = jointArticulationBodies[6].xDrive.target;

        return joints;
    }

    /// <summary>
    ///     Create a new MoverServiceRequest with the current values of the robot's joint angles,
    ///     the target cube's current position and rotation, and the targetPlacement position and rotation.
    ///
    ///     Call the MoverService using the ROSConnection and if a trajectory is successfully planned,
    ///     execute the trajectories in a coroutine.
    /// </summary>
    public void PublishJoints()
    {
        TrajectoryServiceRequest request = new TrajectoryServiceRequest();
        request.joints_input = CurrentJointConfig();
        var pickPosition = pickPoseOffset * 2 + pickup.transform.localPosition;
        var placePosition = place.transform.localPosition + pickPoseOffset * 3;
 
        // Pick Pose
        request.pick_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (pickPosition).To<FLU>(),
            // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
            orientation = pickup.transform.rotation.To<FLU>()
        };

        // Place Pose
        request.place_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (placePosition).To<FLU>(),
            orientation = place.transform.rotation.To<FLU>()
        };

        // Waypoints
        /*int numWayPoints = lineRenderer.positionCount - 4;
        //int numWayPoints = 1;
        request.waypoints = new RosMessageTypes.Geometry.Pose[numWayPoints];
        for (int i = 0; i < numWayPoints; i++)
        {
            request.waypoints[i] = new RosMessageTypes.Geometry.Pose
            {
                position = (waypoints[i+2]).To<FLU>(),
                // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
                orientation = Quaternion.Euler(180, 0, 0).To<FLU>()
            };
        }*/

        ros.SendServiceMessage<TrajectoryServiceResponse>(rosServiceName, request, TrajectoryResponse);
    }

    public void GoToRestPosition()
    {
        StartCoroutine(GoToRest());
    }

    private IEnumerator GoToRest()
    {
        float[] target = { 0.0f, -45.0f, 0.0f, -135.0f, 0.0f, 90.0f, 45.0f };
        var currentJointConfig = CurrentJointConfig();
        float[] lastJointState = { 
                (float)currentJointConfig.joint_00,
                (float)currentJointConfig.joint_01,
                (float)currentJointConfig.joint_02,
                (float)currentJointConfig.joint_03,
                (float)currentJointConfig.joint_04,
                (float)currentJointConfig.joint_05,
                (float)currentJointConfig.joint_06,
        };
        for (int i = 0; i <= 100; i++)
        {
            for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
            {
                var joint1XDrive = jointArticulationBodies[joint].xDrive;
                joint1XDrive.target = lastJointState[joint] + (target[joint] - lastJointState[joint]) * 0.01f * (float)i;
                jointArticulationBodies[joint].xDrive = joint1XDrive;
            }

            yield return new WaitForSeconds(jointAssignmentWaitRest);
        }
        CloseGripper();
    }

    void TrajectoryResponse(TrajectoryServiceResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned.");
        }
    }

    private IEnumerator ExecuteTrajectories(TrajectoryServiceResponse response)
    {
        if (response.trajectories != null)
        {
            // For every trajectory plan returned
            for (int poseIndex  = 0 ; poseIndex < response.trajectories.Length; poseIndex++)
            {
                // For every robot pose in trajectory plan
                for (int jointConfigIndex  = 0 ; jointConfigIndex < response.trajectories[poseIndex].joint_trajectory.points.Length; jointConfigIndex++)
                {
                    var jointPositions = response.trajectories[poseIndex].joint_trajectory.points[jointConfigIndex].positions;
                    float[] result = jointPositions.Select(r=> (float)r * Mathf.Rad2Deg).ToArray();
                    for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive = jointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = result[joint];
                        jointArticulationBodies[joint].xDrive = joint1XDrive;
                    }

                    yield return new WaitForSeconds(jointAssignmentWait);

                }
                yield return new WaitForSeconds(poseAssignmentWait);
                if (poseIndex == (int)Poses.PreGrasp || poseIndex == (int)Poses.Place)
                    OpenGripper();
                // Close the gripper if completed executing the trajectory for the Grasp pose
                else if (poseIndex == (int)Poses.Grasp)
                    CloseGripper();  
            }
            yield return new WaitForSeconds(poseAssignmentWait);
            GoToRestPosition();
        }
    }

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    void Start()
    {
        // Get ROS connection static instance
        ros = ROSConnection.instance;
       
        jointArticulationBodies = new ArticulationBody[numRobotJoints];
        string link0 = "panda_link0";

        string link1 = link0 + "/panda_link1";
        jointArticulationBodies[0] = robot.transform.Find(link1).GetComponent<ArticulationBody>();

        string link2 = link1 + "/panda_link2";
        jointArticulationBodies[1] = robot.transform.Find(link2).GetComponent<ArticulationBody>();

        string link3 = link2 + "/panda_link3";
        jointArticulationBodies[2] = robot.transform.Find(link3).GetComponent<ArticulationBody>();

        string link4 = link3 + "/panda_link4";
        jointArticulationBodies[3] = robot.transform.Find(link4).GetComponent<ArticulationBody>();

        string link5 = link4 + "/panda_link5";
        jointArticulationBodies[4] = robot.transform.Find(link5).GetComponent<ArticulationBody>();

        string link6 = link5 + "/panda_link6";
        jointArticulationBodies[5] = robot.transform.Find(link6).GetComponent<ArticulationBody>();

        string link7 = link6 + "/panda_link7";
        jointArticulationBodies[6] = robot.transform.Find(link7).GetComponent<ArticulationBody>();

        string hand = link7 + "/panda_link8/panda_hand";
        // Find left and right fingers
        string right_gripper = hand + "/panda_leftfinger";
        string left_gripper = hand + "/panda_rightfinger";

        leftGripperGameObject = robot.transform.Find(left_gripper);
        rightGripperGameObject = robot.transform.Find(right_gripper);

        rightGripper = rightGripperGameObject.GetComponent<ArticulationBody>();
        leftGripper = leftGripperGameObject.GetComponent<ArticulationBody>();

        GoToRestPosition();
    }
}