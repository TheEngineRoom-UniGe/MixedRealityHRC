using System.Collections;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.BaxterMoveit;
using UnityEngine;

using ROSGeometry;
using Quaternion = UnityEngine.Quaternion;
using RosImage = RosMessageTypes.Sensor.Image;
using Transform = UnityEngine.Transform;
using Vector3 = UnityEngine.Vector3;


public class TrajectoryPlanner : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;

    // Hardcoded variables 
    private int numRobotJoints = 7;
    private readonly float jointAssignmentWaitRest = 0.01f;
    private readonly float jointAssignmentWait = 0.005f;
    private readonly float poseAssignmentWait = 1.5f;
    private readonly Vector3 pickPoseOffset = Vector3.up * 0.0925f - Vector3.forward *0.02f;
    private readonly Vector3 placeOffset = Vector3.up * (-0.02f);
    
    // Variables required for ROS communication
    public string rosServiceName = "baxter_moveit_trajectory";

    public GameObject baxter;
    public GameObject target;
    public GameObject targetPlacement;

    // Articulation Bodies
    private ArticulationBody[] leftJointArticulationBodies;
    private ArticulationBody[] rightJointArticulationBodies;
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

        leftDrive.target = 0.00f;
        rightDrive.target = 0.00f;

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

        leftDrive.target = 0.025f;
        rightDrive.target = -0.025f;

        leftGripper.xDrive = leftDrive;
        rightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>BaxterMoveitJoints</returns>
    BaxterMoveitJoints CurrentJointConfig()
    {
        BaxterMoveitJoints joints = new BaxterMoveitJoints();
        
        joints.joint_00 = leftJointArticulationBodies[0].xDrive.target;
        joints.joint_01 = leftJointArticulationBodies[1].xDrive.target;
        joints.joint_02 = leftJointArticulationBodies[2].xDrive.target;
        joints.joint_03 = leftJointArticulationBodies[3].xDrive.target;
        joints.joint_04 = leftJointArticulationBodies[4].xDrive.target;
        joints.joint_05 = leftJointArticulationBodies[5].xDrive.target;
        joints.joint_06 = leftJointArticulationBodies[6].xDrive.target;

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
        var pickPosition = pickPoseOffset + target.transform.position;
        var placePosition = placeOffset + targetPlacement.transform.position;

        /*var trajRenderer = GameObject.Find("TrajectoryRender");
        var lineRenderer = trajRenderer.GetComponent<LineRenderer>();
        Vector3[] waypoints = new Vector3[lineRenderer.positionCount];
        lineRenderer.GetPositions(waypoints);*/
 
        // Pick Pose
        request.pick_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (pickPosition).To<FLU>(),
            // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
            orientation = target.transform.rotation.To<FLU>()
        };

        // Place Pose
        request.place_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (placePosition).To<FLU>(),
            orientation = targetPlacement.transform.rotation.To<FLU>()
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

    public void DoHandover()
    {
        TrajectoryServiceRequest request = new TrajectoryServiceRequest();
        request.joints_input = CurrentJointConfig();
        var pickPosition = pickPoseOffset + target.transform.position;
        var placePosition = placeOffset + targetPlacement.transform.position;

        // Pick Pose
        request.pick_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (pickPosition).To<FLU>(),
            // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
            orientation = Quaternion.Euler(180.0f, 0.0f, 0.0f).To<FLU>()
        };

        // Handover Pose
        request.place_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (placePosition).To<FLU>(),
            orientation = Quaternion.Euler(90.0f, 0.0f, 0.0f).To<FLU>()
        };

        ros.SendServiceMessage<TrajectoryServiceResponse>(rosServiceName, request, TrajectoryResponse);
    }

    public void GoToRestPosition(string whichArm)
    {
        if (whichArm == "left")
        {
            StartCoroutine(GoToRestLeft());
        }
        else if (whichArm == "right")
        {
            StartCoroutine(GoToRestRight());
        }
        else
        {
            StartCoroutine(GoToRestLeft());
            StartCoroutine(GoToRestRight());
        }
    }

    private IEnumerator GoToRestLeft()
    {
        float[] target = { -0.47f, -57.17f, -68.3f, 109.6f, 38.34f, 59.17f, -28.56f };
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
            for (int joint = 0; joint < leftJointArticulationBodies.Length; joint++)
            {
                var joint1XDrive = leftJointArticulationBodies[joint].xDrive;
                joint1XDrive.target = lastJointState[joint] + (target[joint] - lastJointState[joint]) * 0.01f * (float)i;
                leftJointArticulationBodies[joint].xDrive = joint1XDrive;
            }

            yield return new WaitForSeconds(jointAssignmentWaitRest);
        }
        CloseGripper();
    }

    private IEnumerator GoToRestRight()
    {
        float[] target = { 0.47f, -57.17f, -68.3f, 109.6f, 38.34f, 59.17f, -28.56f };
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
            for (int joint = 0; joint < rightJointArticulationBodies.Length; joint++)
            {
                var joint1XDrive = rightJointArticulationBodies[joint].xDrive;
                joint1XDrive.target = lastJointState[joint] + (target[joint] - lastJointState[joint]) * 0.01f * (float)i;
                rightJointArticulationBodies[joint].xDrive = joint1XDrive;
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
            Debug.LogError("No trajectory returned from MoverService.");
        }
    }

    private IEnumerator ExecuteTrajectories(TrajectoryServiceResponse response)
    {
        if (response.trajectories != null)
        {
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
            // For every trajectory plan returned
            int steps = 100;
            for (int poseIndex  = 0 ; poseIndex < response.trajectories.Length; poseIndex++)
            {
                // For every robot pose in trajectory plan
                for (int jointConfigIndex  = 0 ; jointConfigIndex < response.trajectories[poseIndex].joint_trajectory.points.Length; jointConfigIndex++)
                {
                    var jointPositions = response.trajectories[poseIndex].joint_trajectory.points[jointConfigIndex].positions;
                    float[] result = jointPositions.Select(r=> (float)r * Mathf.Rad2Deg).ToArray();
                    for (int i=0; i<=steps; i++)
                    {
                        for (int joint = 0; joint < leftJointArticulationBodies.Length; joint++)
                        {
                            var joint1XDrive = leftJointArticulationBodies[joint].xDrive;
                            joint1XDrive.target = lastJointState[joint] + (result[joint]- lastJointState[joint]) * (1.0f/steps) * i;
                            leftJointArticulationBodies[joint].xDrive = joint1XDrive;
                        }

                        yield return new WaitForSeconds(jointAssignmentWait);

                    }

                    // Wait for robot to achieve pose for all joint assignments
                    lastJointState = result;

                }
                if(poseIndex == (int)Poses.PreGrasp)
                    OpenGripper();
                // Close the gripper if completed executing the trajectory for the Grasp pose
                else if (poseIndex == (int)Poses.Grasp)
                    CloseGripper();
                
                // Wait for the robot to achieve the final pose from joint assignment                
            }
            // All trajectories have been executed, open the gripper to place the target cube
            yield return new WaitForSeconds(poseAssignmentWait);
        }
    }

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the leftJointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    void Start()
    {
        // Get ROS connection static instance
        ros = ROSConnection.instance;
        var side = "left";
        leftJointArticulationBodies = new ArticulationBody[numRobotJoints];
        string upper_shoulder = "base/torso/"+ side + "_arm_mount/" + side + "_upper_shoulder";
        leftJointArticulationBodies[0] = baxter.transform.Find(upper_shoulder).GetComponent<ArticulationBody>();

        string lower_shoudler = upper_shoulder + "/" + side + "_lower_shoulder";
        leftJointArticulationBodies[1] = baxter.transform.Find(lower_shoudler).GetComponent<ArticulationBody>();

        string upper_elbow = lower_shoudler + "/" + side + "_upper_elbow";
        leftJointArticulationBodies[2] = baxter.transform.Find(upper_elbow).GetComponent<ArticulationBody>();

        string lower_elbow = upper_elbow + "/" + side + "_lower_elbow";
        leftJointArticulationBodies[3] = baxter.transform.Find(lower_elbow).GetComponent<ArticulationBody>();

        string upper_forearm = lower_elbow + "/" + side + "_upper_forearm";
        leftJointArticulationBodies[4] = baxter.transform.Find(upper_forearm).GetComponent<ArticulationBody>();

        string lower_forearm = upper_forearm + "/" + side + "_lower_forearm";
        leftJointArticulationBodies[5] = baxter.transform.Find(lower_forearm).GetComponent<ArticulationBody>();

        string wrist = lower_forearm + "/" + side + "_wrist";
        leftJointArticulationBodies[6] = baxter.transform.Find(wrist).GetComponent<ArticulationBody>();

        string hand = wrist + "/" + side + "_hand";
        // Find left and right fingers
        string right_gripper = hand + "/" + side + "_gripper_base/l_gripper_r_finger";
        string left_gripper = hand + "/" + side + "_gripper_base/l_gripper_l_finger";
        string gripper_base = hand + "/" + side + "_gripper_base/Collisions/unnamed";

        gripperBase = baxter.transform.Find(gripper_base);
        leftGripperGameObject = baxter.transform.Find(left_gripper);
        rightGripperGameObject = baxter.transform.Find(right_gripper);

        rightGripper = rightGripperGameObject.GetComponent<ArticulationBody>();
        leftGripper = leftGripperGameObject.GetComponent<ArticulationBody>();


        side = "right";
        rightJointArticulationBodies = new ArticulationBody[numRobotJoints];
        upper_shoulder = "base/torso/" + side + "_arm_mount/" + side + "_upper_shoulder";
        rightJointArticulationBodies[0] = baxter.transform.Find(upper_shoulder).GetComponent<ArticulationBody>();

        lower_shoudler = upper_shoulder + "/" + side + "_lower_shoulder";
        rightJointArticulationBodies[1] = baxter.transform.Find(lower_shoudler).GetComponent<ArticulationBody>();

        upper_elbow = lower_shoudler + "/" + side + "_upper_elbow";
        rightJointArticulationBodies[2] = baxter.transform.Find(upper_elbow).GetComponent<ArticulationBody>();

        lower_elbow = upper_elbow + "/" + side + "_lower_elbow";
        rightJointArticulationBodies[3] = baxter.transform.Find(lower_elbow).GetComponent<ArticulationBody>();

        upper_forearm = lower_elbow + "/" + side + "_upper_forearm";
        rightJointArticulationBodies[4] = baxter.transform.Find(upper_forearm).GetComponent<ArticulationBody>();

        lower_forearm = upper_forearm + "/" + side + "_lower_forearm";
        rightJointArticulationBodies[5] = baxter.transform.Find(lower_forearm).GetComponent<ArticulationBody>();

        wrist = lower_forearm + "/" + side + "_wrist";
        rightJointArticulationBodies[6] = baxter.transform.Find(wrist).GetComponent<ArticulationBody>();


        GoToRestPosition("both");
    }
}