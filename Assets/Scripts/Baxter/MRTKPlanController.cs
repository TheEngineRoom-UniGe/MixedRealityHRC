using System.Collections;
using System.Linq;

using RosMessageTypes.BaxterMoveit;
using ROSGeometry;
using Quaternion = UnityEngine.Quaternion;
using Transform = UnityEngine.Transform;
using Vector3 = UnityEngine.Vector3;

using UnityEngine;

public class MRTKPlanController : MonoBehaviour
{
    private int numRobotJoints = 7;

    // Timing variables for rendering trajectory
    private float jointAssignmentWaitRest = 0.01f;
    private float jointAssignmentWait = 0.005f;
    private float poseAssignmentWait = 0.25f;
    private float oneSecondWait = 1.0f;
    private float handoverPoseWait = 5.0f;

    // Offset variables for picking and placing objects
    private readonly Vector3 liftOffset = Vector3.up * 0.1f;

    // Offset variables for spawning robot correctly
    private readonly Vector3 depthOffset = Vector3.forward * 0.15f;
    private readonly Vector3 heightOffset = Vector3.up * 0.285f;
    
    // Scene objects (robot and interactables)
    private GameObject baxter;
    private GameObject[] pickPoses;
    private GameObject[] placePoses;
    private GameObject[] tools;

    // Articulation Bodies
    private ArticulationBody[] leftJointArticulationBodies;
    private ArticulationBody[] rightJointArticulationBodies;
    private ArticulationBody[] leftHand;
    private ArticulationBody[] rightHand;

    // Other scene objects (buttons, marker)
    private GameObject ImageTarget;
    private GameObject[] buttons;
    
    // Hardcoded variables needed for referencing joints indices
    private double[] restPosition;
    int[] leftIndices = { 4, 5, 2, 3, 6, 7, 8 };
    int[] rightIndices = { 11, 12, 9, 10, 13, 14, 15 };

    // Utility variables
    int currentPickID = 0;
    int currentToolID = 0;

    private Vector3[] objectsGraspOffsets =
    {
        new Vector3(0.05f, 0, -0.05f),
        Vector3.zero,
        Vector3.zero,
        Vector3.zero,
        Vector3.zero,
        Vector3.zero,
        Vector3.zero,
        Vector3.zero,
    };

    private Vector3[] toolsGraspOffsets =
    {
        Vector3.forward * 0.025f,
        Vector3.forward * -0.1f,
    };

    private enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Move,
        Place,
        Liftup,
        Return
    };

    private void CloseGripper(string hand)
    {
        var gripper = leftHand;
        if(hand == "right")
        {
            gripper = rightHand;
        }
        
        var leftDrive = gripper[0].xDrive;
        var rightDrive = gripper[1].xDrive;

        leftDrive.target = -0.005f;
        rightDrive.target = 0.005f;

        gripper[0].xDrive = leftDrive;
        gripper[1].xDrive = rightDrive;
    }

    private void OpenGripper(string hand)
    {
        var gripper = leftHand;
        if (hand == "right")
        {
            gripper = rightHand;
        }

        var leftDrive = gripper[0].xDrive;
        var rightDrive = gripper[1].xDrive;

        leftDrive.target = 0.025f;
        rightDrive.target = -0.025f;

        gripper[0].xDrive = leftDrive;
        gripper[1].xDrive = rightDrive;
    }

    public void JointStateServiceResponse(JointConfigServiceResponse response)
    {
        var msg = response.joint_state_msg;
        restPosition = new double[msg.position.Length];
        {
            for(int i = 0; i<msg.position.Length; i++)
            {
                restPosition[i] = msg.position[i];
            }
        }
        GoToRestPosition("both");
    }

    public void GoToRestPosition(string whichArm)
    {
        if (whichArm == "left")
        {
            StartCoroutine(GoToRest(leftIndices, whichArm));
        }
        else if (whichArm == "right")
        {
            StartCoroutine(GoToRest(rightIndices, whichArm));
        }
        else
        {
            StartCoroutine(GoToRest(leftIndices, "left"));
            StartCoroutine(GoToRest(rightIndices, "right"));
        }
    }

    private IEnumerator GoToRest(int[] indices, string arm)
    {
        float[] target = new float[indices.Length];
        for(int i = 0; i<indices.Length; i++)
        {
            target[i] = Mathf.Rad2Deg * (float)restPosition[indices[i]];
        }
        var currentJointConfig = CurrentJointConfig(arm);
        float[] lastJointState = {
                (float)currentJointConfig.joint_00,
                (float)currentJointConfig.joint_01,
                (float)currentJointConfig.joint_02,
                (float)currentJointConfig.joint_03,
                (float)currentJointConfig.joint_04,
                (float)currentJointConfig.joint_05,
                (float)currentJointConfig.joint_06,
        };
        var steps = 100;
        var jointArticulationBodies = leftJointArticulationBodies;
        if(arm == "right")
        {
            jointArticulationBodies = rightJointArticulationBodies;
        }
        for (int i = 0; i <= steps; i++)
        {
            for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
            {
                var joint1XDrive = jointArticulationBodies[joint].xDrive;
                joint1XDrive.target = lastJointState[joint] + (target[joint] - lastJointState[joint]) * (float)(1.0f / steps) * (float)i;
                jointArticulationBodies[joint].xDrive = joint1XDrive;
            }

            yield return new WaitForSeconds(jointAssignmentWaitRest);
        }
        OpenGripper(arm);
    }

    BaxterMoveitJoints CurrentJointConfig(string arm)
    {
        BaxterMoveitJoints joints = new BaxterMoveitJoints();
        var jointArticulationBodies = leftJointArticulationBodies;
        if(arm == "right")
        {
            jointArticulationBodies = rightJointArticulationBodies;
        }

        joints.joint_00 = jointArticulationBodies[0].xDrive.target;
        joints.joint_01 = jointArticulationBodies[1].xDrive.target;
        joints.joint_02 = jointArticulationBodies[2].xDrive.target;
        joints.joint_03 = jointArticulationBodies[3].xDrive.target;
        joints.joint_04 = jointArticulationBodies[4].xDrive.target;
        joints.joint_05 = jointArticulationBodies[5].xDrive.target;
        joints.joint_06 = jointArticulationBodies[6].xDrive.target;

        return joints;
    }

    // Pick and place service request
    public TrajectoryServiceRequest PickAndPlaceService(int ID)
    {
        currentPickID = ID;
        TrajectoryServiceRequest request = new TrajectoryServiceRequest();
        request.operation = "pickandplace";

        var pickPosition = pickPoses[ID].transform.localPosition + liftOffset + objectsGraspOffsets[ID];

        string arm = "left";
        var placeObj = placePoses[0];
        if(pickPosition.x > 0)
        {
            placeObj = placePoses[1];
            arm = "right";
        }
        var placePosition = placeObj.transform.localPosition + liftOffset;

        request.arm = arm;
        request.joints_input = CurrentJointConfig(arm);
        // Pick Pose
        request.pick_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (pickPosition).To<FLU>(),
            orientation = Quaternion.Euler(180, 0, 0).To<FLU>()
        };

        // Place Pose
        request.place_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (placePosition).To<FLU>(),
            orientation = Quaternion.Euler(180, 0, 0).To<FLU>()
        };

        return request;
    }

    // Handover service request
    public TrajectoryServiceRequest HandoverService(int ID)
    {
        currentToolID = ID;
        
        TrajectoryServiceRequest request = new TrajectoryServiceRequest();
        request.operation = "handover";

        // TODO: fixed handover with left arm
        string arm = "left";
        var placeObj = placePoses[2];

        request.arm = arm;
        request.joints_input = CurrentJointConfig(arm);
        var pickPosition = tools[ID].transform.localPosition + liftOffset + toolsGraspOffsets[ID];
        var placePosition = placeObj.transform.localPosition;

        placeObj.SetActive(false);        

        // Pick Pose
        request.pick_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (pickPosition).To<FLU>(),
            orientation = Quaternion.Euler(-180.0f, 0.0f, 0.0f).To<FLU>()
        };

        // Handover Pose
        request.place_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (placePosition).To<FLU>(),
            orientation = Quaternion.Euler(-90.0f, 90.0f, 90.0f).To<FLU>()
        };

        return request;
    }

    public void ROSServiceResponse(TrajectoryServiceResponse response)
    {
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.Log("No trajectory returned from MoveIt.");
        }
    }

    private IEnumerator ExecuteTrajectories(TrajectoryServiceResponse response)
    {
        if (response.trajectories != null)
        {
            var arm = response.arm;
            var currentJointConfig = CurrentJointConfig(arm);
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
            int steps = 40;
            var jointArticulationBodies = leftJointArticulationBodies;
            if (arm == "right")
            {
                jointArticulationBodies = rightJointArticulationBodies;
            }
            yield return new WaitForSeconds(oneSecondWait);
            for (int poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                // For every robot pose in trajectory plan
                for (int jointConfigIndex = 0; jointConfigIndex < response.trajectories[poseIndex].joint_trajectory.points.Length; jointConfigIndex++)
                {
                    var jointPositions = response.trajectories[poseIndex].joint_trajectory.points[jointConfigIndex].positions;
                    float[] result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();
                    for (int i = 0; i <= steps; i++)
                    {
                        for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
                        {
                            var joint1XDrive = jointArticulationBodies[joint].xDrive;
                            joint1XDrive.target = lastJointState[joint] + (result[joint] - lastJointState[joint]) * (1.0f / steps) * i;
                            jointArticulationBodies[joint].xDrive = joint1XDrive;
                        }

                        yield return new WaitForSeconds(jointAssignmentWait);

                    }
                    // Wait for robot to achieve pose for all joint assignments
                    lastJointState = result;
                }
                // Handle different cases based on the executed operation
                if (response.operation == "pickandplace")
                {
                    if (poseIndex == (int)Poses.PreGrasp || poseIndex == (int)Poses.Place)
                    {
                        yield return new WaitForSeconds(poseAssignmentWait);
                        OpenGripper(arm);
                    }
                    else if (poseIndex == (int)Poses.Grasp)
                        CloseGripper(arm);
                    else if (poseIndex == (int)Poses.Return)
                    {
                        pickPoses[currentPickID].SetActive(false);
                    }
                }
                else if (response.operation == "handover")
                {
                    if (poseIndex == (int)Poses.Move)
                    {
                        yield return new WaitForSeconds(handoverPoseWait);
                        OpenGripper(arm);
                        tools[currentToolID].SetActive(false);
                    }
                    else if (poseIndex == (int)Poses.Grasp)
                        CloseGripper(arm);
                }
            }
        }
    }

    void GetRobotReference()
    {
        var side = "left";
        leftJointArticulationBodies = new ArticulationBody[numRobotJoints];
        string upper_shoulder = "base/" + side + "_arm_mount/" + side + "_upper_shoulder";
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

        leftHand = new ArticulationBody[2];
        leftHand[0] = baxter.transform.Find(left_gripper).GetComponent<ArticulationBody>();
        leftHand[1] = baxter.transform.Find(right_gripper).GetComponent<ArticulationBody>();

        side = "right";
        rightJointArticulationBodies = new ArticulationBody[numRobotJoints];
        upper_shoulder = "base/" + side + "_arm_mount/" + side + "_upper_shoulder";
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

        hand = wrist + "/" + side + "_hand";
        // Find left and right fingers
        right_gripper = hand + "/" + side + "_gripper_base/r_gripper_r_finger";
        left_gripper = hand + "/" + side + "_gripper_base/r_gripper_l_finger";

        rightHand = new ArticulationBody[2];
        rightHand[0] = baxter.transform.Find(left_gripper).GetComponent<ArticulationBody>();
        rightHand[1] = baxter.transform.Find(right_gripper).GetComponent<ArticulationBody>();
    }

    public void Init(GameObject baxter, Transform ground)
    {
        this.baxter = baxter;

        int n = ground.childCount;
        int nPick = 8;
        int nPlace = 3;

        // Pick poses
        this.pickPoses = new GameObject[nPick];
        int i = 0;
        for(i=0; i < nPick; i++)
        {
            this.pickPoses[i] = ground.GetChild(i).gameObject;
        }

        // Place poses (0 -> left, 1-> right, 2-> handover)
        this.placePoses = new GameObject[3];
        int k = 0;
        for (int j = i; j < i+nPlace; j++)
        {
            this.placePoses[k] = ground.GetChild(j).gameObject;
            k++;
        }

        this.tools = new GameObject[2];
        // Screwdriver
        this.tools[0] = ground.GetChild(n-2).gameObject;
        // Hammer
        this.tools[1] = ground.GetChild(n-1).gameObject;

        GetRobotReference();

        // Get reference to interface buttons and initialize them as inactive
        var buttonsLayer = GameObject.Find("Canvas/Buttons");
        int numButtons = buttonsLayer.transform.childCount;
        buttons = new GameObject[numButtons];
        for (int j = 0; j < numButtons; j++)
        {
            var button = buttonsLayer.transform.GetChild(j).gameObject;
            button.SetActive(false);
            buttons[j] = button;
        }

        // Get instance of pose marker object
        ImageTarget = GameObject.Find("PoseMarker");

    }

    public void SpawnRobotAndInterface()
    {
        var imTargetPosition = ImageTarget.transform.position;
        ImageTarget.GetComponent<Behaviour>().enabled = false;
        foreach (GameObject button in buttons)
        {
            button.SetActive(true);
        }

        var spawnPosition = imTargetPosition + depthOffset - heightOffset;
        baxter.transform.SetPositionAndRotation(spawnPosition, Quaternion.Euler(0,-180.0f, 0));
        baxter.SetActive(true);
    }

    public void ResetScene()
    {
        // Reset place objects colliders
        foreach (GameObject placePose in placePoses)
        {
            placePose.SetActive(true);

        }
    }

}
