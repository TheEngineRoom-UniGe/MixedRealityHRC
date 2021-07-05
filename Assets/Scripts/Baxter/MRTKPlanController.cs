using System.Collections;
using System.Linq;

using RosMessageTypes.BaxterMoveit;
using ROSGeometry;
using Quaternion = UnityEngine.Quaternion;
using Transform = UnityEngine.Transform;
using Vector3 = UnityEngine.Vector3;

using UnityEngine;
using Microsoft.MixedReality.Toolkit.UI;

public class MRTKPlanController : MonoBehaviour
{
    private int numRobotJoints = 7;

    // Timing variables for rendering trajectory
    private float jointAssignmentWaitRest = 0.01f;
    private float jointAssignmentWait = 0.005f;
    private float oneSecondWait = 1.0f;
    private float placeWait = 9.0f;
    private float toolHandoverPoseWait = 9.0f;
    private float componentHandoverPoseWait = 17.0f;

    // Offset variables for picking and placing objects
    private readonly Vector3 liftOffset = Vector3.up * 0.1f;
    private readonly Vector3 dropOffset = Vector3.up * 0.02f;
    private readonly float depthOffset = 0.16f;
    private readonly float heightOffset = 0.525f;

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
    private GameObject depthOffsetSlider;
    private GameObject heightOffsetSlider;
    
    // Hardcoded variables needed for referencing joints indices
    private double[] restPosition;
    int[] leftIndices = { 4, 5, 2, 3, 6, 7, 8 };
    int[] rightIndices = { 11, 12, 9, 10, 13, 14, 15 };

    // Utility variables
    Vector3[] pickInitialPositions;
    Quaternion[] pickInitialRotations;
    Vector3[] toolsInitialPositions;
    Quaternion[] toolsInitialRotations;
    Queue pickIdQueue;
    Queue toolIdQueue;

    private Vector3[] objectsGraspOffsets =
    {
        new Vector3(0.05f, 0, -0.05f),
        new Vector3(0,-0.015f,0),
        Vector3.zero,
        new Vector3(-0.03f, 0, -0.06f),
        new Vector3(-0.03f, 0, -0.06f),
        new Vector3(-0.03f, 0, -0.06f),
        Vector3.zero,
        Vector3.zero,
    };

    private Vector3[] toolsGraspOffsets =
    {
        Vector3.forward * -0.1f,
        Vector3.forward * 0.025f,
        Vector3.zero,
    };

    private enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Move,
        Place,
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

    BaxterArmJoints InitialJointConfig(string arm)
    {
        BaxterArmJoints joints = new BaxterArmJoints();
        var indices = (arm == "left") ? leftIndices : rightIndices;

        joints.angles = new double[numRobotJoints];
        for(int i = 0; i < numRobotJoints; i++)
        {
            joints.angles[i] = Mathf.Rad2Deg * (float)restPosition[indices[i]];
        }
        return joints;
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
        float[] lastJointState = {0,0,0,0,0,0,0}; 
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

    // Pick and place service request
    public TrajectoryServiceRequest PickAndPlaceService(int ID)
    {
        pickIdQueue.Enqueue(ID);
        TrajectoryServiceRequest request = new TrajectoryServiceRequest();
        request.operation = "pick_and_place";
        string arm = "left";

        // Pick Pose
        var pickPosition = pickPoses[ID].transform.localPosition + liftOffset + objectsGraspOffsets[ID];
        var pickOrientation = Quaternion.Euler(180, 0, 0).To<FLU>();
        request.pick_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (pickPosition).To<FLU>(),
            orientation = pickOrientation
        };

        // Place Pose
        var placeObj = placePoses[0];
        if (pickPosition.x > 0)
        {
            placeObj = placePoses[1];
            arm = "right";
        }
        var placePosition = placeObj.transform.localPosition + liftOffset;
        var placeOrientation = pickOrientation;
        if (ID == 1 || ID == 2)
        {
            placeOrientation = Quaternion.Euler(180, 90, 0).To<FLU>();
        }
        request.place_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (placePosition).To<FLU>(),
            orientation = placeOrientation
        };

        request.arm = arm;
        request.joints = InitialJointConfig(arm);

        return request;
    }

    public TrajectoryServiceRequest ComponentHandoverService(int ID)
    {
        pickIdQueue.Enqueue(ID);

        pickPoses[ID].transform.localPosition = pickInitialPositions[ID];
        pickPoses[ID].transform.localRotation = pickInitialRotations[ID];
        // Reactivate tool and make it physically interactable again
        pickPoses[ID].GetComponent<Rigidbody>().isKinematic = false;
        pickPoses[ID].SetActive(true);

        TrajectoryServiceRequest request = new TrajectoryServiceRequest();
        request.operation = "component_handover";
        string arm = "left";

        // Pick Pose
        var pickPosition = pickPoses[ID].transform.localPosition + liftOffset + objectsGraspOffsets[ID];
        request.pick_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (pickPosition).To<FLU>(),
            orientation = Quaternion.Euler(180, 90, 0).To<FLU>()
        };

        // Handover Pose
        if (pickPosition.x > 0)
        {
            arm = "right";
        }
        var placePosition = placePoses[6].transform.localPosition;
        request.place_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (placePosition).To<FLU>(),
            orientation = Quaternion.Euler(180, 90, 0).To<FLU>()
        };

        request.arm = arm;
        request.joints = InitialJointConfig(arm);

        return request;
    }

    // Tool handover service request
    public TrajectoryServiceRequest ToolHandoverService(int ID)
    {
        toolIdQueue.Enqueue(ID);
        TrajectoryServiceRequest request = new TrajectoryServiceRequest();
        request.operation = "tool_handover";
        string arm = "left";

        // Pick Pose
        var pickPosition = tools[ID].transform.localPosition + liftOffset + toolsGraspOffsets[ID];
        request.pick_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (pickPosition).To<FLU>(),
            orientation = Quaternion.Euler(-180.0f, 0.0f, 0.0f).To<FLU>()
        };

        // Handover Pose
        var placePosition = placePoses[2].transform.localPosition;

        if (pickPosition.x > 0)
        {
            arm = "right";
            placePosition = placePoses[3].transform.localPosition;
        }

        request.place_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (placePosition).To<FLU>(),
            orientation = Quaternion.Euler(-90.0f, 90.0f, 90.0f).To<FLU>()
        };

        request.arm = arm;
        request.joints = InitialJointConfig(arm);

        return request;
    }

    // Put object back service request
    public TrajectoryServiceRequest PutBackService(int ID)
    {
        toolIdQueue.Enqueue(ID);
        TrajectoryServiceRequest request = new TrajectoryServiceRequest();
        request.operation = "put_back";

        var putBackPickPose = placePoses[4];
        string arm = "left";

        if (toolsInitialPositions[ID].x > 0)
        {
            putBackPickPose = placePoses[5];
            arm = "right";
        }

        tools[ID].transform.localPosition = putBackPickPose.transform.localPosition;
        tools[ID].transform.localRotation = toolsInitialRotations[ID];
        // Reactivate tool and make it physically interactable again
        tools[ID].GetComponent<Rigidbody>().isKinematic = false;
        tools[ID].SetActive(true);

        // Pick Pose
        var pickPosition = putBackPickPose.transform.localPosition + liftOffset + toolsGraspOffsets[ID];
        request.pick_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (pickPosition).To<FLU>(),
            orientation = Quaternion.Euler(-180, 0, 0).To<FLU>()
        };

        // Place Pose
        var placePosition = toolsInitialPositions[ID] + liftOffset + toolsGraspOffsets[ID] + dropOffset;
        var placeOrientation = Quaternion.Euler(-180, 0, 0).To<FLU>();

        request.place_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (placePosition).To<FLU>(),
            orientation = placeOrientation
        };

        request.arm = arm;
        request.joints = InitialJointConfig(arm);

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
            var initialJointConfig = InitialJointConfig(arm);
            double[] lastJointState = initialJointConfig.angles;
            // For every trajectory plan returned
            int steps = 25;
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
                    double[] result = jointPositions.Select(r => (double)r * Mathf.Rad2Deg).ToArray();
                    for (int i = 0; i <= steps; i++)
                    {
                        for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
                        {
                            var joint1XDrive = jointArticulationBodies[joint].xDrive;
                            joint1XDrive.target = (float)(lastJointState[joint] + (result[joint] - lastJointState[joint]) * (1.0f / steps) * i);
                            jointArticulationBodies[joint].xDrive = joint1XDrive;
                        }

                        yield return new WaitForSeconds(jointAssignmentWait);

                    }
                    // Wait for robot to achieve pose for all joint assignments
                    lastJointState = result;
                }
                // Make sure gripper is open at the beginning
                if (poseIndex == (int)Poses.PreGrasp)
                {
                    OpenGripper(arm);
                }
                // Close gripper on object grasping
                if (poseIndex == (int) Poses.Grasp)
                {
                    CloseGripper(arm);
                }
                // Handle different cases based on the executed operation
                if (response.operation == "pick_and_place" && poseIndex == (int)Poses.Place)
                {            
                    yield return new WaitForSeconds(placeWait);
                    OpenGripper(arm);
                    var currentPickID = (int)pickIdQueue.Dequeue();
                    pickPoses[currentPickID].GetComponent<Rigidbody>().isKinematic = true;
                    pickPoses[currentPickID].SetActive(false);
                }
                else if (response.operation == "tool_handover" && poseIndex == (int)Poses.Move)
                {
                    yield return new WaitForSeconds(toolHandoverPoseWait);
                    OpenGripper(arm);
                    var currentToolID = (int)toolIdQueue.Dequeue();
                    tools[currentToolID].GetComponent<Rigidbody>().isKinematic = true;
                    tools[currentToolID].SetActive(false);
                }
                else if (response.operation == "component_handover")
                {
                    if (poseIndex == (int)Poses.Move)
                    {
                        yield return new WaitForSeconds(componentHandoverPoseWait);
                    }
                    else if (poseIndex == (int)Poses.Place)
                    {
                        yield return new WaitForSeconds(oneSecondWait);
                        OpenGripper(arm);
                        var currentPickID = (int)pickIdQueue.Dequeue();
                        pickPoses[currentPickID].GetComponent<Rigidbody>().isKinematic = true;
                        pickPoses[currentPickID].SetActive(false);
                    }
                }
                else if (response.operation == "put_back" && poseIndex == (int)Poses.Place)
                {
                    yield return new WaitForSeconds(placeWait);
                    OpenGripper(arm);
                    var currentToolID = (int)toolIdQueue.Dequeue();
                    tools[currentToolID].GetComponent<Rigidbody>().isKinematic = true;
                    tools[currentToolID].SetActive(false);
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
        int nPick = 6;
        int nPlace = 7;
        int nTools = 3;

        // Store initial poses for future reset of scene
        pickInitialPositions = new Vector3[nPick];
        pickInitialRotations = new Quaternion[nPick];
        toolsInitialPositions = new Vector3[nTools];
        toolsInitialRotations = new Quaternion[nTools];

        // Pick poses
        pickPoses = new GameObject[nPick];
        int i = 0;
        for(i=0; i < nPick; i++)
        {
            pickPoses[i] = ground.GetChild(i).gameObject;
            pickInitialPositions[i] = pickPoses[i].transform.localPosition;
            pickInitialRotations[i] = pickPoses[i].transform.localRotation;
        }

        /* 
         * Place poses
         * 0 -> place left
         * 1 -> place right
         * 2 -> tool handover left
         * 3 -> tool handover right
         * 4 -> put back pick up left
         * 5 -> put back pick up right
         * 6 -> component handover
         */
        placePoses = new GameObject[nPlace];
        int k = 0;
        for (int j = i; j < i+nPlace; j++)
        {
            placePoses[k] = ground.GetChild(j).gameObject;
            k++;
        }

        tools = new GameObject[nTools];
        // Hammer
        tools[0] = ground.GetChild(n-3).gameObject;
        // Cross Screwdriver
        tools[1] = ground.GetChild(n-2).gameObject;
        // Allen key screwdriver
        tools[2] = ground.GetChild(n-1).gameObject;
        for (int z = 0; z < nTools; z++)
        {
            toolsInitialPositions[z] = tools[z].transform.localPosition;
            toolsInitialRotations[z] = tools[z].transform.localRotation;
        }

        // Get reference to gameobjects that compose robot's model
        GetRobotReference();

        //Get reference to sliders and get their values
        depthOffsetSlider = GameObject.Find("Canvas/DepthOffsetSlider");
        heightOffsetSlider = GameObject.Find("Canvas/HeightOffsetSlider");

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

        //Instantiate queues of ids for correct rendering of objects
        pickIdQueue = new Queue();
        toolIdQueue = new Queue();
    }

    public void SpawnRobotAndInterface()
    {
        var imTargetPosition = ImageTarget.transform.position;
        ImageTarget.GetComponent<Behaviour>().enabled = false;

        foreach (GameObject button in buttons)
        {
            button.SetActive(true);
        }

        var spawnPosition = imTargetPosition + Vector3.forward * depthOffset - Vector3.up * heightOffset;
        baxter.transform.SetPositionAndRotation(spawnPosition, Quaternion.Euler(0,-180.0f, 0));
        baxter.SetActive(true);
    }

    public void ResetScene()
    {
        // Revert objects to initial positions and orientations
        for(int i = 0; i < pickPoses.Length; i++)
        {
            pickPoses[i].transform.localPosition = pickInitialPositions[i];
            pickPoses[i].transform.localRotation = pickInitialRotations[i];
            // Reactivate object and make it physically interactable again
            pickPoses[i].GetComponent<Rigidbody>().isKinematic = false;
            pickPoses[i].SetActive(true);
        }

        // Revert tools to initial positions and orientations
        for (int i = 0; i < tools.Length; i++)
        {
            tools[i].transform.localPosition = toolsInitialPositions[i];
            tools[i].transform.localRotation = toolsInitialRotations[i];
            // Reactivate tool and make it physically interactable again
            tools[i].GetComponent<Rigidbody>().isKinematic = false;
            tools[i].SetActive(true);
        }
    }
}