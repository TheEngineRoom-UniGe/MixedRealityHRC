using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;

using RosMessageTypes.BaxterMoveit;
using ROSGeometry;
using Quaternion = UnityEngine.Quaternion;
using Transform = UnityEngine.Transform;
using Vector3 = UnityEngine.Vector3;

using UnityEngine;

using Microsoft.MixedReality.Toolkit.UI;

public class MRTKInteractionController : MonoBehaviour
{
    // Hardcoded variables 
    private int numRobotJoints = 7;
    private readonly float jointAssignmentWaitRest = 0.01f;
    private readonly float jointAssignmentWait = 0.005f;
    private readonly float oneSecondWait = 1.0f;
    private readonly Vector3 pickPoseOffset = Vector3.up * 0.1f;
    private readonly Vector3 depthOffset = Vector3.forward * 0.15f;
    private readonly Vector3 heightOffset = Vector3.up * 0.285f;
    
    // Scene objects (robot and interactables)
    private GameObject baxter;
    private Transform ground;
    private GameObject pickObj;
    private GameObject[] placeObjs;
    private GameObject[] tools;
    private List<GameObject> waypointsList;

    // Articulation Bodies
    private ArticulationBody[] leftJointArticulationBodies;
    private ArticulationBody[] rightJointArticulationBodies;
    private ArticulationBody[] leftHand;
    private ArticulationBody[] rightHand;

    // Other scene objects (tooltips, buttons, markers)
    private IProgressIndicator indicator;
    private GameObject ImageTarget;
    private GameObject[] buttons;
    private GameObject[] tooltips;
    
    // Hardcoded variables needed for referencing joints indices
    private double[] restPosition;
    int[] leftIndices = { 4, 5, 2, 3, 6, 7, 8 };
    int[] rightIndices = { 11, 12, 9, 10, 13, 14, 15 };

    // Other utility variables
    private Vector3 lastPickPosition;
    private Transform[] startingToolsPoses;
    private int currentToolID;

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
    public TrajectoryServiceRequest PickAndPlaceService()
    {
        TrajectoryServiceRequest request = new TrajectoryServiceRequest();
        request.operation = "pickandplace";

        lastPickPosition = pickObj.transform.localPosition;
        var pickPosition = lastPickPosition + pickPoseOffset;

        string arm = "left";
        var placeObj = placeObjs[0];
        if(pickPosition.x > 0)
        {
            placeObj = placeObjs[1];
            arm = "right";
        }
        placeObj.GetComponent<SphereCollider>().enabled = false;
        var placePosition = placeObj.transform.localPosition + pickPoseOffset;

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

        // Waypoints
        int numWayPoints = waypointsList.Count;
        if(numWayPoints > 0)
        {
            request.waypoints = new RosMessageTypes.Geometry.Pose[numWayPoints];
            for (int i = 0; i < numWayPoints; i++)
            {
                waypointsList[i].GetComponent<SphereCollider>().enabled = false;
                request.waypoints[i] = new RosMessageTypes.Geometry.Pose
                {
                    position = (waypointsList[i].transform.localPosition).To<FLU>(),
                    orientation = Quaternion.Euler(180, 0, 0).To<FLU>()
                };
            }
        }

        OpenProgressIndicator();
        return request;
    }

    // Handover service request
    public TrajectoryServiceRequest HandoverService(int ID)
    {
        currentToolID = ID;
        var graspOffset = Vector3.forward;
        if (currentToolID == 0)
        {
            graspOffset *= 0.025f;
        }
        else if (currentToolID == 1)
        {
            graspOffset *= -0.1f;
        }

        TrajectoryServiceRequest request = new TrajectoryServiceRequest();
        request.operation = "handover";

        // TODO: fixed handover with left arm
        string arm = "left";
        var placeObj = placeObjs[0];

        request.arm = arm;
        request.joints_input = CurrentJointConfig(arm);
        var pickPosition = tools[ID].transform.localPosition + pickPoseOffset + graspOffset;
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

        OpenProgressIndicator();
        return request;
    }

    public void ROSServiceResponse(TrajectoryServiceResponse response)
    {
        CloseProgressIndicator();
        if (response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.Log("No trajectory returned from MoveIt.");
            StartCoroutine(ShowErrorMessage());
        }
    }

    private IEnumerator ShowErrorMessage()
    {
        yield return new WaitForSeconds(2 * oneSecondWait);
        tooltips[1].SetActive(true);
        yield return new WaitForSeconds(5 * oneSecondWait);
        tooltips[1].SetActive(false);
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
            int armIdx = 0;
            var jointArticulationBodies = leftJointArticulationBodies;
            if(arm == "right")
            {
                jointArticulationBodies = rightJointArticulationBodies;
                armIdx = 1;
            }
            yield return new WaitForSeconds(oneSecondWait);
            for (int poseIndex  = 0 ; poseIndex < response.trajectories.Length; poseIndex++)
            {
                // For every robot pose in trajectory plan
                for (int jointConfigIndex  = 0 ; jointConfigIndex < response.trajectories[poseIndex].joint_trajectory.points.Length; jointConfigIndex++)
                {
                    var jointPositions = response.trajectories[poseIndex].joint_trajectory.points[jointConfigIndex].positions;
                    float[] result = jointPositions.Select(r=> (float)r * Mathf.Rad2Deg).ToArray();
                    for (int i=0; i<=steps; i++)
                    {
                        for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
                        {
                            var joint1XDrive = jointArticulationBodies[joint].xDrive;
                            joint1XDrive.target = lastJointState[joint] + (result[joint]- lastJointState[joint]) * (1.0f/steps) * i;
                            jointArticulationBodies[joint].xDrive = joint1XDrive;
                        }

                        yield return new WaitForSeconds(jointAssignmentWait);

                    }
                    // Wait for robot to achieve pose for all joint assignments
                    lastJointState = result;
                }
                // Handle different cases based on the executed operation
                if(response.operation == "pickandplace")
                {
                    if (poseIndex == (int)Poses.PreGrasp || poseIndex == (int)Poses.Place)
                    {
                        yield return new WaitForSeconds(0.5f * oneSecondWait);
                        OpenGripper(arm);
                    }
                    // Close the gripper if completed executing the trajectory for the Grasp pose
                    else if (poseIndex == (int)Poses.Grasp)
                        CloseGripper(arm);
                }
                else if (response.operation == "handover")
                {
                    if (poseIndex == (int)Poses.Move)
                    {
                        yield return new WaitForSeconds(10 * oneSecondWait);
                        OpenGripper(arm);
                        tools[currentToolID].SetActive(false);
                    }
                    else if (poseIndex == (int)Poses.Grasp)
                        CloseGripper(arm);
                }
            }
            // All trajectories have been executed perform final operations
            if(response.operation == "pickandplace")
            {
                yield return new WaitForSeconds(oneSecondWait);
                pickObj.transform.localPosition = lastPickPosition;
            }
            else if (response.operation == "handover")
            {
                yield return new WaitForSeconds(oneSecondWait);
                placeObjs[armIdx].SetActive(true);
            }

            // Make scene objects collidable again
            placeObjs[armIdx].GetComponent<SphereCollider>().enabled = true;
            if(waypointsList.Any())
            {
                for(int i = 0; i < waypointsList.Count; i++)
                {
                    waypointsList[i].GetComponent<SphereCollider>().enabled = true;
                }
            }
            //GoToRestPosition(arm);
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
        this.ground = ground;
        this.pickObj = ground.GetChild(0).gameObject;
        this.placeObjs = new GameObject[2];
        // Left place object
        this.placeObjs[0] = ground.GetChild(1).gameObject;
        // Right place object
        this.placeObjs[1] = ground.GetChild(2).gameObject;
        this.tools = new GameObject[2];
        // Screwdriver
        this.tools[0] = ground.GetChild(3).gameObject;
        // Hammer
        this.tools[1] = ground.GetChild(4).gameObject;

        this.startingToolsPoses = new Transform[tools.Length];
        for(int i = 0; i < tools.Length; i++)
        {
            this.startingToolsPoses[i] = new GameObject().transform;
            this.startingToolsPoses[i].localPosition = tools[i].transform.localPosition;
            this.startingToolsPoses[i].rotation = tools[i].transform.rotation;
        }
        GetRobotReference();

        // Get reference to interface buttons and initialize them as inactive
        var buttonsLayer = GameObject.Find("Canvas/Buttons");
        int numButtons = buttonsLayer.transform.childCount;
        buttons = new GameObject[numButtons];
        for (int i = 0; i < numButtons; i++)
        {
            var button = buttonsLayer.transform.GetChild(i).gameObject;
            button.SetActive(false);
            buttons[i] = button;
        }

        // Get instance of pose marker object
        ImageTarget = GameObject.Find("PoseMarker");

        // Get reference to UI elements (tooltips and indicators)
        var tooltipsLayer = GameObject.Find("Canvas/Tooltips");
        int numTooltips = tooltipsLayer.transform.childCount;
        tooltips = new GameObject[numTooltips];
        for (int i = 0; i < numTooltips; i++)
        {
            var tooltip = tooltipsLayer.transform.GetChild(i).gameObject;
            tooltip.SetActive(false);
            // First indicator is progress, thus get reference to it
            if (i == 0)
            {
                indicator = tooltip.GetComponent<IProgressIndicator>();
                
            }
            // Last element is initial tooltip, so don't deactivate
            else if(i == 2) {
                tooltip.SetActive(true);
            }
            tooltips[i] = tooltip;
        }

        // Initialize waypoints list
        waypointsList = new List<GameObject>();
    }

    private async void OpenProgressIndicator()
    {
        tooltips[0].SetActive(true);
        await indicator.OpenAsync();

        indicator.Message = "Computing plan...";
        await Task.Yield();

    }

    private async void CloseProgressIndicator()
    {
        await indicator.CloseAsync();
    }

    public void SpawnRobotAndInterface()
    {
        var imTargetPosition = ImageTarget.transform.position;
        tooltips[2].SetActive(false);
        ImageTarget.GetComponent<Behaviour>().enabled = false;
        foreach (GameObject button in buttons)
        {
            button.SetActive(true);
        }

        var spawnPosition = imTargetPosition + depthOffset - heightOffset;
        baxter.transform.SetPositionAndRotation(spawnPosition, Quaternion.Euler(0,-180.0f, 0));
        baxter.SetActive(true);
    }

    public void AddWaypoint()
    {
        var WaypointPrefab = Resources.Load("WaypointPrefab");
        GameObject waypoint = (GameObject) Instantiate(WaypointPrefab, new Vector3(baxter.transform.position.x, baxter.transform.position.y, 1.5f), Quaternion.identity);
        waypoint.transform.SetParent(ground);
        waypointsList.Add(waypoint);
    }

    public void ResetScene()
    {
        // Reset place objects colliders
        foreach (GameObject placeObj in placeObjs)
        {
            placeObj.SetActive(true);
            placeObj.GetComponent<SphereCollider>().enabled = true;
        }
        // Destroy any instantiated waypoint
        foreach (GameObject waypoint in waypointsList)
        {
            Destroy(waypoint);
        }
        waypointsList.Clear();
        // Revert tools to initial positions and orientations
        for(int i = 0; i < tools.Length; i++)
        {
            tools[i].transform.localPosition = startingToolsPoses[i].localPosition;
            tools[i].transform.rotation = startingToolsPoses[i].rotation;
            tools[i].SetActive(true);
        }
    }

}
