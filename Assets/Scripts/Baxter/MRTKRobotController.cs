using System.Collections;
using System.Linq;

using RosMessageTypes.BaxterHrc;
using ROSGeometry;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;

using UnityEngine;

public class MRTKRobotController : MonoBehaviour
{
    // Timing variables for rendering trajectory
    private float jointAssignmentWait = 0.005f;
    private static float placeWait = 8.0f;
    private float handoverPoseWait = 1.5f * placeWait;

    // Robot
    private GameObject baxter;
    private int numRobotJoints = 7;

    // Articulation Bodies
    private ArticulationBody[] leftJointArticulationBodies;
    private ArticulationBody[] rightJointArticulationBodies;
    private ArticulationBody[] leftHand;
    private ArticulationBody[] rightHand;

    // Hardcoded variables needed for referencing joints indices
    private double[] restPosition;
    int[] leftIndices = { 4, 5, 2, 3, 6, 7, 8 };
    int[] rightIndices = { 11, 12, 9, 10, 13, 14, 15 };

    // Utility variables
    public bool leftCoroutineRunning = false;
    public bool rightCoroutineRunning = false;
    private int steps;

    private enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Move,
        Place,
        Return
    };

    public void Init(GameObject baxter, int steps)
    {
        this.baxter = baxter;
        this.steps = steps;
        GetRobotReference();
    }

    private void GetRobotReference()
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

    public void SpawnRobotAndInterface(Vector3 spawnPosition)
    {
        baxter.transform.SetPositionAndRotation(spawnPosition, Quaternion.Euler(0, -180.0f, 0));
        baxter.SetActive(true);
    }

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

    public void JointStateServiceResponse(JointStateServiceResponse response)
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

    ArmJoints InitialJointConfig(string arm)
    {
        ArmJoints joints = new ArmJoints();
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

            yield return new WaitForSeconds(jointAssignmentWait);
        }
        OpenGripper(arm);
    }

    // Routine to generate motion planning request
    public ActionServiceRequest PlanningRequest(string arm, string op, Vector3 pickPos, Vector3 placePos, Quaternion pickOr, Quaternion placeOr)
    {
        ActionServiceRequest request = new ActionServiceRequest();
        request.operation = op;
        request.arm = arm;

        if (arm == "left")
        {
            leftCoroutineRunning = true;
        }
        else
            rightCoroutineRunning = true;

        request.pick_pose = new RosMessageTypes.Geometry.Pose
        {
            position = pickPos.To<FLU>(),
            orientation = pickOr.To<FLU>()
        };

        request.place_pose = new RosMessageTypes.Geometry.Pose
        {
            position = placePos.To<FLU>(),
            orientation = placeOr.To<FLU>()
        };

        request.joints = InitialJointConfig(arm);

        return request;
    }
 
    public void ROSServiceResponse(ActionServiceResponse response)
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

    private IEnumerator ExecuteTrajectories(ActionServiceResponse response)
    {
        if (response.trajectories != null)
        {
            var arm = response.arm;
            var initialJointConfig = InitialJointConfig(arm);
            double[] lastJointState = initialJointConfig.angles;
            // For every trajectory plan returned
            var jointArticulationBodies = leftJointArticulationBodies;
            if (arm == "right")
            {
                jointArticulationBodies = rightJointArticulationBodies;
            }
            yield return new WaitForSeconds(1.0f);
            for (int poseIndex = 0; poseIndex < response.trajectories.Length; poseIndex++)
            {
                // For every robot pose in trajectory plan
                for (int jointConfigIndex = 0; jointConfigIndex < response.trajectories[poseIndex].joint_trajectory.points.Length; jointConfigIndex++)
                {
                    var jointPositions = response.trajectories[poseIndex].joint_trajectory.points[jointConfigIndex].positions;
                    double[] result = jointPositions.Select(r => (double)r * Mathf.Rad2Deg).ToArray();
                    for (int i = 0; i <= this.steps; i++)
                    {
                        for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
                        {
                            var joint1XDrive = jointArticulationBodies[joint].xDrive;
                            joint1XDrive.target = (float)(lastJointState[joint] + (result[joint] - lastJointState[joint]) * (1.0f / this.steps) * i);
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
                    EndTrajectoryExecution(arm);
                }
                else if (response.operation == "tool_handover" && poseIndex == (int)Poses.Move)
                {
                    yield return new WaitForSeconds(placeWait);
                    EndTrajectoryExecution(arm);
                    
                }
                else if (response.operation == "component_handover")
                {
                    if (poseIndex == (int)Poses.Move)
                    {
                        yield return new WaitForSeconds(handoverPoseWait);
                    }
                    else if (poseIndex == (int)Poses.Place)
                    {
                        yield return new WaitForSeconds(1.0f);
                        EndTrajectoryExecution(arm);
                    }
                }
                else if (response.operation == "put_back" && poseIndex == (int)Poses.Place)
                {
                    yield return new WaitForSeconds(placeWait);
                    EndTrajectoryExecution(arm);
                }
            }
        }
    }

    private void EndTrajectoryExecution(string arm)
    {
        OpenGripper(arm);
        if (arm == "left")
        {
            leftCoroutineRunning = false;
        }
        else
            rightCoroutineRunning = false;
    }
}