using System.Collections;

using UnityEngine;
using RosMessageTypes.BaxterHrc;

public class ROSPlanInterface : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;

    // Scene objects
    public GameObject robot;
    public GameObject ground;
    public int steps = 15;

    // Variables required for ROS communication
    public string plannerServiceName = "baxter_hrc_trajectory";
    public string jointStateServiceName = "baxter_joint_states";
    public string nextActionTopicName = "next_action";

    // Offset variables for picking and placing objects
    private readonly Vector3 liftOffset = Vector3.up * 0.1f;
    private readonly Vector3 dropOffset = Vector3.up * 0.02f;
    private readonly float depthOffset = 0.135f;
    private readonly float heightOffset = 0.295f;

    // Scene objects
    private GameObject[] pickPoses;
    private GameObject[] placePoses;
    private GameObject[] tools;

    // Other scene objects (buttons, marker)
    private GameObject ImageTarget;
    private GameObject[] buttons;

    // Utility variables
    private Vector3[] pickInitialPositions;
    private Quaternion[] pickInitialRotations;
    private Vector3[] toolsInitialPositions;
    private Quaternion[] toolsInitialRotations;
    private Queue pickIdQueue;
    private Queue toolIdQueue;

    private MRTKRobotController controller;

    private Vector3[] objectsGraspOffsets =
    {
        new Vector3(0.05f, 0.005f, -0.05f),
        new Vector3(0,-0.01f,0),
        new Vector3(0,-0.01f,0),
        new Vector3(-0.03f, 0.01f, -0.06f),
        new Vector3(-0.03f, 0.01f, -0.06f),
        new Vector3(-0.03f, 0.01f, -0.06f),
        Vector3.zero,
        Vector3.zero,
    };

    private Vector3[] toolsGraspOffsets =
    {
        Vector3.forward * -0.1f,
        Vector3.forward * 0.025f,
        Vector3.zero,
    };

    void Start()
    {
        // Get ROS connection static instance
        ros = ROSConnection.instance;

        // Subscribe to baxter action topic
        ros.Subscribe<NextAction>(nextActionTopicName, PlanAction);
        
        // Instantiate Baxter Controller
        controller = gameObject.AddComponent<MRTKRobotController>();
        controller.Init(robot, steps);
        
        // Request initial joint position from real robot controller
        var request = new JointStateServiceRequest();
        ros.SendServiceMessage<JointStateServiceResponse>(jointStateServiceName, request, controller.JointStateServiceResponse);

        // Scene variable (may change depending on task)
        int n = ground.transform.childCount;
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
        for (i = 0; i < nPick; i++)
        {
            pickPoses[i] = ground.transform.GetChild(i).gameObject;
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
        for (int j = i; j < i + nPlace; j++)
        {
            placePoses[k] = ground.transform.GetChild(j).gameObject;
            k++;
        }

        tools = new GameObject[nTools];
        // Hammer
        tools[0] = ground.transform.GetChild(n - 3).gameObject;
        // Cross Screwdriver
        tools[1] = ground.transform.GetChild(n - 2).gameObject;
        // Allen key screwdriver
        tools[2] = ground.transform.GetChild(n - 1).gameObject;
        for (int z = 0; z < nTools; z++)
        {
            toolsInitialPositions[z] = tools[z].transform.localPosition;
            toolsInitialRotations[z] = tools[z].transform.localRotation;
        }

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

    public void Spawn()
    {
        var imTargetPosition = ImageTarget.transform.position;
        ImageTarget.GetComponent<Behaviour>().enabled = false;

        foreach (GameObject button in buttons)
        {
            button.SetActive(true);
        }

        var spawnPosition = imTargetPosition + Vector3.forward * depthOffset - Vector3.up * heightOffset;
        controller.SpawnRobotAndInterface(spawnPosition);
    }

    public void PlanAction(NextAction msg)
    {
        StartCoroutine(PlanActionRoutine(msg));
    }

    private IEnumerator PlanActionRoutine(NextAction msg)
    {
        int ID = msg.id;
        string op = msg.operation;
        string arm = "left";

        Vector3 pickPosition;
        Vector3 placePosition;
        Quaternion pickOrientation;
        Quaternion placeOrientation;

        if (op == "pick_and_place")
        {
            pickIdQueue.Enqueue(ID);

            // Pick Pose
            pickPosition = pickPoses[ID].transform.localPosition + liftOffset + objectsGraspOffsets[ID];
            pickOrientation = Quaternion.Euler(180, 0, 0);

            // Place Pose
            var placeObj = placePoses[0];
            if (pickPosition.x > 0)
            {
                placeObj = placePoses[1];
                arm = "right";
            }
            placePosition = placeObj.transform.localPosition + liftOffset;
            placeOrientation = pickOrientation;
            if (ID == 0)
            {
                placeOrientation = Quaternion.Euler(180, -90, 0);
            }
            else if (ID == 1 || ID == 2)
            {
                placeOrientation = Quaternion.Euler(180, 90, 0);
            }
        }
        else if (op == "tool_handover")
        {
            toolIdQueue.Enqueue(ID);

            // Pick Pose
            pickPosition = tools[ID].transform.localPosition + liftOffset + toolsGraspOffsets[ID];
            pickOrientation = Quaternion.Euler(-180.0f, 0.0f, 0.0f);

            // Handover Pose
            placePosition = placePoses[2].transform.localPosition;
            placeOrientation = Quaternion.Euler(-90.0f, 90.0f, 90.0f);
            if (pickPosition.x > 0)
            {
                arm = "right";
                placePosition = placePoses[3].transform.localPosition;
            }
        }
        else if (op == "component_handover")
        {
            pickIdQueue.Enqueue(ID);

            pickPoses[ID].transform.localPosition = pickInitialPositions[ID];
            pickPoses[ID].transform.localRotation = pickInitialRotations[ID];
            // Reactivate tool and make it physically interactable again
            pickPoses[ID].GetComponent<Rigidbody>().isKinematic = false;
            pickPoses[ID].SetActive(true);

            // Pick Pose
            pickPosition = pickPoses[ID].transform.localPosition + liftOffset + objectsGraspOffsets[ID];
            pickOrientation = Quaternion.Euler(180, 90, 0);

            // Handover Pose
            placePosition = placePoses[6].transform.localPosition;
            placeOrientation = Quaternion.Euler(180, 90, 0);
            if (pickPosition.x > 0)
            {
                arm = "right";
            }
        }
        else // Put back case
        {
            toolIdQueue.Enqueue(ID);

            var putBackPickPose = placePoses[4];
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
            pickPosition = putBackPickPose.transform.localPosition + liftOffset + toolsGraspOffsets[ID];
            pickOrientation = Quaternion.Euler(-180, 0, 0);

            // Place Pose
            placePosition = toolsInitialPositions[ID] + liftOffset + toolsGraspOffsets[ID] + dropOffset;
            placeOrientation = Quaternion.Euler(-180, 0, 0);
        }

        ActionServiceRequest request = null;
        request = controller.PlanningRequest(arm, msg.operation, pickPosition, placePosition, pickOrientation, placeOrientation);
        ros.SendServiceMessage<ActionServiceResponse>(request.arm + "_group/" + plannerServiceName, request, controller.ROSServiceResponse);

        if(arm == "left")
        {
            while (controller.leftCoroutineRunning)
            {
                yield return new WaitForSeconds(0.1f);
            }
        }
        else
        {
            while (controller.rightCoroutineRunning)
            {
                yield return new WaitForSeconds(0.1f);
            }
        }
        
        if (op == "pick_and_place" || op == "component_handover")
        {
            var currentPickID = (int)pickIdQueue.Dequeue();
            pickPoses[currentPickID].GetComponent<Rigidbody>().isKinematic = true;
            pickPoses[currentPickID].SetActive(false);
        }
        else if(op == "tool_handover" || op == "put_back")
        {
            var currentToolID = (int)toolIdQueue.Dequeue();
            tools[currentToolID].GetComponent<Rigidbody>().isKinematic = true;
            tools[currentToolID].SetActive(false);
        }
    }

    public void ResetScene()
    {
        // Revert objects to initial positions and orientations
        for (int i = 0; i < pickPoses.Length; i++)
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

        var request = new JointStateServiceRequest();
        ros.SendServiceMessage<JointStateServiceResponse>(jointStateServiceName, request, controller.JointStateServiceResponse);
    }
}